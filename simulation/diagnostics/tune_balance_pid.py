"""Search PID/setpoint settings on the provisional MuJoCo balance plant.

This is an offline simulation experiment, not a hardware tuning utility. It
keeps the motor torque/sign fixture fixed and scores survival, pitch tracking,
contact, and effort for each deterministic candidate.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import sys

import mujoco
import numpy as np

from simulation.balance_config import DEFAULT_CONFIG, ROOT, load_config, require_imu_pose
from simulation.balance_controller import BalanceController
from simulation.imu_model import WallEImuModel
from simulation.motor_model import WallEMotorModel
from simulation.run_balance import (gravity_specific_force_in_site_frame,
                                    joint_speed, sensor_vector)


DEFAULT_MODEL = ROOT / "simulation/models/wall_e_balance.xml"


def make_candidates(count: int, seed: int, setpoint_min: float,
                    setpoint_max: float, kp_min: float, kp_max: float,
                    ki_max: float, kd_min: float, kd_max: float) -> list[dict[str, float]]:
    rng = random.Random(seed)
    presets = [
        {"setpoint_deg": 0.0, "kp": 2.0, "ki": 0.0, "kd": 0.2},
        {"setpoint_deg": 9.0, "kp": 11.0, "ki": 0.0, "kd": 4.0},
        {"setpoint_deg": 23.0, "kp": 2.0, "ki": 0.0, "kd": 0.2},
        {"setpoint_deg": 30.0, "kp": 2.0, "ki": 0.0, "kd": 0.2},
        {"setpoint_deg": 36.0, "kp": 2.0, "ki": 0.0, "kd": 0.2},
        {"setpoint_deg": 25.9113236997, "kp": 4.125225286,
         "ki": 0.0235318932, "kd": 7.877558314},
    ]
    candidates = [c for c in presets
                  if setpoint_min <= c["setpoint_deg"] <= setpoint_max
                  and kp_min <= c["kp"] <= kp_max
                  and c["ki"] <= ki_max and (c["kd"] == 0 or kd_min <= c["kd"] <= kd_max)][:count]
    while len(candidates) < count:
        candidates.append({
            "setpoint_deg": rng.uniform(setpoint_min, setpoint_max),
            "kp": 10 ** rng.uniform(math.log10(kp_min), math.log10(kp_max)),
            "ki": 0.0 if rng.random() < 0.35 else 10 ** rng.uniform(-4.0, math.log10(ki_max)),
            "kd": 0.0 if rng.random() < 0.15 else 10 ** rng.uniform(math.log10(kd_min), math.log10(kd_max)),
        })
    return candidates


def evaluate(candidate: dict[str, float], model: mujoco.MjModel,
             config: dict, motor: WallEMotorModel,
             actuator_ids: tuple[int, int], chassis_id: int,
             imu_site_id: int, seconds: float, fall_angle_deg: float,
             keep_trace: bool = False,
             pwm_output_mode: str = "continuous") -> tuple[dict, list[dict]]:
    trial_config = json.loads(json.dumps(config))
    trial_config["controller"].update(
        kp=candidate["kp"], ki=candidate["ki"], kd=candidate["kd"],
        pwm_output_mode=pwm_output_mode,
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    imu = WallEImuModel(trial_config)
    seed_pitch, _ = imu.seed_from_gravity(
        gravity_specific_force_in_site_frame(data.site_xmat[imu_site_id]))
    controller = BalanceController(trial_config)
    physics_dt = float(model.opt.timestep)
    control_dt = float(trial_config["controller"].get("period_s", 0.01))
    substeps = round(control_dt / physics_dt)
    steps = max(1, round(seconds / control_dt))
    angle_zero = trial_config["imu"].get("angle_zero_deg")
    target_pitch = candidate["setpoint_deg"] - (0.0 if angle_zero is None else float(angle_zero))
    errors: list[float] = []
    pitches: list[float] = []
    efforts: list[float] = []
    contacts: list[int] = []
    wheel_speeds: list[float] = []
    trace: list[dict] = []
    fall_time = seconds
    fell = False

    for step in range(steps):
        accel = sensor_vector(model, data, "imu_accel")
        gyro = sensor_vector(model, data, "imu_gyro")
        reading = imu.update(accel, gyro, control_dt)
        output = controller.step(reading.pitch_deg, candidate["setpoint_deg"], control_dt)
        torque = motor.pwm_to_torques(
            output.signed_pwm_percent,
            joint_speed(model, data, "wheel_left_hinge"),
            joint_speed(model, data, "wheel_right_hinge"),
        )
        data.ctrl[actuator_ids[0]], data.ctrl[actuator_ids[1]] = torque.left_nm, torque.right_nm
        for _ in range(substeps):
            mujoco.mj_step(model, data)

        pitch, roll, tilt = _attitude(data.xmat[chassis_id])
        errors.append(pitch - target_pitch)
        pitches.append(pitch)
        efforts.append(abs(output.signed_pwm_percent))
        contacts.append(int(data.ncon))
        left_speed = joint_speed(model, data, "wheel_left_hinge")
        right_speed = joint_speed(model, data, "wheel_right_hinge")
        wheel_speeds.append(max(abs(left_speed), abs(right_speed)))
        if keep_trace:
            trace.append({
                "time_s": float(data.time), "true_pitch_deg": pitch,
                "true_roll_deg": roll, "total_tilt_deg": tilt,
                "imu_seed_pitch_deg": seed_pitch,
                "pitch_accel_deg": reading.pitch_accel_deg,
                "pitch_estimate_deg": reading.pitch_deg,
                "setpoint_deg": candidate["setpoint_deg"],
                "pitch_error_deg": output.error_deg,
                "pid_command": output.command,
                "signed_pwm_percent": output.signed_pwm_percent,
                "left_torque_nm": torque.left_nm,
                "right_torque_nm": torque.right_nm,
                "left_wheel_rad_s": left_speed,
                "right_wheel_rad_s": right_speed,
                "contact_count": int(data.ncon),
            })
        if tilt >= fall_angle_deg:
            fell = True
            fall_time = float(data.time)
            break

    pitch_array = np.asarray(pitches, dtype=float)
    tail_samples = max(1, round(min(1.0, seconds * 0.25) / control_dt))
    tail_errors = np.asarray(errors[-tail_samples:], dtype=float)
    tail_pitches = pitch_array[-tail_samples:]
    summary = {
        **candidate,
        "survival_s": fall_time,
        "fell": fell,
        "samples": len(errors),
        "rms_true_pitch_error_deg": float(np.sqrt(np.mean(np.square(errors)))),
        "mean_abs_true_pitch_error_deg": float(np.mean(np.abs(errors))),
        "pitch_span_deg": float(np.ptp(pitch_array)) if len(pitch_array) else 180.0,
        "peak_abs_pitch_deg": float(np.max(np.abs(pitch_array))) if len(pitch_array) else 180.0,
        "mean_abs_pwm_percent": float(np.mean(efforts)) if efforts else 100.0,
        "minimum_contacts": min(contacts) if contacts else 0,
        "tail_window_s": len(tail_errors) * control_dt,
        "tail_rms_true_pitch_error_deg": float(np.sqrt(np.mean(np.square(tail_errors)))) if len(tail_errors) else 180.0,
        "tail_pitch_span_deg": float(np.ptp(tail_pitches)) if len(tail_pitches) else 180.0,
        "final_max_abs_wheel_speed_rad_s": float(wheel_speeds[-1]) if wheel_speeds else math.inf,
    }
    return summary, trace


def _attitude(rotation_flat: np.ndarray) -> tuple[float, float, float]:
    rotation = np.asarray(rotation_flat, dtype=float).reshape(3, 3)
    pitch = math.degrees(math.atan2(
        rotation[2, 1], math.hypot(rotation[0, 1], rotation[1, 1])))
    roll = math.degrees(math.atan2(
        -rotation[2, 0], math.hypot(rotation[2, 1], rotation[2, 2])))
    tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1.0, 1.0))))
    return pitch, roll, tilt


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--trials", type=int, default=250)
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="maximum seconds per candidate before it is scored")
    parser.add_argument("--validation-seconds", type=float, default=5.0,
                        help="independent-duration rerun for the top candidates")
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20261013)
    parser.add_argument("--setpoint-min-deg", type=float, default=-10.0)
    parser.add_argument("--setpoint-max-deg", type=float, default=50.0)
    parser.add_argument("--kp-min", type=float, default=0.05)
    parser.add_argument("--kp-max", type=float, default=30.0)
    parser.add_argument("--ki-max", type=float, default=1.0)
    parser.add_argument("--kd-min", type=float, default=0.01)
    parser.add_argument("--kd-max", type=float, default=20.0)
    parser.add_argument("--fall-angle-deg", type=float, default=45.0)
    parser.add_argument("--ideal-test-torque-nm", type=float, required=True,
                        help="fixed artificial torque scale; not a measured motor parameter")
    parser.add_argument("--left-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--right-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.trials < 1 or args.top < 1 or args.seconds <= 0 or args.validation_seconds <= 0:
        parser.error("trials/top must be positive and durations must be positive")
    if args.setpoint_max_deg < args.setpoint_min_deg:
        parser.error("setpoint max must be >= setpoint min")
    if not (0 < args.kp_min < args.kp_max and args.ki_max > 0
            and 0 < args.kd_min < args.kd_max):
        parser.error("gain bounds must be positive and each min must be below its max")
    if not math.isfinite(args.fall_angle_deg) or not 0 < args.fall_angle_deg < 180:
        parser.error("fall angle must be finite and between 0 and 180 degrees")
    if not args.model.is_file():
        parser.error(f"MuJoCo model not found: {args.model}")

    config = load_config(args.config)
    require_imu_pose(config)
    config["controller"]["pwm_output_mode"] = "continuous"
    model = mujoco.MjModel.from_xml_path(str(args.model.expanduser().resolve()))
    physics_dt = float(model.opt.timestep)
    control_dt = float(config["controller"].get("period_s", 0.01))
    substeps = round(control_dt / physics_dt)
    if substeps < 1 or not np.isclose(substeps * physics_dt, control_dt, rtol=0, atol=1e-9):
        parser.error("controller period must be an integer multiple of the MuJoCo timestep")
    if round(args.seconds / control_dt) < 1 or round(args.validation_seconds / control_dt) < 1:
        parser.error("trial durations must cover at least one controller period")

    motor = WallEMotorModel(
        config, args.ideal_test_torque_nm, (args.left_sign, args.right_sign))
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
    actuator_ids = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                         for name in ("left_wheel_torque", "right_wheel_torque"))
    if chassis_id < 0 or site_id < 0 or min(actuator_ids) < 0:
        parser.error("model is missing chassis, IMU site, or wheel torque actuators")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.output_dir or ROOT / "experiments/mujoco_pid_search" / stamp
    out_dir = out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    candidates = make_candidates(args.trials, args.seed,
                                 args.setpoint_min_deg, args.setpoint_max_deg,
                                 args.kp_min, args.kp_max, args.ki_max,
                                 args.kd_min, args.kd_max)
    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "model_path": str(args.model.expanduser().resolve()),
        "model_sha256": hashlib.sha256(args.model.expanduser().resolve().read_bytes()).hexdigest(),
        "config_path": str(args.config.expanduser().resolve()),
        "mujoco_version": mujoco.__version__, "python_version": sys.version,
        "search_seed": args.seed,
        "objective_order": ["maximize survival_s", "minimize tail_rms_true_pitch_error_deg",
                            "minimize tail_pitch_span_deg", "minimize final_max_abs_wheel_speed_rad_s",
                            "minimize mean_abs_pwm_percent"],
        "limitations": [
            "CAD/screenshot-derived model, not hardware validated",
            "continuous PWM magnitude mode, not firmware-clamped behavior",
            "fixed artificial ideal torque and explicitly supplied wheel signs",
            "no measured motor response, calibrated contact, or chassis impact geometry",
        ],
        "search": {
            "trials": args.trials, "seconds_per_trial": args.seconds,
            "validation_seconds": args.validation_seconds, "top_candidates": args.top,
            "setpoint_range_deg": [args.setpoint_min_deg, args.setpoint_max_deg],
            "gain_ranges": {"kp": [args.kp_min, args.kp_max], "ki": [0.0, args.ki_max],
                            "kd": [0.0, args.kd_max]},
            "fall_angle_deg": args.fall_angle_deg,
            "ideal_test_torque_nm": args.ideal_test_torque_nm,
            "left_sign": args.left_sign, "right_sign": args.right_sign,
            "continuous_effort": True,
        },
    }
    (out_dir / "run_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    summaries = []
    for index, candidate in enumerate(candidates, start=1):
        result, _ = evaluate(candidate, model, config, motor, actuator_ids,
                             chassis_id, site_id, args.seconds, args.fall_angle_deg)
        result["trial"] = index
        summaries.append(result)
        if index % 25 == 0 or index == len(candidates):
            best = max(summaries, key=_rank)
            print(f"{index}/{len(candidates)} trials; best survival={best['survival_s']:.2f}s, "
                  f"setpoint={best['setpoint_deg']:.2f}, Kp={best['kp']:.4g}, "
                  f"Ki={best['ki']:.4g}, Kd={best['kd']:.4g}", flush=True)

    summaries.sort(key=_rank, reverse=True)
    summary_path = out_dir / "search_results.csv"
    fields = ["trial", "setpoint_deg", "kp", "ki", "kd", "survival_s", "fell", "samples",
              "rms_true_pitch_error_deg", "mean_abs_true_pitch_error_deg", "pitch_span_deg",
              "peak_abs_pitch_deg", "mean_abs_pwm_percent", "minimum_contacts",
              "tail_window_s", "tail_rms_true_pitch_error_deg", "tail_pitch_span_deg",
              "final_max_abs_wheel_speed_rad_s"]
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    finalists = summaries[:min(args.top, len(summaries))]
    validated = []
    for rank, coarse in enumerate(finalists, start=1):
        candidate = {key: float(coarse[key]) for key in ("setpoint_deg", "kp", "ki", "kd")}
        result, _ = evaluate(candidate, model, config, motor, actuator_ids,
                             chassis_id, site_id, args.validation_seconds,
                             args.fall_angle_deg)
        result["coarse_rank"] = rank
        validated.append(result)
    validated.sort(key=_rank, reverse=True)
    best_candidate = {key: float(validated[0][key])
                      for key in ("setpoint_deg", "kp", "ki", "kd")}
    best_summary, best_trace = evaluate(
        best_candidate, model, config, motor, actuator_ids, chassis_id, site_id,
        args.validation_seconds, args.fall_angle_deg, keep_trace=True)
    best_summary.update({"candidate": best_candidate, "validation": validated[0],
                         "model_scope": "provisional simulation only"})
    (out_dir / "best_candidate.json").write_text(
        json.dumps(best_summary, indent=2) + "\n", encoding="utf-8")
    with (out_dir / "validation_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "coarse_rank", "setpoint_deg", "kp", "ki", "kd", "survival_s", "fell", "samples",
            "rms_true_pitch_error_deg", "mean_abs_true_pitch_error_deg", "pitch_span_deg",
            "peak_abs_pitch_deg", "mean_abs_pwm_percent", "minimum_contacts", "tail_window_s",
            "tail_rms_true_pitch_error_deg", "tail_pitch_span_deg",
            "final_max_abs_wheel_speed_rad_s"])
        writer.writeheader()
        writer.writerows(validated)
    if best_trace:
        with (out_dir / "best_candidate_telemetry.csv").open(
                "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(best_trace[0]))
            writer.writeheader()
            writer.writerows(best_trace)

    print("Best validated candidate (this simulated setup only):")
    print(json.dumps(best_summary, indent=2))
    print(f"Saved trial table: {summary_path}")
    print(f"Saved best candidate: {out_dir / 'best_candidate.json'}")
    print("Do not transfer these values to the physical bot without measured motor/model validation.")


def _rank(row: dict) -> tuple[float, float, float, float, float]:
    return (float(row["survival_s"]),
            -float(row["tail_rms_true_pitch_error_deg"]),
            -float(row["tail_pitch_span_deg"]),
            -float(row["final_max_abs_wheel_speed_rad_s"]),
            -float(row["mean_abs_pwm_percent"]))


if __name__ == "__main__":
    main()
