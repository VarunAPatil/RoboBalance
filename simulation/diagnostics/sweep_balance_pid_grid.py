"""Exhaustive finite-grid PID sweep at the configured simulated upright target.

This evaluates software/controller candidates on the current provisional MJCF;
it does not identify the physical TT motors or prove hardware balance.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

from simulation.balance_config import DEFAULT_CONFIG, ROOT, load_config, require_imu_pose
from simulation.motor_model import WallEMotorModel
from simulation.diagnostics.tune_balance_pid import DEFAULT_MODEL, _rank, evaluate


def grid_values(low: float, high: float, step: float, name: str) -> list[float]:
    if not all(math.isfinite(x) for x in (low, high, step)) or step <= 0 or high < low:
        raise ValueError(f"Invalid {name} grid bounds/step")
    intervals = (high - low) / step
    if not math.isclose(intervals, round(intervals), rel_tol=0, abs_tol=1e-8):
        raise ValueError(f"{name} range must be evenly divisible by its step")
    return [round(low + i * step, 10) for i in range(round(intervals) + 1)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--setpoint-deg", type=float, default=0.0)
    parser.add_argument("--kp-range", type=float, nargs=3, default=(0.0, 30.0, 0.5),
                        metavar=("MIN", "MAX", "STEP"))
    parser.add_argument("--ki-range", type=float, nargs=3, default=(0.0, 1.0, 0.05),
                        metavar=("MIN", "MAX", "STEP"))
    parser.add_argument("--kd-range", type=float, nargs=3, default=(0.0, 20.0, 0.5),
                        metavar=("MIN", "MAX", "STEP"))
    parser.add_argument("--trial-seconds", type=float, default=2.0)
    parser.add_argument("--validation-seconds", type=float, default=10.0)
    parser.add_argument("--validation-top", type=int, default=25)
    parser.add_argument("--fall-angle-deg", type=float, default=45.0)
    parser.add_argument("--pwm-output-mode", choices=("firmware_clamped", "continuous"),
                        default="firmware_clamped")
    parser.add_argument("--ideal-test-torque-nm", type=float,
                        help="Required only when the selected config uses ideal_torque_test_only")
    parser.add_argument("--left-sign", type=int, choices=(-1, 1),
                        help="Optional override; both signs must be supplied together")
    parser.add_argument("--right-sign", type=int, choices=(-1, 1),
                        help="Optional override; both signs must be supplied together")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.setpoint_deg):
        parser.error("setpoint must be finite")
    if args.trial_seconds <= 0 or args.validation_seconds <= 0 or args.validation_top < 1:
        parser.error("durations and validation-top must be positive")
    if not 0 < args.fall_angle_deg < 180:
        parser.error("fall angle must be between 0 and 180 degrees")
    try:
        kp_values = grid_values(*args.kp_range, "Kp")
        ki_values = grid_values(*args.ki_range, "Ki")
        kd_values = grid_values(*args.kd_range, "Kd")
    except ValueError as error:
        parser.error(str(error))

    config = load_config(args.config)
    require_imu_pose(config)
    if (args.left_sign is None) != (args.right_sign is None):
        parser.error("provide both --left-sign and --right-sign, or neither")
    if config["motor"].get("mode") == "ideal_torque_test_only":
        if args.ideal_test_torque_nm is None:
            parser.error("--ideal-test-torque-nm is required for ideal_torque_test_only mode")
    elif config["motor"].get("mode") == "dc_motor":
        if args.ideal_test_torque_nm is not None:
            parser.error("do not pass --ideal-test-torque-nm with the dc_motor config")
    else:
        parser.error("config motor.mode must be ideal_torque_test_only or dc_motor")
    if args.model.is_file() is False:
        parser.error(f"MuJoCo model not found: {args.model}")
    model_path = args.model.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    config["controller"]["pwm_output_mode"] = args.pwm_output_mode
    physics_dt = float(model.opt.timestep)
    control_dt = float(config["controller"].get("period_s", 0.01))
    substeps = round(control_dt / physics_dt)
    if substeps < 1 or not np.isclose(substeps * physics_dt, control_dt, rtol=0, atol=1e-9):
        parser.error("controller period must be an integer multiple of MuJoCo timestep")
    sign_override = ((args.left_sign, args.right_sign)
                     if args.left_sign is not None else None)
    motor = WallEMotorModel(config, args.ideal_test_torque_nm, sign_override)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
    actuator_ids = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                         for n in ("left_wheel_torque", "right_wheel_torque"))
    if chassis_id < 0 or site_id < 0 or min(actuator_ids) < 0:
        parser.error("model is missing chassis, IMU site, or wheel torque actuators")

    candidates = [
        {"setpoint_deg": args.setpoint_deg, "kp": kp, "ki": ki, "kd": kd}
        for kp in kp_values for ki in ki_values for kd in kd_values
    ]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.output_dir or ROOT / "experiments/mujoco_pid_grid" / stamp
    out_dir = out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    limitations = [
        "Finite grid search, not every real-valued PID combination",
        "Provisional CAD-derived model; no hardware validation",
        "Selected hinge signs are test assumptions",
    ]
    if config["motor"]["mode"] == "ideal_torque_test_only":
        limitations.append("Artificial constant torque, not identified motor response")
    else:
        limitations.append(
            "DC motor curve is an assumption inferred from supplied specs, not measured characterization")
    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "model_path": str(model_path),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "config_path": str(args.config.expanduser().resolve()),
        "mujoco_version": mujoco.__version__, "python_version": sys.version,
        "fixed_setpoint_deg": args.setpoint_deg,
        "grid": {"Kp": args.kp_range, "Ki": args.ki_range, "Kd": args.kd_range,
                 "candidate_count": len(candidates)},
        "trial_seconds": args.trial_seconds,
        "validation_seconds": args.validation_seconds,
        "validation_top": args.validation_top,
        "pwm_output_mode": args.pwm_output_mode,
        "fall_angle_deg": args.fall_angle_deg,
        "ideal_test_torque_nm": args.ideal_test_torque_nm,
        "motor_mode": config["motor"]["mode"],
        "motor_config": config["motor"],
        "left_sign": motor.left_sign, "right_sign": motor.right_sign,
        "reset_reference": {
            "angle_zero_deg": config["imu"].get("angle_zero_deg"),
            "configured_setpoint_deg": config["imu"].get("pitch_setpoint_deg"),
        },
        "objective_order": ["maximize survival", "minimize tail pitch RMS",
                            "minimize tail pitch span", "minimize final wheel speed",
                            "minimize effort"],
        "limitations": limitations,
    }
    (out_dir / "run_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    fields = ["trial", "setpoint_deg", "kp", "ki", "kd", "survival_s", "fell", "samples",
              "rms_true_pitch_error_deg", "mean_abs_true_pitch_error_deg", "pitch_span_deg",
              "peak_abs_pitch_deg", "mean_abs_pwm_percent", "minimum_contacts", "tail_window_s",
              "tail_rms_true_pitch_error_deg", "tail_pitch_span_deg",
              "final_max_abs_wheel_speed_rad_s"]
    summaries: list[dict] = []
    for index, candidate in enumerate(candidates, start=1):
        result, _ = evaluate(candidate, model, config, motor, actuator_ids, chassis_id,
                             site_id, args.trial_seconds, args.fall_angle_deg,
                             pwm_output_mode=args.pwm_output_mode)
        result["trial"] = index
        summaries.append(result)
        if index % 500 == 0 or index == len(candidates):
            top = max(summaries, key=_rank)
            print(f"{index}/{len(candidates)}; best survival={top['survival_s']:.2f}s, "
                  f"Kp={top['kp']:g}, Ki={top['ki']:g}, Kd={top['kd']:g}", flush=True)
    summaries.sort(key=_rank, reverse=True)
    with (out_dir / "grid_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)

    finalists = summaries[:min(args.validation_top, len(summaries))]
    validated = []
    for rank, coarse in enumerate(finalists, start=1):
        candidate = {key: float(coarse[key]) for key in ("setpoint_deg", "kp", "ki", "kd")}
        result, _ = evaluate(candidate, model, config, motor, actuator_ids, chassis_id,
                             site_id, args.validation_seconds, args.fall_angle_deg,
                             pwm_output_mode=args.pwm_output_mode)
        result["grid_rank"] = rank
        validated.append(result)
    validated.sort(key=_rank, reverse=True)
    validation_fields = ["grid_rank", *fields[1:]]
    with (out_dir / "validation_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=validation_fields)
        writer.writeheader()
        writer.writerows(validated)

    best = validated[0]
    best_candidate = {key: float(best[key]) for key in ("setpoint_deg", "kp", "ki", "kd")}
    best_result, trace = evaluate(best_candidate, model, config, motor, actuator_ids,
                                  chassis_id, site_id, args.validation_seconds,
                                  args.fall_angle_deg, keep_trace=True,
                                  pwm_output_mode=args.pwm_output_mode)
    summary = {"best_candidate": best_candidate, "validation": best,
               "trace_summary": best_result, "grid_count": len(candidates),
               "model_scope": "provisional simulation only",
               "artifacts": {"grid": str(out_dir / "grid_results.csv"),
                             "validation": str(out_dir / "validation_results.csv"),
                             "telemetry": str(out_dir / "best_candidate_telemetry.csv")}}
    (out_dir / "best_candidate.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    if trace:
        with (out_dir / "best_candidate_telemetry.csv").open(
                "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
            writer.writeheader()
            writer.writerows(trace)
    print("Best finite-grid candidate, simulation only:")
    print(json.dumps(summary, indent=2))
    print(f"Full grid: {out_dir / 'grid_results.csv'}")


if __name__ == "__main__":
    main()
