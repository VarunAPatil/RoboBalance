"""Explore balance setpoints and record a MuJoCo preview plus IMU telemetry.

This is a diagnostic sweep, not a hardware-tuning tool. It deliberately
requires an explicit ideal torque scale and wheel signs because those motor
properties have not been characterized.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from contextlib import nullcontext
import hashlib
import json
import os
from pathlib import Path
import sys

# Prefer offscreen EGL for reproducible recordings, including when a stale
# DISPLAY variable exists but no desktop X server is reachable.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from simulation.balance_config import DEFAULT_CONFIG, ROOT, load_config, require_imu_pose
from simulation.balance_controller import BalanceController
from simulation.imu_model import WallEImuModel
from simulation.motor_model import WallEMotorModel
from simulation.run_balance import joint_speed, sensor_vector


DEFAULT_MODEL = ROOT / "simulation/models/wall_e_balance.xml"


def true_forward_pitch_deg(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> float:
    """Chassis forward-lean angle; forward/nose-down is negative by convention."""
    rotation = data.xmat[body_id].reshape(3, 3)
    return float(np.degrees(np.arctan2(
        rotation[2, 1], np.hypot(rotation[0, 1], rotation[1, 1]))))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seconds", type=float, default=2.0,
                        help="duration for each setpoint trial (default: 2 s)")
    parser.add_argument("--start", type=float, default=23.0)
    parser.add_argument("--stop", type=float, default=30.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--no-preview", action="store_true",
                        help="skip GIF rendering (useful for wide sweeps)")
    parser.add_argument("--ideal-test-torque-nm", type=float, required=True,
                        help="explicit artificial torque scale; not a measured motor value")
    parser.add_argument("--left-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--right-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0 or args.step <= 0 or args.stop < args.start:
        parser.error("seconds and step must be positive, and stop must be >= start")
    if not args.model.is_file():
        parser.error(f"MuJoCo model not found: {args.model}")

    config = load_config(args.config)
    require_imu_pose(config)
    model = mujoco.MjModel.from_xml_path(str(args.model.expanduser().resolve()))
    physics_dt = float(model.opt.timestep)
    control_dt = float(config["controller"].get("period_s", 0.01))
    substeps = round(control_dt / physics_dt)
    if substeps < 1 or not np.isclose(substeps * physics_dt, control_dt, rtol=0, atol=1e-9):
        parser.error("controller period must be an integer multiple of the MuJoCo timestep")
    samples_per_trial = round(args.seconds / control_dt)
    if samples_per_trial < 1:
        parser.error("each trial must cover at least one controller sample")
    setpoints = np.arange(args.start, args.stop + args.step * 0.5, args.step)
    if not np.isclose(setpoints[-1], args.stop, atol=1e-9):
        parser.error("the requested range must land exactly on the stop value using step")

    motor = WallEMotorModel(config, args.ideal_test_torque_nm,
                            (args.left_sign, args.right_sign))
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
               for name in ("left_wheel_torque", "right_wheel_torque")]
    if chassis_id < 0 or min(act_ids) < 0:
        parser.error("model is missing the chassis or named wheel actuators")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = args.output_dir or ROOT / "experiments/mujoco_setpoint_sweep" / stamp
    out_dir = out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "model_path": str(args.model.expanduser().resolve()),
        "model_sha256": hashlib.sha256(args.model.expanduser().resolve().read_bytes()).hexdigest(),
        "config_path": str(args.config.expanduser().resolve()),
        "config": config,
        "python_version": sys.version,
        "mujoco_version": mujoco.__version__,
        "numpy_version": np.__version__,
        "sweep": {
            "start_deg": args.start, "stop_deg": args.stop, "step_deg": args.step,
            "trial_seconds": args.seconds, "ideal_test_torque_nm": args.ideal_test_torque_nm,
            "left_sign": args.left_sign, "right_sign": args.right_sign,
            "preview_rendered": not args.no_preview,
            "imu_random_seed": config["imu"].get("random_seed"),
            "samples_per_trial": samples_per_trial,
            "stability_screen": {
                "mean_absolute_pitch_error_last_0_5s_deg_lt": 2.0,
                "pitch_span_last_0_5s_deg_lt": 3.0,
                "gyro_saturation_samples_eq": 0,
                "minimum_contact_count_gt": 0,
            },
        },
    }
    (out_dir / "run_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    csv_path = out_dir / "telemetry.csv"
    gif_path = out_dir / "setpoint_sweep.gif"
    summary_path = out_dir / "summary.csv"
    fields = ["setpoint_deg", "time_s", "true_forward_pitch_deg", "roll_accel_deg",
              "roll_estimate_deg", "pitch_accel_deg", "pitch_estimate_deg", "pitch_error_deg",
              "gyro_y_counts", "signed_pwm_percent",
              "left_torque_nm", "right_torque_nm", "left_wheel_rad_s", "right_wheel_rad_s",
              "contact_count"]
    summary_fields = ["setpoint_deg", "stable_last_0_5s", "mean_pitch_error_last_0_5s_deg",
                      "pitch_span_last_0_5s_deg", "gyro_saturation_samples", "min_contact_count",
                      "max_abs_pitch_deg"]

    frames: list[Image.Image] = []
    summaries: list[dict[str, object]] = []
    width, height = 800, 600
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "three_quarter")
    if camera_id < 0:
        parser.error("model does not have the three_quarter camera")

    renderer_context = (nullcontext(None) if args.no_preview else
                        mujoco.Renderer(model, width=width, height=height))
    with renderer_context as renderer, \
            csv_path.open("w", newline="", encoding="utf-8") as telemetry_stream:
        writer = csv.DictWriter(telemetry_stream, fieldnames=fields)
        writer.writeheader()
        for trial_index, setpoint in enumerate(setpoints, start=1):
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            imu = WallEImuModel(config)
            controller = BalanceController(config)
            trial_rows: list[dict[str, float | int]] = []
            for sample in range(samples_per_trial):
                accel = sensor_vector(model, data, "imu_accel")
                gyro = sensor_vector(model, data, "imu_gyro")
                reading = imu.update(accel, gyro, control_dt)
                output = controller.step(reading.pitch_deg, float(setpoint), control_dt)
                torque = motor.pwm_to_torques(
                    output.signed_pwm_percent,
                    joint_speed(model, data, "wheel_left_hinge"),
                    joint_speed(model, data, "wheel_right_hinge"),
                )
                data.ctrl[act_ids[0]], data.ctrl[act_ids[1]] = torque.left_nm, torque.right_nm
                for _ in range(substeps):
                    mujoco.mj_step(model, data)

                row: dict[str, float | int] = {
                    "setpoint_deg": float(setpoint), "time_s": float(data.time),
                    "true_forward_pitch_deg": true_forward_pitch_deg(model, data, chassis_id),
                    "roll_accel_deg": reading.roll_accel_deg,
                    "roll_estimate_deg": reading.roll_deg,
                    "pitch_accel_deg": reading.pitch_accel_deg,
                    "pitch_estimate_deg": reading.pitch_deg,
                    "pitch_error_deg": output.error_deg,
                    "gyro_y_counts": int(reading.gyro_counts[1]),
                    "signed_pwm_percent": output.signed_pwm_percent,
                    "left_torque_nm": torque.left_nm, "right_torque_nm": torque.right_nm,
                    "left_wheel_rad_s": joint_speed(model, data, "wheel_left_hinge"),
                    "right_wheel_rad_s": joint_speed(model, data, "wheel_right_hinge"),
                    "contact_count": int(data.ncon),
                }
                writer.writerow(row)
                trial_rows.append(row)

                # One preview frame every 0.1 s keeps the recording compact.
                if renderer is not None and (sample % max(1, round(0.1 / control_dt)) == 0
                                             or sample == samples_per_trial - 1):
                    renderer.update_scene(data, camera=camera_id)
                    frame = Image.fromarray(renderer.render()).convert("RGB")
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((0, 0, width, 54), fill=(0, 0, 0))
                    draw.text((14, 10),
                              f"Setpoint {setpoint:.1f} deg | trial {trial_index}/{len(setpoints)} "
                              f"| t={data.time:.2f}s | pitch={row['true_forward_pitch_deg']:.1f} deg",
                              fill=(255, 255, 255))
                    frames.append(frame)

            tail = [r for r in trial_rows if r["time_s"] >= args.seconds - 0.5]
            errors = np.asarray([float(r["pitch_error_deg"]) for r in tail])
            pitches = np.asarray([float(r["true_forward_pitch_deg"]) for r in tail])
            pitch_span = float(np.ptp(pitches))
            gyro_saturation = sum(abs(int(r["gyro_y_counts"])) >= 32767 for r in trial_rows)
            min_contacts = min(int(r["contact_count"]) for r in tail)
            stable = (float(np.mean(np.abs(errors))) < 2.0 and pitch_span < 3.0
                      and gyro_saturation == 0 and min_contacts > 0)
            summary = {
                "setpoint_deg": float(setpoint), "stable_last_0_5s": stable,
                "mean_pitch_error_last_0_5s_deg": float(np.mean(errors)),
                "pitch_span_last_0_5s_deg": pitch_span,
                "gyro_saturation_samples": gyro_saturation,
                "min_contact_count": min_contacts,
                "max_abs_pitch_deg": max(abs(float(r["true_forward_pitch_deg"])) for r in trial_rows),
            }
            summaries.append(summary)
            print(f"{setpoint:5.1f} deg: stable={stable}, "
                  f"final-window mean error={summary['mean_pitch_error_last_0_5s_deg']:.1f} deg, "
                  f"pitch span={pitch_span:.1f} deg, gyro saturation samples={gyro_saturation}",
                  flush=True)

    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summaries)
    if frames:
        frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=100, loop=0)

    print(f"Saved sweep telemetry: {csv_path}")
    print(f"Saved per-setpoint summary: {summary_path}")
    if frames:
        print(f"Saved recorded MuJoCo preview: {gif_path}")
    else:
        print("Preview rendering skipped")
    print("Diagnostic only: the ideal torque scale and signs are test inputs, not measured motor values.")


if __name__ == "__main__":
    main()
