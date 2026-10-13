"""Run the provisional MuJoCo sensor/controller/actuator loop and log telemetry."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import mujoco
import numpy as np
import yaml

from simulation.balance_config import DEFAULT_CONFIG, ROOT, load_config, require_imu_pose
from simulation.balance_controller import BalanceController
from simulation.imu_model import G_M_S2, WallEImuModel
from simulation.motor_model import WallEMotorModel


DEFAULT_MODEL = ROOT / "simulation/models/wall_e_balance.xml"


def sensor_vector(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise ValueError(f"Required sensor '{name}' is missing from MJCF")
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != 3:
        raise ValueError(f"Sensor '{name}' must be 3-axis; compiled dimension is {dimension}")
    return data.sensordata[address:address + dimension].copy()


def joint_speed(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"Required joint '{name}' is missing from MJCF")
    return float(data.qvel[int(model.jnt_dofadr[joint_id])])


def gravity_specific_force_in_site_frame(site_rotation: np.ndarray) -> np.ndarray:
    """Return the rest-pose gravity vector expressed in a MuJoCo site frame."""
    rotation = np.asarray(site_rotation, dtype=float).reshape(3, 3)
    return rotation.T @ np.array([0.0, 0.0, G_M_S2])


def chassis_attitude(rotation: np.ndarray) -> tuple[float, float, float]:
    """Return firmware-sign pitch, roll, and total tilt from a body rotation."""
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
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
    parser.add_argument("--seconds", type=float,
                        help="Run duration; --tune runs until a window is closed when omitted")
    parser.add_argument("--viewer", action="store_true", help="Open passive MuJoCo viewer")
    parser.add_argument("--tune", action="store_true",
                        help="Open MuJoCo plus a live PID/setpoint slider panel")
    parser.add_argument("--setpoint-deg", type=float,
                        help="Override imu.pitch_setpoint_deg for this run")
    parser.add_argument("--kp", type=float, help="Override controller.kp for this run")
    parser.add_argument("--ki", type=float, help="Override controller.ki for this run")
    parser.add_argument("--kd", type=float, help="Override controller.kd for this run")
    parser.add_argument("--pwm-output-mode", choices=("continuous", "firmware_clamped"),
                        help="Motor effort mapping; --tune defaults to continuous, other runs to firmware_clamped")
    parser.add_argument("--fall-angle-deg", type=float,
                        help="Reset the episode at this total chassis tilt (default: config value)")
    parser.add_argument("--ideal-test-torque-nm", type=float,
                        help="Explicit arbitrary test scale; not a measured TT motor value")
    parser.add_argument("--left-sign", type=int, choices=(-1, 1))
    parser.add_argument("--right-sign", type=int, choices=(-1, 1))
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.tune:
        args.viewer = True
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            parser.error("No desktop display available; --tune requires a graphical session")
    duration_s = args.seconds if args.seconds is not None else (None if args.tune else 5.0)
    config = load_config(args.config)
    if args.pwm_output_mode is not None:
        pwm_output_mode = args.pwm_output_mode
    elif args.tune:
        pwm_output_mode = "continuous"
    else:
        pwm_output_mode = str(config["controller"].get("pwm_output_mode", "firmware_clamped"))
    config["controller"]["pwm_output_mode"] = pwm_output_mode
    for option, section, key in (
        (args.setpoint_deg, "imu", "pitch_setpoint_deg"),
        (args.kp, "controller", "kp"),
        (args.ki, "controller", "ki"),
        (args.kd, "controller", "kd"),
    ):
        if option is not None:
            if not math.isfinite(option):
                parser.error(f"{key} override must be finite")
            config[section][key] = float(option)
    fall_angle = float(args.fall_angle_deg if args.fall_angle_deg is not None
                       else config.get("simulation", {}).get("fall_angle_deg", 45.0))
    if not math.isfinite(fall_angle) or not 0 < fall_angle < 180:
        parser.error("--fall-angle-deg must be finite and between 0 and 180 degrees")
    require_imu_pose(config)
    if not args.model.is_file():
        parser.error(f"Balance MJCF not found: {args.model}. Configure the IMU pose and run simulation.build_balance_model first.")

    sign_override = None
    if args.left_sign is not None or args.right_sign is not None:
        if args.left_sign is None or args.right_sign is None:
            parser.error("provide both --left-sign and --right-sign")
        sign_override = (args.left_sign, args.right_sign)
    motor = WallEMotorModel(config, args.ideal_test_torque_nm, sign_override)
    model = mujoco.MjModel.from_xml_path(str(args.model.expanduser().resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    imu = WallEImuModel(config)
    imu_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
    if imu_site_id < 0:
        raise ValueError("Balance MJCF lacks the named imu_site")
    imu_seed_pitch_deg, _ = imu.seed_from_gravity(
        gravity_specific_force_in_site_frame(data.site_xmat[imu_site_id]))
    controller = BalanceController(config)
    left_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_torque")
    right_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_torque")
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    if min(left_act, right_act, chassis_id) < 0:
        raise ValueError("Balance MJCF lacks chassis or named wheel torque actuators")

    physics_dt = float(model.opt.timestep)
    control_dt = float(config["controller"].get("period_s", 0.01))
    substeps = round(control_dt / physics_dt)
    if substeps < 1 or not np.isclose(substeps * physics_dt, control_dt, rtol=0, atol=1e-9):
        raise ValueError(f"Controller period {control_dt}s must be an integer multiple of MuJoCo timestep {physics_dt}s")

    run_dir = args.output_dir or ROOT / "experiments/mujoco_balance" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "config.yaml").write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    (run_dir / "run.json").write_text(json.dumps({
        "mujoco_version": mujoco.__version__, "model": str(args.model.resolve()),
        "physics_timestep_s": physics_dt, "control_period_s": control_dt,
        "requested_duration_s": duration_s,
        "imu_random_seed": config["imu"].get("random_seed", 42),
        "controller_mode": config["controller"].get("mode"),
        "pwm_output_mode": pwm_output_mode,
        "imu_initialization": "gravity projection of MJCF reset-pose orientation; subsequent samples use full accelerometer",
        "reset_pose_source": "MuJoCo model qpos0 (CAD-derived chassis frame)",
        "pitch_setpoint_deg": config["imu"].get("pitch_setpoint_deg", 9.0),
        "kp": config["controller"]["kp"], "ki": config["controller"]["ki"],
        "kd": config["controller"]["kd"], "fall_angle_deg": fall_angle,
        "live_tuning": args.tune,
        "angle_zero_calibrated": config["imu"].get("angle_zero_deg") is not None,
        "motor_mode": motor.mode,
        "ideal_test_torque_nm": args.ideal_test_torque_nm,
        "hardware_validation": False,
    }, indent=2) + "\n", encoding="utf-8")

    log_path = run_dir / "telemetry.csv"
    fields = ["time_s", "episode", "episode_time_s", "fall_reset", "reset_reason",
              "imu_seed_pitch_deg",
              "true_pitch_debug_deg",
              "true_roll_debug_deg", "total_tilt_deg", "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
              "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "accel_x_counts", "accel_y_counts",
              "accel_z_counts", "gyro_x_counts", "gyro_y_counts", "gyro_z_counts",
              "roll_accel_deg", "roll_estimate_deg", "pitch_accel_deg", "pitch_estimate_deg",
              "pitch_setpoint_deg", "kp", "ki", "kd", "pitch_error_deg", "controller_command",
              "motor_cmd_magnitude", "motor_direction", "signed_pwm_percent",
              "left_wheel_rad_s", "right_wheel_rad_s",
              "left_torque_nm", "right_torque_nm"]
    control_steps = None if duration_s is None else max(1, round(duration_s / control_dt))
    latest_reading = None
    latest_output = None
    latest_torques = (0.0, 0.0)
    viewer_context = None
    if args.viewer:
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            parser.error("No desktop display available; omit --viewer for headless simulation")
        from mujoco import viewer as mujoco_viewer
        viewer_context = mujoco_viewer.launch_passive(model, data)

    tuner = None
    if args.tune:
        from simulation.balance_tuner import BalanceTuner
        tuner = BalanceTuner(
            float(config["imu"].get("pitch_setpoint_deg", 9.0)),
            float(config["controller"]["kp"]),
            float(config["controller"]["ki"]),
            float(config["controller"]["kd"]),
            fall_angle,
            pwm_output_mode,
        )

    reset_count = 0
    fall_reset_count = 0
    manual_reset_count = 0
    try:
        with log_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            episode = 0
            episode_step = 0
            control_index = 0
            pending_reset_reason = ""
            while control_steps is None or control_index < control_steps:
                if tuner is not None:
                    tuner.pump()
                    if tuner.closed:
                        break
                    values = tuner.parameters()
                    config["imu"]["pitch_setpoint_deg"] = values["setpoint_deg"]
                    for key in ("kp", "ki", "kd"):
                        config["controller"][key] = values[key]
                    fall_angle = values["fall_angle_deg"]
                    config.setdefault("simulation", {})["fall_angle_deg"] = fall_angle
                    if tuner.consume_reset_request():
                        data.ctrl[:] = 0.0
                        mujoco.mj_resetData(model, data)
                        mujoco.mj_forward(model, data)
                        imu.reset()
                        imu_seed_pitch_deg, _ = imu.seed_from_gravity(
                            gravity_specific_force_in_site_frame(data.site_xmat[imu_site_id]))
                        controller.reset()
                        episode += 1
                        episode_step = 0
                        reset_count += 1
                        manual_reset_count += 1
                        pending_reset_reason = "manual"
                    if tuner.paused:
                        viewer_context.sync()
                        if not viewer_context.is_running():
                            break
                        time.sleep(0.02)
                        continue

                accel = sensor_vector(model, data, "imu_accel")
                gyro = sensor_vector(model, data, "imu_gyro")
                latest_reading = imu.update(accel, gyro, control_dt)
                setpoint = float(config["imu"].get("pitch_setpoint_deg", 9.0))
                latest_output = controller.step(latest_reading.pitch_deg, setpoint, control_dt)
                torques = motor.pwm_to_torques(
                    latest_output.signed_pwm_percent,
                    joint_speed(model, data, "wheel_left_hinge"),
                    joint_speed(model, data, "wheel_right_hinge"),
                )
                latest_torques = (torques.left_nm, torques.right_nm)
                data.ctrl[left_act], data.ctrl[right_act] = latest_torques

                for _ in range(substeps):
                    mujoco.mj_step(model, data)
                    if viewer_context is not None:
                        viewer_context.sync()
                        if not viewer_context.is_running():
                            break
                        time.sleep(0.001)
                if viewer_context is not None and not viewer_context.is_running():
                    break

                rotation = data.xmat[chassis_id].reshape(3, 3)
                true_pitch, true_roll, total_tilt = chassis_attitude(rotation)
                did_fall = total_tilt >= fall_angle
                writer.writerow({
                    "time_s": (control_index + 1) * control_dt,
                    "episode": episode,
                    "episode_time_s": (episode_step + 1) * control_dt,
                    "fall_reset": int(did_fall),
                    "reset_reason": "fall" if did_fall else pending_reset_reason,
                    "imu_seed_pitch_deg": imu_seed_pitch_deg,
                    "true_pitch_debug_deg": true_pitch,
                    "true_roll_debug_deg": true_roll,
                    "total_tilt_deg": total_tilt,
                    **{f"accel_{axis}_m_s2": accel[i] for i, axis in enumerate("xyz")},
                    **{f"gyro_{axis}_rad_s": gyro[i] for i, axis in enumerate("xyz")},
                    **{f"accel_{axis}_counts": latest_reading.accel_counts[i] for i, axis in enumerate("xyz")},
                    **{f"gyro_{axis}_counts": latest_reading.gyro_counts[i] for i, axis in enumerate("xyz")},
                    "roll_accel_deg": latest_reading.roll_accel_deg,
                    "roll_estimate_deg": latest_reading.roll_deg,
                    "pitch_accel_deg": latest_reading.pitch_accel_deg,
                    "pitch_estimate_deg": latest_reading.pitch_deg,
                    "pitch_setpoint_deg": setpoint,
                    "kp": config["controller"]["kp"],
                    "ki": config["controller"]["ki"],
                    "kd": config["controller"]["kd"],
                    "pitch_error_deg": latest_output.error_deg,
                    "controller_command": latest_output.command,
                    "motor_cmd_magnitude": abs(latest_output.command),
                    "motor_direction": ("BACKWARD" if latest_output.signed_pwm_percent < 0
                                        else "FORWARD" if latest_output.signed_pwm_percent > 0
                                        else "STOP"),
                    "signed_pwm_percent": latest_output.signed_pwm_percent,
                    "left_wheel_rad_s": joint_speed(model, data, "wheel_left_hinge"),
                    "right_wheel_rad_s": joint_speed(model, data, "wheel_right_hinge"),
                    "left_torque_nm": latest_torques[0], "right_torque_nm": latest_torques[1],
                })
                if did_fall:
                    reset_count += 1
                    fall_reset_count += 1
                    episode += 1
                    episode_step = 0
                    data.ctrl[:] = 0.0
                    mujoco.mj_resetData(model, data)
                    mujoco.mj_forward(model, data)
                    imu.reset()
                    imu_seed_pitch_deg, _ = imu.seed_from_gravity(
                        gravity_specific_force_in_site_frame(data.site_xmat[imu_site_id]))
                    controller.reset()
                    pending_reset_reason = ""
                else:
                    episode_step += 1
                    pending_reset_reason = ""
                control_index += 1
                if viewer_context is not None:
                    viewer_context.sync()
    finally:
        if tuner is not None and not tuner.closed:
            tuner.close()
        if viewer_context is not None:
            viewer_context.close()
    (run_dir / "config_final.yaml").write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    run_metadata_path = run_dir / "run.json"
    run_metadata = json.loads(run_metadata_path.read_text(encoding="utf-8"))
    run_metadata.update({
        "final_pitch_setpoint_deg": config["imu"].get("pitch_setpoint_deg", 9.0),
        "final_kp": config["controller"]["kp"],
        "final_ki": config["controller"]["ki"],
        "final_kd": config["controller"]["kd"],
        "final_fall_angle_deg": fall_angle,
        "fall_reset_count": fall_reset_count,
        "manual_reset_count": manual_reset_count,
    })
    run_metadata_path.write_text(json.dumps(run_metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Run complete (simulation only; {fall_reset_count} fall resets, "
          f"{manual_reset_count} manual resets). Telemetry: {log_path}")


if __name__ == "__main__":
    main()
