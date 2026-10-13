"""Run Wall-E unpowered under gravity and log firmware-style IMU telemetry."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from simulation.balance_config import DEFAULT_CONFIG, ROOT, load_config, require_imu_pose
from simulation.imu_model import WallEImuModel
from simulation.run_balance import sensor_vector


DEFAULT_MODEL = ROOT / "simulation/models/wall_e_balance.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path,
                        help="CSV path (default: timestamped file under experiments/mujoco_gravity)")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if not args.model.is_file():
        parser.error(f"MuJoCo model not found: {args.model}")

    config = load_config(args.config)
    require_imu_pose(config)
    model = mujoco.MjModel.from_xml_path(str(args.model.expanduser().resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    imu = WallEImuModel(config)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    if chassis_id < 0:
        parser.error("Model is missing the 'chassis' body")

    physics_dt = float(model.opt.timestep)
    sample_dt = float(config["controller"].get("period_s", 0.01))
    substeps = round(sample_dt / physics_dt)
    if substeps < 1 or not np.isclose(substeps * physics_dt, sample_dt, rtol=0, atol=1e-9):
        parser.error(f"IMU sample period {sample_dt}s must be an integer multiple of physics timestep {physics_dt}s")
    sample_count = round(args.seconds / sample_dt)
    if sample_count < 1:
        parser.error("--seconds must cover at least one IMU sample period")

    output = args.output
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = ROOT / "experiments/mujoco_gravity" / f"imu_gravity_{stamp}.csv"
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    fields = ["time_s", "true_pitch_debug_deg", "accel_x_m_s2", "accel_y_m_s2",
              "accel_z_m_s2", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
              "accel_x_counts", "accel_y_counts", "accel_z_counts", "gyro_x_counts",
              "gyro_y_counts", "gyro_z_counts", "roll_accel_deg", "roll_estimate_deg",
              "pitch_accel_deg", "pitch_estimate_deg",
              "contact_count"]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for _ in range(sample_count):
            # Sensors are evaluated at the current simulated state. No controller
            # or motor command is applied; gravity and contact are the only forces.
            accel = sensor_vector(model, data, "imu_accel")
            gyro = sensor_vector(model, data, "imu_gyro")
            reading = imu.update(accel, gyro, sample_dt)
            rotation = data.xmat[chassis_id].reshape(3, 3)
            # The axle is chassis +X; pitch is rotation of the chassis +Y
            # forward axis toward/away from world vertical. Negating the
            # vertical component follows the firmware convention: forward
            # (nose-down) lean is negative.
            true_pitch = np.degrees(np.arctan2(
                rotation[2, 1], np.hypot(rotation[0, 1], rotation[1, 1])))
            writer.writerow({
                "time_s": data.time,
                "true_pitch_debug_deg": true_pitch,
                **{f"accel_{axis}_m_s2": accel[i] for i, axis in enumerate("xyz")},
                **{f"gyro_{axis}_rad_s": gyro[i] for i, axis in enumerate("xyz")},
                **{f"accel_{axis}_counts": reading.accel_counts[i] for i, axis in enumerate("xyz")},
                **{f"gyro_{axis}_counts": reading.gyro_counts[i] for i, axis in enumerate("xyz")},
                "roll_accel_deg": reading.roll_accel_deg,
                "roll_estimate_deg": reading.roll_deg,
                "pitch_accel_deg": reading.pitch_accel_deg,
                "pitch_estimate_deg": reading.pitch_deg,
                "contact_count": data.ncon,
            })
            data.ctrl[:] = 0.0
            for _ in range(substeps):
                mujoco.mj_step(model, data)

    print(f"MuJoCo {mujoco.__version__}: simulated {sample_count * sample_dt:.3f} s "
          f"under gravity with zero motor commands.")
    print(f"IMU samples: {sample_count} at {1 / sample_dt:.1f} Hz; CSV: {output}")
    print("Simulation only; logged IMU values are generated from the configured model and ideal sensor settings.")


if __name__ == "__main__":
    main()
