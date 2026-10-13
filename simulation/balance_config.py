"""Load and validate the explicitly incomplete-to-measure Wall-E balance config."""

from __future__ import annotations

from pathlib import Path
import math
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs/mujoco_balance.yaml"


def load_config(path: Path | str = DEFAULT_CONFIG) -> dict[str, Any]:
    with Path(path).expanduser().resolve().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    for section in ("imu", "controller", "motor"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Missing required '{section}' mapping in {path}")
    return config


def require_imu_pose(config: dict[str, Any]) -> tuple[list[float], list[float]]:
    imu = config["imu"]
    pos, quat = imu.get("site_pos_m"), imu.get("site_quat_wxyz")
    if pos is None or quat is None:
        raise ValueError(
            "IMU mounting pose is not configured. Measure the MPU6050 site position "
            "in meters and orientation as a normalized MuJoCo wxyz quaternion, then "
            "set imu.site_pos_m and imu.site_quat_wxyz in configs/mujoco_balance.yaml. "
            "No pose is inferred or guessed by this builder."
        )
    if len(pos) != 3 or len(quat) != 4:
        raise ValueError("IMU site pose must have 3 position and 4 wxyz quaternion values")
    pos = [float(value) for value in pos]
    quat = [float(value) for value in quat]
    if not all(math.isfinite(value) for value in pos + quat):
        raise ValueError("IMU site pose values must all be finite")
    norm = sum(value * value for value in quat) ** 0.5
    if not 0.999 <= norm <= 1.001:
        raise ValueError(f"IMU site quaternion must be normalized; got norm={norm:.6g}")
    return pos, quat


def require_axis_mapping(section: dict[str, Any], name: str) -> tuple[list[int], list[int]]:
    order = section.get(f"{name}_axis_order")
    signs = section.get(f"{name}_axis_signs")
    if order is None or signs is None or sorted(order) != [0, 1, 2]:
        raise ValueError(f"{name} axis order must be a permutation of [0, 1, 2]")
    if len(signs) != 3 or any(value not in (-1, 1) for value in signs):
        raise ValueError(f"{name} axis signs must contain three values, each -1 or +1")
    return list(order), list(signs)
