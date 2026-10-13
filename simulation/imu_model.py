"""MPU6050-like sensor conversion and the Wall-E complementary attitude estimate."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simulation.balance_config import require_axis_mapping


G_M_S2 = 9.80665


def si_to_raw_counts(accel_m_s2: np.ndarray, gyro_rad_s: np.ndarray,
                     accel_counts_per_g: float = 16384.0,
                     gyro_counts_per_dps: float = 131.0) -> tuple[np.ndarray, np.ndarray]:
    """Quantize SI sensor values to MPU6050-like signed integer counts."""
    if accel_counts_per_g <= 0 or gyro_counts_per_dps <= 0:
        raise ValueError("MPU count scales must be positive")
    accel = np.rint(np.asarray(accel_m_s2, dtype=float) * accel_counts_per_g / G_M_S2)
    gyro = np.rint(np.degrees(np.asarray(gyro_rad_s, dtype=float)) * gyro_counts_per_dps)
    if accel.shape != (3,) or gyro.shape != (3,):
        raise ValueError("Expected three-axis acceleration and angular velocity vectors")
    # The configured MPU ranges saturate at signed 16-bit register limits.
    return (np.clip(accel, -32768, 32767).astype(np.int64),
            np.clip(gyro, -32768, 32767).astype(np.int64))


def firmware_pitch_accel_deg(accel_counts: np.ndarray) -> float:
    ax, ay, az = np.asarray(accel_counts, dtype=float)
    return math.degrees(math.atan2(-ax, math.hypot(ay, az)))


def firmware_roll_accel_deg(accel_counts: np.ndarray) -> float:
    ax, ay, az = np.asarray(accel_counts, dtype=float)
    return math.degrees(math.atan2(ay, math.hypot(ax, az)))


@dataclass
class ImuReading:
    accel_counts: np.ndarray
    gyro_counts: np.ndarray
    roll_accel_deg: float
    roll_rate_dps: float
    roll_deg: float
    pitch_accel_deg: float
    pitch_rate_dps: float
    pitch_deg: float


class WallEImuModel:
    """Consumes MuJoCo site sensors and emits firmware-style roll/pitch readings."""

    def __init__(self, config: dict, rng: np.random.Generator | None = None):
        self.cfg = config["imu"]
        self.accel_order, self.accel_signs = require_axis_mapping(self.cfg, "accel")
        self.gyro_order, self.gyro_signs = require_axis_mapping(self.cfg, "gyro")
        self.rng = rng or np.random.default_rng(int(self.cfg.get("random_seed", 42)))
        self.roll_deg: float | None = None
        self.pitch_deg: float | None = None
        self.startup_elapsed_s = 0.0

    def reset(self) -> None:
        """Clear filter state at an episode reset while retaining RNG sequence."""
        self.roll_deg = None
        self.pitch_deg = None
        self.startup_elapsed_s = 0.0

    def seed_from_gravity(self, gravity_specific_force_site_m_s2: np.ndarray) -> tuple[float, float]:
        """Seed roll/pitch from pose gravity, excluding reset-time linear acceleration.

        MuJoCo's accelerometer includes translational and angular acceleration.
        Immediately after an episode reset, those terms can be large even though
        the robot is being treated as freshly placed in its reset pose. This
        gravity-only seed models the held-at-reset initial attitude; subsequent
        samples use the complete simulated accelerometer as usual.
        """
        accel = np.asarray(gravity_specific_force_site_m_s2, dtype=float)
        if accel.shape != (3,) or not np.isfinite(accel).all():
            raise ValueError("Expected finite three-axis gravity specific-force vector")
        accel = accel - np.asarray(self.cfg.get("accel_bias_m_s2", [0, 0, 0]), dtype=float)
        accel += self.rng.normal(0, self.cfg.get("accel_noise_std_m_s2", [0, 0, 0]))
        accel = accel[self.accel_order] * np.asarray(self.accel_signs)
        accel_counts, _ = si_to_raw_counts(
            accel,
            np.zeros(3),
            float(self.cfg.get("accel_counts_per_g", 16384.0)),
            float(self.cfg.get("gyro_counts_per_dps", 131.0)),
        )
        self.pitch_deg = firmware_pitch_accel_deg(accel_counts)
        self.roll_deg = firmware_roll_accel_deg(accel_counts)
        angle_zero = self.cfg.get("angle_zero_deg")
        if angle_zero is not None:
            self.pitch_deg -= float(angle_zero)
        self.startup_elapsed_s = 0.0
        return self.pitch_deg, self.roll_deg

    def update(self, accel_m_s2: np.ndarray, gyro_rad_s: np.ndarray, dt: float) -> ImuReading:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError(f"dt must be finite and positive, got {dt}")
        accel = np.asarray(accel_m_s2, dtype=float)
        gyro = np.asarray(gyro_rad_s, dtype=float)
        if accel.shape != (3,) or gyro.shape != (3,) or not np.isfinite(accel).all() or not np.isfinite(gyro).all():
            raise ValueError("Expected finite three-axis MuJoCo sensor vectors")

        accel = accel - np.asarray(self.cfg.get("accel_bias_m_s2", [0, 0, 0]), dtype=float)
        gyro = gyro - np.asarray(self.cfg.get("gyro_bias_rad_s", [0, 0, 0]), dtype=float)
        accel += self.rng.normal(0, self.cfg.get("accel_noise_std_m_s2", [0, 0, 0]))
        gyro += self.rng.normal(0, self.cfg.get("gyro_noise_std_rad_s", [0, 0, 0]))
        accel = accel[self.accel_order] * np.asarray(self.accel_signs)
        gyro = gyro[self.gyro_order] * np.asarray(self.gyro_signs)

        accel_counts, gyro_counts = si_to_raw_counts(
            accel, gyro,
            float(self.cfg.get("accel_counts_per_g", 16384.0)),
            float(self.cfg.get("gyro_counts_per_dps", 131.0)),
        )
        pitch_accel = firmware_pitch_accel_deg(accel_counts)
        roll_accel = firmware_roll_accel_deg(accel_counts)
        gyro_x = int(gyro_counts[0])
        gyro_y = int(gyro_counts[1])
        if bool(self.cfg.get("gyro_integer_truncation", True)):
            # C int16 / int literal division truncates toward zero.
            pitch_rate = math.trunc(gyro_y / 131)
            roll_rate = math.trunc(gyro_x / 131)
        else:
            gyro_scale = float(self.cfg.get("gyro_counts_per_dps", 131.0))
            pitch_rate = gyro_y / gyro_scale
            roll_rate = gyro_x / gyro_scale

        angle_zero = self.cfg.get("angle_zero_deg")
        offset = 0.0 if angle_zero is None else float(angle_zero)
        alpha = float(self.cfg.get("complementary_alpha", 0.995))
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("complementary_alpha must be in [0, 1]")
        if self.roll_deg is None:
            self.roll_deg = roll_accel
        else:
            self.roll_deg = alpha * (self.roll_deg + roll_rate * dt) + (1 - alpha) * roll_accel
        if self.pitch_deg is None:
            self.pitch_deg = pitch_accel - offset
        else:
            self.pitch_deg = alpha * (self.pitch_deg + pitch_rate * dt) + (1 - alpha) * (pitch_accel - offset)

        # Empirical, pose-specific startup settling seen in fused hardware logs.
        # This is deliberately separate from the complementary filter and from
        # raw sensor bias: its cause is unknown, and the three captures were in
        # one static, non-balancing pose. Start the clock at the first sample.
        startup_tau = float(self.cfg.get("startup_transient_tau_s", 0.0))
        if startup_tau < 0 or not math.isfinite(startup_tau):
            raise ValueError("startup_transient_tau_s must be finite and nonnegative")
        if bool(self.cfg.get("startup_transient_enabled", False)) and startup_tau > 0:
            decay = math.exp(-self.startup_elapsed_s / startup_tau)
            roll_output = self.roll_deg + float(self.cfg.get("startup_roll_seed_error_deg", 0.0)) * decay
            pitch_output = self.pitch_deg + float(self.cfg.get("startup_pitch_seed_error_deg", 0.0)) * decay
        else:
            roll_output, pitch_output = self.roll_deg, self.pitch_deg
        self.startup_elapsed_s += dt
        return ImuReading(
            accel_counts=accel_counts,
            gyro_counts=gyro_counts,
            roll_accel_deg=roll_accel,
            roll_rate_dps=roll_rate,
            roll_deg=roll_output,
            pitch_accel_deg=pitch_accel,
            pitch_rate_dps=pitch_rate,
            pitch_deg=pitch_output,
        )

    def calibrate_angle_zero(self, upright_pitch_samples_deg: list[float]) -> float:
        if len(upright_pitch_samples_deg) < 2:
            raise ValueError("Collect at least two pitch samples while the robot is held upright")
        if not np.isfinite(upright_pitch_samples_deg).all():
            raise ValueError("Upright pitch calibration samples must all be finite")
        offset = float(np.mean(upright_pitch_samples_deg))
        self.cfg["angle_zero_deg"] = offset
        self.pitch_deg = None
        self.startup_elapsed_s = 0.0
        return offset
