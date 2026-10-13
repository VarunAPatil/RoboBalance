"""PWM-to-torque mapping; physical mode refuses unmeasured motor parameters."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass
class WheelTorques:
    left_nm: float
    right_nm: float


class WallEMotorModel:
    REQUIRED = (
        "resistance_ohm", "torque_constant_nm_per_a", "back_emf_v_per_rad_s",
        "battery_voltage_v", "gear_ratio_motor_per_wheel", "efficiency",
        "max_wheel_torque_nm", "deadzone_duty", "no_load_current_a",
    )

    def __init__(self, config: dict[str, Any], ideal_test_torque_nm: float | None = None,
                 sign_override: tuple[int, int] | None = None):
        self.cfg = config["motor"]
        self.mode = self.cfg.get("mode")
        signs = sign_override or (self.cfg.get("left_sign"), self.cfg.get("right_sign"))
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError(
                "Wheel actuator signs are unknown. Verify each hinge's positive direction, "
                "then set motor.left_sign/right_sign; no sign is guessed."
            )
        self.left_sign, self.right_sign = signs
        self.ideal_test_torque_nm = ideal_test_torque_nm
        if self.mode == "ideal_torque_test_only":
            if ideal_test_torque_nm is None or not math.isfinite(ideal_test_torque_nm) or ideal_test_torque_nm <= 0:
                raise ValueError("Ideal mode requires a positive explicit ideal_test_torque_nm")
        elif self.mode == "dc_motor":
            dc = self.cfg.get("dc_motor", {})
            missing = [key for key in self.REQUIRED if dc.get(key) is None]
            if missing:
                raise ValueError("dc_motor mode requires measured parameters: " + ", ".join(missing))
            values = {key: float(dc[key]) for key in self.REQUIRED}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("dc_motor parameters must all be finite")
            for key in ("resistance_ohm", "torque_constant_nm_per_a", "back_emf_v_per_rad_s",
                        "battery_voltage_v", "gear_ratio_motor_per_wheel", "max_wheel_torque_nm"):
                if values[key] <= 0:
                    raise ValueError(f"dc_motor.{key} must be positive")
            if values["no_load_current_a"] < 0:
                raise ValueError("dc_motor.no_load_current_a must be non-negative")
            if not 0 < values["efficiency"] <= 1:
                raise ValueError("dc_motor.efficiency must be in (0, 1]")
            if not 0 <= values["deadzone_duty"] < 1:
                raise ValueError("dc_motor.deadzone_duty must be in [0, 1)")
            self.dc = dc
        else:
            raise ValueError("motor.mode must be ideal_torque_test_only or dc_motor")

    def pwm_to_torques(self, signed_pwm_percent: float,
                       left_wheel_speed_rad_s: float,
                       right_wheel_speed_rad_s: float) -> WheelTorques:
        if self.mode == "ideal_torque_test_only":
            torque = float(signed_pwm_percent) / 100.0 * float(self.ideal_test_torque_nm)
            return WheelTorques(self.left_sign * torque, self.right_sign * torque)
        return WheelTorques(
            self._dc_torque(signed_pwm_percent, left_wheel_speed_rad_s, self.left_sign),
            self._dc_torque(signed_pwm_percent, right_wheel_speed_rad_s, self.right_sign),
        )

    def _dc_torque(self, pwm: float, wheel_speed: float, sign: int) -> float:
        p = self.dc
        duty = max(-1.0, min(1.0, float(pwm) / 100.0))
        if abs(duty) <= float(p["deadzone_duty"]):
            duty = 0.0
        ratio = float(p["gear_ratio_motor_per_wheel"])
        # Hinge speed is converted into the motor's positive rotation frame.
        # This matters when the two wheel hinge axes point in opposite
        # directions in the MJCF.
        motor_speed = ratio * float(wheel_speed) * sign
        electrical_torque = (float(p["torque_constant_nm_per_a"]) / float(p["resistance_ohm"])) * (
            float(p["battery_voltage_v"]) * duty
            - float(p["back_emf_v_per_rad_s"]) * motor_speed
        )
        friction_torque = (float(p["torque_constant_nm_per_a"])
                           * float(p["no_load_current_a"]))
        if abs(motor_speed) > 1e-6:
            friction_direction = math.copysign(1.0, motor_speed)
        else:
            # At rest, approximate static friction as opposing the requested
            # electromagnetic torque up to breakaway, never driving the motor
            # backward when the requested torque is smaller than friction.
            friction_direction = math.copysign(1.0, electrical_torque) if electrical_torque else 0.0
            friction_torque = min(friction_torque, abs(electrical_torque))
        motor_torque = electrical_torque - friction_direction * friction_torque
        wheel_torque = sign * ratio * float(p["efficiency"]) * motor_torque
        bound = float(p["max_wheel_torque_nm"])
        return max(-bound, min(bound, wheel_torque))
