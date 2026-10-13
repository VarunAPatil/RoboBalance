"""Standalone pitch controller, with faithful and time-correct variants."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class ControllerOutput:
    error_deg: float
    command: float
    signed_pwm_percent: float


class BalanceController:
    def __init__(self, config: dict):
        self.cfg = config["controller"]
        self.previous_error = 0.0
        self.integral = 0.0

    def reset(self) -> None:
        self.previous_error = 0.0
        self.integral = 0.0

    def step(self, pitch_deg: float, setpoint_deg: float, dt: float) -> ControllerOutput:
        if dt <= 0 or not math.isfinite(dt):
            raise ValueError("Controller dt must be finite and positive")
        c = self.cfg
        error = float(setpoint_deg - pitch_deg)
        if c.get("mode", "firmware_compatible") == "firmware_compatible":
            # Mirrors self_balancing.c: sum(error) and delta(error), both per sample.
            self.integral += error
            derivative = error - self.previous_error
            integral_limit = 850.0
            derivative_limit = 850.0
        elif c.get("mode") == "time_corrected":
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt
            integral_limit = float(c.get("integral_limit_deg_s", 850.0))
            derivative_limit = float(c.get("derivative_limit_deg_s", 850.0))
        else:
            raise ValueError("controller.mode must be firmware_compatible or time_corrected")

        raw = (float(c["kp"]) * error
               + float(c["ki"]) * max(-integral_limit, min(integral_limit, self.integral))
               + float(c["kd"]) * max(-derivative_limit, min(derivative_limit, derivative)))
        limit = float(c.get("command_limit", 90.0))
        command = max(-limit, min(limit, raw))
        deadband = float(c.get("error_deadband_deg", 1.0))
        if abs(error) <= deadband:
            pwm = 0.0
        else:
            pwm_max = float(c.get("pwm_max_percent", 80.0))
            magnitude = min(pwm_max, abs(command))
            output_mode = c.get("pwm_output_mode", "firmware_clamped")
            if output_mode == "firmware_clamped":
                magnitude = max(float(c.get("pwm_min_percent", 60.0)), magnitude)
            elif output_mode != "continuous":
                raise ValueError("controller.pwm_output_mode must be firmware_clamped or continuous")
            # Firmware computes abs(PID correction) for duty magnitude, but
            # chooses MOTOR_BACKWARD/FORWARD from pitch_error itself.
            # Positive signed PWM represents MOTOR_FORWARD in motor_model.py,
            # so positive error must map to a negative (backward) command.
            pwm = -magnitude if error > deadband else magnitude
        self.previous_error = error
        return ControllerOutput(error, command, pwm)
