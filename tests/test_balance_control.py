import math

import numpy as np

from simulation.balance_controller import BalanceController
from simulation.imu_model import G_M_S2, WallEImuModel
from simulation.motor_model import WallEMotorModel
from simulation.run_balance import chassis_attitude


def controller(kp=1.0, ki=0.0, kd=0.0):
    return BalanceController({"controller": {
        "mode": "firmware_compatible",
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "command_limit": 90.0,
        "error_deadband_deg": 1.0,
        "pwm_min_percent": 60.0,
        "pwm_max_percent": 80.0,
    }})


def test_direction_follows_pitch_error_even_if_pid_correction_sign_differs():
    pid = controller(kp=1.0, kd=20.0)
    pid.step(pitch_deg=0.0, setpoint_deg=3.0, dt=0.01)
    output = pid.step(pitch_deg=0.5, setpoint_deg=3.0, dt=0.01)

    assert output.error_deg > 1.0
    assert output.command < 0.0  # derivative made the PID correction negative
    assert output.signed_pwm_percent == -60.0  # firmware still commands BACKWARD


def test_active_error_with_zero_pid_output_still_gets_firmware_minimum_pwm():
    output = controller(kp=0.0).step(pitch_deg=0.0, setpoint_deg=2.0, dt=0.01)
    assert output.command == 0.0
    assert output.signed_pwm_percent == -60.0


def test_continuous_effort_mode_scales_pwm_with_pid_output():
    def continuous(kp):
        cfg = {"controller": {
            "mode": "firmware_compatible", "pwm_output_mode": "continuous",
            "kp": kp, "ki": 0.0, "kd": 0.0, "command_limit": 90.0,
            "error_deadband_deg": 1.0, "pwm_min_percent": 60.0,
            "pwm_max_percent": 80.0,
        }}
        return BalanceController(cfg).step(pitch_deg=0.0, setpoint_deg=5.0, dt=0.01)

    assert continuous(0.0).signed_pwm_percent == 0.0
    assert continuous(2.0).signed_pwm_percent == -10.0
    assert continuous(5.0).signed_pwm_percent == -25.0


def test_imu_gravity_seed_ignores_reset_time_acceleration_spike():
    config = {"imu": {
        "accel_axis_order": [0, 1, 2], "accel_axis_signs": [1, 1, 1],
        "gyro_axis_order": [0, 1, 2], "gyro_axis_signs": [1, 1, 1],
        "accel_counts_per_g": 16384.0, "gyro_counts_per_dps": 131.0,
        "complementary_alpha": 0.995, "angle_zero_deg": None,
        "gyro_integer_truncation": True, "random_seed": 42,
        "accel_bias_m_s2": [0.0, 0.0, 0.0],
        "gyro_bias_rad_s": [0.0, 0.0, 0.0],
        "accel_noise_std_m_s2": [0.0, 0.0, 0.0],
        "gyro_noise_std_rad_s": [0.0, 0.0, 0.0],
    }}
    imu = WallEImuModel(config)
    seed_pitch, _ = imu.seed_from_gravity(np.array([0.0, 0.0, G_M_S2]))
    reading = imu.update(
        np.array([-6.7479, 0.0023, 8.1188]), np.zeros(3), 0.01)

    assert abs(seed_pitch) < 0.01
    assert abs(reading.pitch_accel_deg - 39.7) < 0.2
    assert abs(reading.pitch_deg) < 0.3


def test_firmware_deadband_stops_motors():
    output = controller().step(pitch_deg=0.0, setpoint_deg=1.0, dt=0.01)
    assert output.signed_pwm_percent == 0.0


def test_mpu_gyro_arithmetic_defaults_to_firmware_integer_truncation():
    config = {"imu": {
        "accel_axis_order": [0, 1, 2], "accel_axis_signs": [1, 1, 1],
        "gyro_axis_order": [0, 1, 2], "gyro_axis_signs": [1, 1, 1],
        "accel_counts_per_g": 16384.0, "gyro_counts_per_dps": 131.0,
        "complementary_alpha": 0.995, "angle_zero_deg": None,
        "gyro_integer_truncation": True,
        "random_seed": 42,
    }}
    imu = WallEImuModel(config)
    reading = imu.update(
        np.array([0.0, 0.0, G_M_S2]),
        np.array([0.0, math.radians(0.8), 0.0]),
        0.01,
    )
    assert reading.pitch_rate_dps == 0.0


def test_total_tilt_detects_pitch_or_roll_falls():
    pitch_30 = math.radians(30.0)
    pitch_rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(pitch_30), -math.sin(pitch_30)],
        [0.0, math.sin(pitch_30), math.cos(pitch_30)],
    ])
    pitch, _, tilt = chassis_attitude(pitch_rotation)
    assert pitch > 0.0
    assert abs(tilt - 30.0) < 1e-9

    roll_50 = math.radians(50.0)
    roll_rotation = np.array([
        [math.cos(roll_50), 0.0, math.sin(roll_50)],
        [0.0, 1.0, 0.0],
        [-math.sin(roll_50), 0.0, math.cos(roll_50)],
    ])
    _, _, tilt = chassis_attitude(roll_rotation)
    assert abs(tilt - 50.0) < 1e-9


def test_assumed_n20_motor_curve_matches_stall_and_no_load_endpoints():
    motor = WallEMotorModel({
        "motor": {
            "mode": "dc_motor", "left_sign": 1, "right_sign": -1,
            "dc_motor": {
                "resistance_ohm": 16.0,
                "torque_constant_nm_per_a": 0.25582565217391307,
                "back_emf_v_per_rad_s": 0.5271211715203573,
                "battery_voltage_v": 11.8,
                "gear_ratio_motor_per_wheel": 1.0,
                "efficiency": 1.0,
                "max_wheel_torque_nm": 0.1765197,
                "deadzone_duty": 0.0,
                "no_load_current_a": 0.06,
            },
        },
    })
    at_stall = motor.pwm_to_torques(100.0, 0.0, 0.0)
    assert abs(at_stall.left_nm - 0.17332) < 2e-4
    assert abs(at_stall.right_nm + at_stall.left_nm) < 1e-12

    free_speed = (11.8 - 16.0 * 0.06) / 0.5271211715203573
    at_free_speed = motor.pwm_to_torques(100.0, free_speed, -free_speed)
    assert abs(at_free_speed.left_nm) < 1e-12
    assert abs(at_free_speed.right_nm) < 1e-12

    braking = motor.pwm_to_torques(0.0, 100.0 * 2.0 * math.pi / 60.0,
                                   -100.0 * 2.0 * math.pi / 60.0)
    assert braking.left_nm < 0.0
    assert braking.right_nm > 0.0

    # Static friction may cancel a sub-breakaway command, but cannot create
    # reverse driving torque. Check both wheel signs and command directions.
    for pwm in (-8.0, -4.0, -1.0, 0.0, 1.0, 4.0, 8.0):
        stopped = motor.pwm_to_torques(pwm, 0.0, 0.0)
        assert stopped.left_nm == stopped.right_nm == 0.0
    for pwm in (-10.0, 10.0):
        moving = motor.pwm_to_torques(pwm, 0.0, 0.0)
        assert moving.left_nm * pwm > 0
        assert moving.right_nm * pwm < 0
