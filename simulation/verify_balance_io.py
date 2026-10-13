"""Verify sensor/actuator plumbing with explicit synthetic test fixtures only."""

from __future__ import annotations

import copy
from pathlib import Path
import tempfile

import mujoco
import numpy as np

from simulation.balance_config import DEFAULT_CONFIG, load_config
from simulation.build_balance_model import build_balance_model
from simulation.imu_model import (G_M_S2, WallEImuModel, firmware_pitch_accel_deg,
                                  si_to_raw_counts)
from simulation.motor_model import WallEMotorModel
from simulation.run_balance import (gravity_specific_force_in_site_frame,
                                    sensor_vector)


def main() -> None:
    config = load_config(DEFAULT_CONFIG)
    assert config["imu"].get("angle_zero_deg") == 0.0
    assert config["imu"].get("pitch_setpoint_deg") == 0.0
    unconfigured = copy.deepcopy(config)
    unconfigured["imu"]["site_pos_m"] = None
    unconfigured["imu"]["site_quat_wxyz"] = None
    try:
        build_balance_model(unconfigured)
    except ValueError as error:
        assert "IMU mounting pose is not configured" in str(error)
    else:
        raise AssertionError("Builder must reject an unconfigured real IMU pose")

    fixture = copy.deepcopy(config)
    # Test-only frame: identity sensor axes at the chassis origin. This is not
    # asserted to be the physical board mounting pose.
    fixture["imu"]["site_pos_m"] = [0.0, 0.0, 0.0]
    fixture["imu"]["site_quat_wxyz"] = [1.0, 0.0, 0.0, 0.0]
    # Keep the basic firmware-filter plumbing test free of the separate,
    # empirical startup-output transient configured for hardware replay.
    fixture["imu"]["startup_roll_seed_error_deg"] = 0.0
    fixture["imu"]["startup_pitch_seed_error_deg"] = 0.0
    fixture["imu"]["startup_transient_tau_s"] = 0.0
    temp_path = None
    try:
        # Regression check for the live model's reset-time acceleration spike:
        # seed from the reset pose's gravity direction, then blend in the real
        # dynamic sample rather than treating that sample as the initial tilt.
        reset_model = mujoco.MjModel.from_xml_path(
            str((DEFAULT_CONFIG.parent.parent / "simulation/models/wall_e_balance.xml").resolve()))
        reset_data = mujoco.MjData(reset_model)
        mujoco.mj_forward(reset_model, reset_data)
        reset_site_id = mujoco.mj_name2id(reset_model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        reset_imu = WallEImuModel(config)
        reset_seed, _ = reset_imu.seed_from_gravity(
            gravity_specific_force_in_site_frame(reset_data.site_xmat[reset_site_id]))
        assert abs(reset_seed - config["imu"]["pitch_setpoint_deg"]) < 0.1
        reset_reading = reset_imu.update(
            sensor_vector(reset_model, reset_data, "imu_accel"),
            sensor_vector(reset_model, reset_data, "imu_gyro"), 0.01)
        assert abs(reset_seed) < 0.1
        assert reset_reading.pitch_accel_deg > 30.0  # raw reset acceleration remains visible
        assert abs(reset_reading.pitch_deg) < 0.5    # fused estimate starts at reset-pose pitch

        with tempfile.NamedTemporaryFile(prefix=".wall_e_balance_test_", suffix=".xml",
                                         dir=DEFAULT_CONFIG.parent.parent / "simulation/models",
                                         delete=False) as temporary:
            temp_path = Path(temporary.name)
        build_balance_model(fixture, output=temp_path)
        model = mujoco.MjModel.from_xml_path(str(temp_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        assert model.nu == 2 and model.nsensor == 2
        assert np.isfinite(data.sensordata).all()
        assert sensor_vector(model, data, "imu_accel").shape == (3,)
        assert sensor_vector(model, data, "imu_gyro").shape == (3,)

        left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_torque")
        right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_torque")
        assert left_id >= 0 and right_id >= 0
        # Exercise each direct-drive actuator independently; the control is a
        # test torque in N*m, not a motor characterization result.
        for actuator_id, joint_name in ((left_id, "wheel_left_hinge"), (right_id, "wheel_right_hinge")):
            data.ctrl[:] = 0
            data.ctrl[actuator_id] = 0.01
            mujoco.mj_forward(model, data)
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            dof = int(model.jnt_dofadr[joint_id])
            np.testing.assert_allclose(data.qfrc_actuator[dof], 0.01, atol=1e-10)

        zero = np.array([0.0, 0.0, G_M_S2])
        forward_lean = np.array([G_M_S2 * np.sin(np.deg2rad(10)), 0.0,
                                 G_M_S2 * np.cos(np.deg2rad(10))])
        backward_lean = np.array([-forward_lean[0], 0.0, forward_lean[2]])
        zero_counts, _ = si_to_raw_counts(zero, np.zeros(3))
        forward_counts, _ = si_to_raw_counts(forward_lean, np.zeros(3))
        backward_counts, _ = si_to_raw_counts(backward_lean, np.zeros(3))
        assert abs(firmware_pitch_accel_deg(zero_counts)) < 0.01
        assert firmware_pitch_accel_deg(forward_counts) < -9.9
        assert firmware_pitch_accel_deg(backward_counts) > 9.9

        estimator = WallEImuModel(fixture)
        first = estimator.update(zero, np.zeros(3), 0.01)
        second = estimator.update(zero, np.array([0, np.deg2rad(10), 0]), 0.01)
        assert abs(first.pitch_deg) < 0.01 and abs(second.pitch_rate_dps - 10) < 0.02
        assert 0 < second.pitch_deg < 0.11
        offset = estimator.calibrate_angle_zero([-2.0, -2.2, -1.9])
        assert -2.1 < offset < -2.0
        calibrated = estimator.update(forward_lean, np.zeros(3), 0.01)
        assert calibrated.pitch_deg < -7.8  # calibration changes neutral, not tilt sign

        # Replay the three-run stationary observation in fixed synthetic poses:
        # initial fused output near zero, then convergence toward the pose's
        # steady gravity angles over tens of seconds. This validates the
        # empirical signal shape, not the unknown hardware cause or dynamics.
        replay = copy.deepcopy(config)
        replay["imu"]["accel_bias_m_s2"] = [0.0, 0.0, 0.0]
        replay["imu"]["gyro_bias_rad_s"] = [0.0, 0.0, 0.0]
        replay["imu"]["accel_noise_std_m_s2"] = [0.0, 0.0, 0.0]
        replay["imu"]["gyro_noise_std_rad_s"] = [0.0, 0.0, 0.0]
        replay["imu"]["angle_zero_deg"] = None
        replay["imu"]["startup_transient_enabled"] = True
        true_pitch = -0.31
        pitch_accel = np.array([-G_M_S2 * np.sin(np.deg2rad(true_pitch)), 0.0,
                                G_M_S2 * np.cos(np.deg2rad(true_pitch))])
        pitch_replay = WallEImuModel(replay)
        pitch_samples = [pitch_replay.update(pitch_accel, np.zeros(3), 0.01).pitch_deg
                         for _ in range(6000)]
        assert abs(pitch_samples[0] - (-1.81)) < 0.15
        assert abs(float(np.mean(pitch_samples[:1000])) - (-1.25)) < 0.35
        assert abs(pitch_samples[-1] - true_pitch) < 0.1

        true_roll = -13.70
        roll_accel = np.array([0.0, G_M_S2 * np.sin(np.deg2rad(true_roll)),
                               G_M_S2 * np.cos(np.deg2rad(true_roll))])
        roll_replay = WallEImuModel(replay)
        roll_samples = [roll_replay.update(roll_accel, np.zeros(3), 0.01).roll_deg
                        for _ in range(6000)]
        assert abs(roll_samples[0] - 0.16) < 0.15
        assert abs(float(np.mean(roll_samples[:1000])) - (-4.9)) < 0.4
        assert abs(roll_samples[-1] - true_roll) < 0.1

        try:
            dc_config = copy.deepcopy(fixture)
            dc_config["motor"]["mode"] = "dc_motor"
            dc_config["motor"]["dc_motor"] = {key: None for key in WallEMotorModel.REQUIRED}
            WallEMotorModel(dc_config, sign_override=(1, -1))
        except ValueError as error:
            assert "measured parameters" in str(error)
        else:
            raise AssertionError("Physical DC motor mode must reject missing characterization")
        test_motor = WallEMotorModel(fixture, ideal_test_torque_nm=0.02, sign_override=(1, -1))
        torque = test_motor.pwm_to_torques(50, 0.0, 0.0)
        np.testing.assert_allclose([torque.left_nm, torque.right_nm], [0.01, -0.01])

    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
    print(f"MuJoCo {mujoco.__version__}: balance MJCF sensors, actuators, pitch mapping, "
          "reset-pose IMU seeding, filter, empirical startup replay, unknown-parameter guards, "
          "and ideal torque plumbing verified.")
    print("Synthetic sensor replay and ideal motor torque are test fixtures only, not hardware validation.")


if __name__ == "__main__":
    main()
