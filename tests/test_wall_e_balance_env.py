"""Behavior checks for the separate simulation-only PPO environment."""

from copy import deepcopy

import numpy as np
import pytest
import yaml

from environments.wall_e_balance_env import WallEBalanceEnv, load_wall_e_config


@pytest.fixture
def env():
    instance = WallEBalanceEnv()
    yield instance
    instance.close()


def test_gymnasium_contract_and_finite_values(env):
    observation, info = env.reset(seed=123)
    assert observation.shape == (4,) and observation.dtype == np.float32
    assert env.observation_space.contains(observation)
    assert isinstance(info, dict) and np.isfinite(observation).all()
    result = env.step(np.array([0.1], dtype=np.float32))
    observation, reward, terminated, truncated, info = result
    assert observation.shape == (4,) and observation.dtype == np.float32
    assert np.isfinite(observation).all() and np.isfinite(reward)
    assert isinstance(reward, float) and isinstance(terminated, bool) and isinstance(truncated, bool)
    assert info["episode_steps"] == 1


def test_fixed_seed_reproduces_perturbation_and_sensor_state(env):
    first_obs, first_info = env.reset(seed=2043)
    first_step = env.step(np.array([0.25], dtype=np.float32))
    second_obs, second_info = env.reset(seed=2043)
    second_step = env.step(np.array([0.25], dtype=np.float32))
    np.testing.assert_array_equal(first_obs, second_obs)
    assert first_info["reset_pitch_deg"] == second_info["reset_pitch_deg"]
    np.testing.assert_array_equal(first_step[0], second_step[0])
    assert first_step[1] == second_step[1]


def test_reset_pitch_sign_and_filter_reseed(env):
    config = load_wall_e_config()
    config["environment"]["reset_pitch_center_deg"] = 0.0
    with WallEBalanceEnv(config=config) as zero_centered:
        for angle in (-3.0, 3.0):
            observation, info = zero_centered.reset(seed=8, options={"pitch_deg": angle})
            assert abs(info["true_pitch_debug_deg"] - angle) < 0.01
            assert np.sign(info["imu_pitch_deg"]) == np.sign(angle)
            assert np.all(zero_centered.data.qvel == 0)
            assert observation[1] == 0
            zero_centered.step(np.array([0.0], dtype=np.float32))
            assert zero_centered.imu.startup_elapsed_s == pytest.approx(zero_centered.control_dt)
        observation, info = zero_centered.reset(seed=8, options={"pitch_deg": 0.0})
        assert abs(info["imu_pitch_deg"]) < 0.1
        assert observation[1] == 0
        assert zero_centered.last_reading is None and zero_centered.imu.startup_elapsed_s == 0
        assert np.all(zero_centered.data.qvel == 0)


def test_default_reset_centers_on_cad_com_balance_angle(env):
    center = env.settings["reset_pitch_center_deg"]
    _, info = env.reset(seed=8, options={"pitch_deg": center})
    axle = np.mean(env.data.xanchor[env.wheel_joint_ids], axis=0)
    com_offset = env.data.xipos[env.chassis_id] - axle
    assert abs(com_offset[1]) < 0.0001
    assert info["true_pitch_debug_deg"] == pytest.approx(center, abs=0.01)
    assert info["imu_pitch_deg"] == pytest.approx(center, abs=0.1)
    assert env.settings["target_pitch_deg"] == center
    for seed in range(10):
        _, info = env.reset(seed=seed)
        assert abs(info["reset_pitch_deg"] - center) <= env.settings["reset_pitch_range_deg"]


def test_saved_config_motor_update_defaults_preserve_old_runs(tmp_path, monkeypatch):
    config = load_wall_e_config()
    config["environment"].pop("motor_torque_update")
    current_path = tmp_path / "current_snapshot.yaml"
    current_path.write_text(yaml.safe_dump(config))
    assert load_wall_e_config(current_path)["environment"]["motor_torque_update"] == "per_physics_step"

    config["environment"].pop("reset_pitch_center_deg")
    config["environment"]["target_pitch_deg"] = 0.0
    old_path = tmp_path / "old_snapshot.yaml"
    old_path.write_text(yaml.safe_dump(config))
    old = load_wall_e_config(old_path)
    assert old["environment"]["reset_pitch_center_deg"] == 0.0
    assert old["environment"]["motor_torque_update"] == "per_control_interval"
    with WallEBalanceEnv(config=old) as legacy_env:
        legacy_env.reset(seed=10, options={"pitch_deg": 0.0})
        seen = []
        original = legacy_env.motor.pwm_to_torques

        def record(*args):
            seen.append(args)
            return original(*args)

        monkeypatch.setattr(legacy_env.motor, "pwm_to_torques", record)
        legacy_env.step(np.array([0.5], dtype=np.float32))
        assert len(seen) == 1


def test_action_bound_and_existing_motor_torque_path(env, monkeypatch):
    env.reset(seed=10, options={"pitch_deg": env.settings["reset_pitch_center_deg"]})
    env.data.qvel[env.wheel_dofs] = [5.0, -5.0]
    seen = []
    original = env.motor.pwm_to_torques

    def record(pwm, left_speed, right_speed):
        seen.append((pwm, left_speed, right_speed))
        return original(pwm, left_speed, right_speed)

    monkeypatch.setattr(env.motor, "pwm_to_torques", record)
    _, _, _, _, info = env.step(np.array([0.5], dtype=np.float32))
    assert len(seen) == env.substeps
    assert seen[0] == (20.0, 5.0, -5.0)
    assert any((left, right) != (5.0, -5.0) for _, left, right in seen[1:])
    expected = original(*seen[-1])
    np.testing.assert_allclose(env.data.ctrl[env.actuator_ids],
                               [expected.left_nm, expected.right_nm])
    assert info["signed_pwm_percent"] == 20.0
    env.reset(seed=10, options={"pitch_deg": env.settings["reset_pitch_center_deg"]})
    _, _, _, _, info = env.step(np.array([1.0], dtype=np.float32))
    assert len(seen) == 2 * env.substeps
    assert all(pwm == 40.0 for pwm, _, _ in seen[-env.substeps:])
    assert info["signed_pwm_percent"] == 40.0
    with pytest.raises(ValueError):
        env.step(np.array([1.01], dtype=np.float32))
    with pytest.raises(ValueError):
        env.step(np.array([np.nan], dtype=np.float32))


def test_fall_termination_and_time_limit_are_distinct():
    config = load_wall_e_config()
    falling = deepcopy(config)
    falling["environment"]["reset_pitch_center_deg"] = 0.0
    falling["environment"]["fall_angle_deg"] = 1.0
    with WallEBalanceEnv(config=falling) as env:
        env.reset(seed=5, options={"pitch_deg": 2.0})
        _, _, terminated, truncated, info = env.step(np.array([0.0], dtype=np.float32))
        assert terminated and not truncated and info["termination_reason"] == "fall"
        with pytest.raises(RuntimeError):
            env.step(np.array([0.0], dtype=np.float32))
    timed = deepcopy(config)
    timed["environment"]["reset_pitch_center_deg"] = 0.0
    timed["environment"]["max_episode_seconds"] = 0.01
    with WallEBalanceEnv(config=timed) as env:
        env.reset(seed=5, options={"pitch_deg": 0.0})
        _, _, terminated, truncated, info = env.step(np.array([0.0], dtype=np.float32))
        assert truncated and not terminated and info["termination_reason"] == "time_limit"


def test_short_headless_random_rollout_has_no_unexpected_resets(env):
    env.reset(seed=17)
    rng = np.random.default_rng(17)
    total_steps = 0
    episodes = 0
    while total_steps < 250:
        action = np.array([rng.uniform(-1, 1)], dtype=np.float32)
        observation, reward, terminated, truncated, info = env.step(action)
        assert np.isfinite(observation).all() and np.isfinite(reward)
        assert info["episode_steps"] >= 1
        total_steps += 1
        if terminated or truncated:
            episodes += 1
            env.reset()
    assert episodes >= 1


def test_default_reward_favors_surviving_over_falling_quickly():
    config = load_wall_e_config()
    config["environment"]["reset_pitch_center_deg"] = 0.0

    def rollout(action):
        with WallEBalanceEnv(config=config) as env:
            env.reset(seed=123, options={"pitch_deg": 0.0})
            steps = 0
            total_reward = 0.0
            while True:
                _, reward, terminated, truncated, _ = env.step(
                    np.array([action], dtype=np.float32))
                steps += 1
                total_reward += reward
                if terminated or truncated:
                    return steps, total_reward

    slower_steps, slower_return = rollout(0.0)
    faster_steps, faster_return = rollout(1.0)
    assert slower_steps > faster_steps
    assert slower_return == pytest.approx(slower_steps)
    assert faster_return == pytest.approx(faster_steps)
    assert slower_return > faster_return
