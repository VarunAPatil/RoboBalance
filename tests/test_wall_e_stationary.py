"""Station-keeping observations and rewards, separate from legacy survival runs."""

import mujoco
import numpy as np
import pytest

from environments.wall_e_balance_env import WallEBalanceEnv, load_wall_e_config
from simulation.balance_config import ROOT


@pytest.fixture
def stationary():
    with WallEBalanceEnv(config_path=ROOT / "configs/wall_e_stationary_ppo.yaml") as env:
        yield env


def test_stationary_contract_reset_and_position_reference(stationary):
    env = stationary
    first, _ = env.reset(seed=123)
    assert first.shape == (6,) and first.dtype == np.float32
    assert np.all(first[4:] == 0)
    for _ in range(20):
        obs, reward, terminated, truncated, info = env.step(np.array([0.1], dtype=np.float32))
        assert np.isfinite(obs).all() and 0.1 <= reward <= 1.0
        assert obs[4] == pytest.approx(info["forward_displacement_m"] / 0.05, rel=1e-6, abs=1e-6)
        if terminated or truncated:
            break
    again, info = env.reset(seed=123)
    np.testing.assert_array_equal(first, again)
    assert info["forward_displacement_m"] == info["forward_velocity_m_s"] == 0


def test_displacement_reduces_reward_without_negative_fall_incentive(stationary):
    env = stationary
    env.reset(seed=3, options={"pitch_deg": 32.5})
    _, centered_reward, _, _, _ = env.step(np.array([0.0], dtype=np.float32))
    env.reset(seed=3, options={"pitch_deg": 32.5})
    # A rigid horizontal translation preserves the attitude/contact dynamics.
    env.data.qpos[env.free_qadr + 1] += 0.1
    mujoco.mj_forward(env.model, env.data)
    _, displaced_reward, _, _, info = env.step(np.array([0.0], dtype=np.float32))
    assert info["forward_displacement_m"] > 0.09
    assert 0.1 <= displaced_reward < centered_reward <= 1.0


def test_old_policy_observation_shape_is_preserved():
    config = load_wall_e_config()
    with WallEBalanceEnv(config=config) as env:
        assert env.observation_space.shape == (4,)
