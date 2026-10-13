"""Shared CartPole configuration and evaluation helpers."""

from __future__ import annotations

import json
from pathlib import Path

import gymnasium as gym
import yaml
from stable_baselines3.common.monitor import Monitor


def read_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or config.get("environment") != "CartPole-v1":
        raise ValueError("Phase 1 supports environment: CartPole-v1")
    return config


def make_env(seed: int, monitor_file: Path | None = None) -> Monitor:
    env = Monitor(gym.make("CartPole-v1"), filename=str(monitor_file) if monitor_file else None)
    env.reset(seed=seed)
    env.action_space.seed(seed)
    return env


def evaluate(model, episodes: int, seed: int) -> dict:
    if episodes < 1:
        raise ValueError("episodes must be positive")
    env = make_env(seed)
    rewards: list[float] = []
    lengths: list[int] = []
    try:
        for episode in range(episodes):
            observation, _ = env.reset(seed=seed + episode)
            done = False
            episode_reward = 0.0
            steps = 0
            while not done:
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, _ = env.step(action)
                episode_reward += float(reward)
                steps += 1
                done = terminated or truncated
            rewards.append(episode_reward)
            lengths.append(steps)
    finally:
        env.close()
    import statistics

    return {
        "environment": "CartPole-v1",
        "episodes": episodes,
        "seed_first": seed,
        "seed_last": seed + episodes - 1,
        "mean_reward": statistics.mean(rewards),
        "std_reward": statistics.pstdev(rewards),
        "mean_length": statistics.mean(lengths),
        "max_length": max(lengths),
        "min_length": min(lengths),
        "rewards": rewards,
        "lengths": lengths,
    }


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
