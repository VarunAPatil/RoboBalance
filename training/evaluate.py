"""Evaluate a saved CartPole policy on fresh deterministic episodes."""

from __future__ import annotations

import argparse
from pathlib import Path

from stable_baselines3 import PPO

from training.common import evaluate, write_json


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="Path to final_model.zip or best/best_model.zip")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=20042)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    model = PPO.load(args.model, device="cpu")
    result = evaluate(model, args.episodes, args.seed)
    if args.output:
        write_json(args.output, result)
    print(f"Mean reward: {result['mean_reward']:.1f} ± {result['std_reward']:.1f}")
    print(f"Mean episode length: {result['mean_length']:.1f}; range {result['min_length']}–{result['max_length']}")


if __name__ == "__main__":
    main()
