"""Watch or trace one CartPole episode without changing policy weights."""

import argparse
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("right", "ppo"), default="ppo")
    parser.add_argument("--model", type=Path, default=Path("experiments/cartpole-ppo-seed42-100k/final_model.zip"))
    parser.add_argument("--seed", type=int, default=20042)
    parser.add_argument("--render", action="store_true", help="Open a simulation window; requires pygame and a desktop")
    parser.add_argument("--stochastic", action="store_true", help="Sample PPO actions rather than selecting the most probable")
    parser.add_argument("--print-every", type=int, default=50)
    args = parser.parse_args()
    if args.print_every < 1:
        parser.error("--print-every must be positive")
    if args.stochastic and args.controller != "ppo":
        parser.error("--stochastic requires --controller ppo")
    if args.controller == "ppo" and not args.model.is_file():
        parser.error(f"Checkpoint not found: {args.model}")
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    model = PPO.load(args.model, device="cpu") if args.controller == "ppo" else None
    env = gym.make("CartPole-v1", render_mode="human" if args.render else None)
    try:
        observation, _ = env.reset(seed=args.seed)
        print("Observation order: x (m), velocity (m/s), angle (rad), angular velocity (rad/s)")
        print(f"Initial: {observation}")
        total_reward = 0.0
        for step in range(1, 501):
            probabilities = ""
            if model is None:
                action = 1
            else:
                with torch.no_grad():
                    tensor, _ = model.policy.obs_to_tensor(observation)
                    probs = model.policy.get_distribution(tensor).distribution.probs.cpu().numpy()[0]
                probabilities = f" P(left)={probs[0]:.3f} P(right)={probs[1]:.3f}"
                predicted, _ = model.predict(observation, deterministic=not args.stochastic)
                action = int(predicted.item())
            observation, reward, terminated, truncated, _ = env.step(action)
            total_reward += reward
            if step <= 5 or step % args.print_every == 0 or terminated or truncated:
                state = " ".join(f"{value:+.4f}" for value in observation)
                print(f"step={step:3d} action={'LEFT' if action == 0 else 'RIGHT'}{probabilities} next=[{state}] reward={reward:.1f} terminated={terminated} truncated={truncated}")
            if terminated or truncated:
                print(f"Finished: {step} steps; return={total_reward:.1f}; simulated time={step * env.unwrapped.tau:.2f} s")
                break
    finally:
        env.close()


if __name__ == "__main__":
    main()
