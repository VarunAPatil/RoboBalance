"""Replay a fixed policy at several physics timesteps; simulation evidence only."""

import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np
from stable_baselines3 import PPO

from environments.wall_e_balance_env import WallEBalanceEnv, load_wall_e_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--seed", type=int, default=92042)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = load_wall_e_config(args.config)
    config["environment"]["max_episode_seconds"] = args.seconds
    policy = PPO.load(args.model, device="cpu")
    args.output.mkdir(parents=True, exist_ok=False)
    summary = []
    for dt in (0.002, 0.001, 0.0005):
        with WallEBalanceEnv(config=config) as env:
            env.model.opt.timestep = dt
            env.substeps = round(env.control_dt / dt)
            assert np.isclose(env.substeps * dt, env.control_dt)
            obs, _ = env.reset(seed=args.seed)
            rows = []
            while True:
                action, _ = policy.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                rows.append({**info, "reward": reward, "contacts": env.data.ncon})
                if terminated or truncated:
                    break
            with (args.output / f"trace_{dt:g}.csv").open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            settled = [r for r in rows if r["time_s"] >= 1]
            summary.append({
                "physics_timestep_s": dt, "control_period_s": env.control_dt,
                "seed": args.seed, "reset_pitch_deg": rows[0]["reset_pitch_deg"],
                "survival_s": rows[-1]["time_s"], "fell": terminated,
                "max_excursion_m": max(abs(r["forward_displacement_m"]) for r in rows),
                "settled_rms_speed_m_s": float(np.sqrt(np.mean([
                    r["forward_velocity_m_s"] ** 2 for r in settled]))) if settled else None,
                "warnings": [int(w.number) for w in env.data.warning],
            })
    report = {"status": "simulation-only timestep sensitivity, not physical validation",
              "model": str(args.model.resolve()), "config": config,
              "mujoco": mujoco.__version__, "results": summary}
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
