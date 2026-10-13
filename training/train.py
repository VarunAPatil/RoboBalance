"""Train a seeded PPO CartPole policy and retain the experiment artifacts."""

from __future__ import annotations

import argparse
import importlib.metadata
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

from training.common import evaluate, make_env, read_config, write_json


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/ppo.yaml"))
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--timesteps", type=int, help="Override total_timesteps for a smoke run")
    args = parser.parse_args()

    config = read_config(args.config)
    timesteps = args.timesteps if args.timesteps is not None else int(config["total_timesteps"])
    if timesteps < 1:
        parser.error("--timesteps must be positive")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.run_dir or Path("experiments") / f"cartpole-ppo-seed{config['seed']}-{stamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "checkpoints").mkdir()
    (run_dir / "best").mkdir()
    resolved = {**config, "total_timesteps": timesteps}
    (run_dir / "config.yaml").write_text(yaml_dump(resolved), encoding="utf-8")
    write_json(run_dir / "versions.json", {
        "python": sys.version,
        "gymnasium": importlib.metadata.version("gymnasium"),
        "stable-baselines3": importlib.metadata.version("stable-baselines3"),
        "torch": importlib.metadata.version("torch"),
        "tensorboard": importlib.metadata.version("tensorboard"),
        "platform": __import__("platform").platform(),
    })
    frozen = subprocess.run(
        [sys.executable, "-m", "pip", "freeze", "--exclude-editable"],
        check=True, capture_output=True, text=True,
    )
    (run_dir / "requirements-freeze.txt").write_text(frozen.stdout, encoding="utf-8")

    seed = int(config["seed"])
    train_env = make_env(seed, run_dir / "train_monitor.csv")
    validation_env = make_env(seed + 1000, run_dir / "validation_monitor.csv")
    try:
        model = PPO(
            config["policy"], train_env, seed=seed, verbose=1,
            device=config["device"],
            tensorboard_log=str(run_dir / "tensorboard"),
            **config["ppo"],
        )
        callbacks = [
            CheckpointCallback(save_freq=int(config["checkpoint_freq"]), save_path=str(run_dir / "checkpoints")),
            EvalCallback(
                validation_env,
                best_model_save_path=str(run_dir / "best"),
                log_path=str(run_dir / "validation"),
                eval_freq=int(config["eval_freq"]),
                n_eval_episodes=int(config["eval_episodes"]),
                deterministic=True,
            ),
        ]
        model.learn(total_timesteps=timesteps, callback=callbacks, tb_log_name="PPO")
        model.save(run_dir / "final_model")
        results = evaluate(model, int(config["final_eval_episodes"]), seed + 10000)
        results["training_timesteps_actual"] = model.num_timesteps
        write_json(run_dir / "final_evaluation.json", results)
        print(f"Run: {run_dir.resolve()}")
        print(f"Independent evaluation: mean reward {results['mean_reward']:.1f} over {results['episodes']} episodes")
    finally:
        train_env.close()
        validation_env.close()


def yaml_dump(data: dict) -> str:
    import yaml
    return yaml.safe_dump(data, sort_keys=False)


if __name__ == "__main__":
    main()
