"""Train one headless PPO environment on provisional Wall-E MuJoCo dynamics."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import platform
from pathlib import Path
import shutil
import subprocess
import sys

import mujoco
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
import yaml

from environments.wall_e_balance_env import DEFAULT_CONFIG, WallEBalanceEnv, load_wall_e_config
from training.evaluate_wall_e_ppo import evaluate_saved


ROOT = Path(__file__).resolve().parents[1]


def save_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--timesteps", type=int, help="Short smoke-run override")
    parser.add_argument("--run-dir", type=Path, help="New, nonexisting experiment directory")
    args = parser.parse_args()
    config = load_wall_e_config(args.config)
    training = config["training"]
    timesteps = args.timesteps if args.timesteps is not None else int(training["total_timesteps"])
    if timesteps < 1:
        parser.error("--timesteps must be positive")
    n_steps = int(training["ppo"]["n_steps"])
    if n_steps < 2 or int(training["ppo"]["batch_size"]) < 2:
        parser.error("PPO rollout and batch sizes must each be at least 2")
    seed = int(config["seed"])
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir = args.run_dir or ROOT / "experiments/wall_e_ppo" / f"seed{seed}_{stamp}"
    run_dir = run_dir.expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "checkpoints").mkdir()
    (run_dir / "best").mkdir()
    resolved = {**config, "training": {**training, "total_timesteps": timesteps}}
    (run_dir / "config.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False), encoding="utf-8")
    shutil.copyfile(config["motor_config_path"], run_dir / "n20_assumption_snapshot.yaml")
    source_hashes = {}
    for relative in ("environments/wall_e_balance_env.py", "simulation/imu_model.py",
                     "simulation/motor_model.py", "training/train_wall_e_ppo.py",
                     "training/evaluate_wall_e_ppo.py"):
        source = ROOT / relative
        target = run_dir / "source_snapshot" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        source_hashes[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    save_json(run_dir / "versions.json", {
        "python": sys.version, "platform": platform.platform(),
        "mujoco": mujoco.__version__,
        **{name: importlib.metadata.version(name) for name in
           ("numpy", "gymnasium", "stable-baselines3", "torch", "tensorboard", "PyYAML")},
    })
    frozen = subprocess.run([sys.executable, "-m", "pip", "freeze", "--exclude-editable"],
                            check=True, capture_output=True, text=True)
    (run_dir / "requirements-freeze.txt").write_text(frozen.stdout, encoding="utf-8")
    metadata = {
        "status": "simulation-only software training; no hardware validation",
        "seed": seed, "requested_timesteps": timesteps,
        "ppo_rollout_steps": n_steps,
        "validation_seed_first": seed + int(training["validation_seed_offset"]),
        "independent_seed_first": seed + int(training["independent_seed_offset"]),
        "balance_model_sha256": hashlib.sha256(Path(config["model_path"]).read_bytes()).hexdigest(),
        "motor_config_sha256": hashlib.sha256(Path(config["motor_config_path"]).read_bytes()).hexdigest(),
        "motor_assumption": "N20-12V-200 image-derived curve and configured signs; not measured on this robot",
        "observation_limit": "Wheel speeds and stationary-task axle position/velocity are privileged simulator measurements; policy is not deployable on encoderless hardware",
        "cartpole_workflow_modified": False,
        "source_sha256": source_hashes,
    }
    save_json(run_dir / "run.json", metadata)

    train_env = Monitor(WallEBalanceEnv(config=resolved), filename=str(run_dir / "train_monitor.csv"))
    validation_env = Monitor(WallEBalanceEnv(config=resolved),
                             filename=str(run_dir / "validation_monitor.csv"))
    train_env.reset(seed=seed)
    train_env.action_space.seed(seed)
    validation_env.reset(seed=metadata["validation_seed_first"])
    validation_env.action_space.seed(metadata["validation_seed_first"])
    try:
        policy = PPO(training["policy"], train_env, seed=seed, device=training["device"],
                     tensorboard_log=str(run_dir / "tensorboard"), verbose=1,
                     **training["ppo"])
        checkpoint = CheckpointCallback(
            save_freq=min(int(training["checkpoint_every_steps"]), timesteps),
            save_path=str(run_dir / "checkpoints"), name_prefix="wall_e_ppo")
        validation = EvalCallback(
            validation_env, best_model_save_path=str(run_dir / "best"),
            log_path=str(run_dir / "validation"),
            eval_freq=min(int(training["validation_every_steps"]), timesteps),
            n_eval_episodes=int(training["validation_episodes"]), deterministic=True)
        policy.learn(total_timesteps=timesteps, callback=[checkpoint, validation], tb_log_name="PPO")
        final_model = run_dir / "final_model.zip"
        policy.save(str(final_model))
        results = evaluate_saved(final_model, resolved,
                                 int(training["independent_eval_episodes"]),
                                 metadata["independent_seed_first"])
        save_json(run_dir / "final_evaluation.json", results)
        metadata["actual_timesteps"] = policy.num_timesteps
        metadata["final_model_sha256"] = hashlib.sha256(final_model.read_bytes()).hexdigest()
        metadata["independent_evaluation_summary"] = results["summary"]
        save_json(run_dir / "run.json", metadata)
        print(f"Run: {run_dir}")
        print("Independent, simulation-only evaluation:", results["summary"])
    finally:
        train_env.close()
        validation_env.close()


if __name__ == "__main__":
    main()
