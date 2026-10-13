"""Evaluate a saved Wall-E PPO policy on fresh simulation resets (no training)."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from datetime import datetime, timezone
import hashlib
import json
import math
import mujoco
import os
from pathlib import Path
import statistics
import sys
import time

import numpy as np
from stable_baselines3 import PPO

from environments.wall_e_balance_env import WallEBalanceEnv, load_wall_e_config
from simulation.balance_config import ROOT


def evaluate_saved(model_path: Path, config: dict, episodes: int, seed: int,
                   tilts_deg: list[float] | None = None, viewer: bool = False,
                   playback_speed: float = 1.0) -> dict:
    if episodes < 1:
        raise ValueError("episodes must be positive")
    if tilts_deg is not None and not tilts_deg:
        raise ValueError("tilts_deg must not be empty")
    if not math.isfinite(playback_speed) or playback_speed <= 0:
        raise ValueError("playback_speed must be finite and positive")
    if viewer and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("A graphical desktop is required for --viewer")
    model_path = Path(model_path).expanduser().resolve()
    policy = PPO.load(str(model_path), device="cpu")
    env = WallEBalanceEnv(config=config)
    criteria = config.get("evaluation", {
        "settling_time_s": 1.0, "max_excursion_m": 0.05,
        "settled_rms_speed_m_s": 0.02, "settled_rms_pitch_error_deg": 2.0,
    })
    rows = []
    try:
        if policy.observation_space != env.observation_space or policy.action_space != env.action_space:
            raise ValueError("Saved policy observation/action spaces differ from the current config")
        if viewer:
            import mujoco.viewer
            viewer_context = mujoco.viewer.launch_passive(env.model, env.data)
        else:
            viewer_context = nullcontext(None)
        with viewer_context as live_viewer:
            if live_viewer is not None:
                camera = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "three_quarter")
                if camera >= 0:
                    live_viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    live_viewer.cam.fixedcamid = camera
            for index in range(episodes):
                episode_seed = seed + index
                options = None if tilts_deg is None else {"pitch_deg": tilts_deg[index % len(tilts_deg)]}
                observation, info = env.reset(seed=episode_seed, options=options)
                start_forward_m = float(env.data.xpos[env.chassis_id, 1])
                if live_viewer is not None:
                    live_viewer.sync()
                    episode_wall_start = time.perf_counter()
                episode_reward = 0.0
                maximum_pitch = abs(info["true_pitch_debug_deg"])
                maximum_target_error = abs(info["true_pitch_debug_deg"] - config["environment"]["target_pitch_deg"])
                accumulated_target_error = 0.0
                maximum_imu_pitch = abs(info["imu_pitch_deg"])
                maximum_wheel_speed = 0.0
                maximum_excursion = 0.0
                settled_speeds = []
                settled_errors = []
                terminated = truncated = False
                while not (terminated or truncated):
                    action, _ = policy.predict(observation, deterministic=True)
                    observation, reward, terminated, truncated, info = env.step(action)
                    episode_reward += reward
                    maximum_pitch = max(maximum_pitch, abs(info["true_pitch_debug_deg"]))
                    target_error = abs(info["true_pitch_debug_deg"] - config["environment"]["target_pitch_deg"])
                    maximum_target_error = max(maximum_target_error, target_error)
                    accumulated_target_error += target_error
                    maximum_excursion = max(maximum_excursion, abs(info["forward_displacement_m"]))
                    if info["time_s"] >= criteria["settling_time_s"]:
                        settled_speeds.append(info["forward_velocity_m_s"])
                        settled_errors.append(target_error)
                    maximum_imu_pitch = max(maximum_imu_pitch, abs(info["imu_pitch_deg"]))
                    maximum_wheel_speed = max(maximum_wheel_speed,
                                              abs(info["left_wheel_rad_s"]),
                                              abs(info["right_wheel_rad_s"]))
                    if live_viewer is not None:
                        live_viewer.sync()
                        if not live_viewer.is_running():
                            raise RuntimeError("Viewer closed before evaluation completed")
                        # Pace against wall time. Sleeping for a full control
                        # interval *after* compute/render makes 1x playback slow.
                        target_wall_time = (episode_wall_start +
                                            env.steps * env.control_dt / playback_speed)
                        remaining = target_wall_time - time.perf_counter()
                        if remaining > 0:
                            time.sleep(remaining)
                rms_speed = float(np.sqrt(np.mean(np.square(settled_speeds)))) if settled_speeds else None
                rms_error = float(np.sqrt(np.mean(np.square(settled_errors)))) if settled_errors else None
                stationary_success = bool(
                    truncated and not terminated and rms_speed is not None
                    and maximum_excursion <= criteria["max_excursion_m"]
                    and rms_speed <= criteria["settled_rms_speed_m_s"]
                    and rms_error <= criteria["settled_rms_pitch_error_deg"])
                rows.append({
                    "seed": episode_seed, "reset_pitch_deg": info["reset_pitch_deg"],
                    "survival_s": env.steps * env.control_dt,
                    "max_abs_true_pitch_deg": maximum_pitch,
                    "max_abs_target_pitch_error_deg": maximum_target_error,
                    "mean_abs_target_pitch_error_deg": accumulated_target_error / env.steps,
                    "forward_displacement_m": float(env.data.xpos[env.chassis_id, 1]) - start_forward_m,
                    "axle_forward_displacement_m": info["forward_displacement_m"],
                    "max_abs_axle_displacement_m": maximum_excursion,
                    "settled_rms_speed_m_s": rms_speed,
                    "settled_rms_pitch_error_deg": rms_error,
                    "stationary_success": stationary_success,
                    "max_abs_imu_pitch_deg": maximum_imu_pitch,
                    "max_abs_wheel_speed_rad_s": maximum_wheel_speed,
                    "reward": episode_reward, "fell": bool(terminated),
                    "truncated": bool(truncated), "termination_reason": info["termination_reason"],
                })
    finally:
        env.close()
    return {
        "status": "simulation-only; published N20 endpoints, inferred motor curve and provisional contacts/signs",
        "environment": config["environment"],
        "stationary_criteria": criteria,
        "source_sha256": {relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
                          for relative in ("environments/wall_e_balance_env.py",
                                           "simulation/motor_model.py", "simulation/imu_model.py")},
        "policy": str(model_path),
        "policy_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "balance_model": config["model_path"],
        "balance_model_sha256": hashlib.sha256(Path(config["model_path"]).read_bytes()).hexdigest(),
        "motor_config": config["motor_config_path"],
        "motor_config_sha256": hashlib.sha256(Path(config["motor_config_path"]).read_bytes()).hexdigest(),
        "deterministic": True,
        "viewer_playback_speed": playback_speed if viewer else None,
        "seed_first": seed, "seed_last": seed + episodes - 1,
        "reset_tilt_override_cycle_deg": tilts_deg,
        "episodes": rows,
        "summary": {
            "episode_count": episodes,
            "mean_survival_s": statistics.mean(row["survival_s"] for row in rows),
            "max_survival_s": max(row["survival_s"] for row in rows),
            "mean_reward": statistics.mean(row["reward"] for row in rows),
            "fall_rate": statistics.mean(float(row["fell"]) for row in rows),
            "stationary_success_rate": statistics.mean(float(row["stationary_success"]) for row in rows),
            "max_abs_axle_displacement_m": max(row["max_abs_axle_displacement_m"] for row in rows),
            "max_settled_rms_speed_m_s": max(
                (row["settled_rms_speed_m_s"] for row in rows if row["settled_rms_speed_m_s"] is not None), default=None),
            "max_abs_true_pitch_deg": max(row["max_abs_true_pitch_deg"] for row in rows),
            "mean_abs_target_pitch_error_deg": statistics.mean(
                row["mean_abs_target_pitch_error_deg"] for row in rows),
            "mean_abs_forward_displacement_m": statistics.mean(
                abs(row["forward_displacement_m"]) for row in rows),
            "max_abs_wheel_speed_rad_s": max(row["max_abs_wheel_speed_rad_s"] for row in rows),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="Saved final_model.zip or checkpoint .zip")
    parser.add_argument("--config", type=Path, help="Resolved Wall-E PPO config; defaults to run/config.yaml")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seconds", type=float, help="Evaluation episode duration override")
    parser.add_argument("--seed", type=int, help="First held-out reset seed")
    parser.add_argument("--tilts-deg", help="Comma-separated explicit reset tilts, cycled across episodes")
    parser.add_argument("--viewer", action="store_true", help="Optional live MuJoCo evaluation display")
    parser.add_argument("--playback-speed", type=float, default=1.0,
                        help="Viewer pace relative to real time (default 1.0); does not change physics")
    parser.add_argument("--output", type=Path, help="Results JSON path")
    args = parser.parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        parser.error(f"Policy file not found: {model_path}. Check that any shell variable in the path is set.")
    run_dir = model_path.parent
    config_path = args.config or next((candidate for candidate in
                                       (run_dir / "config.yaml", run_dir.parent / "config.yaml")
                                       if candidate.is_file()), None)
    if config_path is None:
        parser.error("Could not find run/config.yaml; provide --config")
    if not config_path.is_file():
        parser.error(f"Config file not found: {config_path}. Check that any shell variable in the path is set.")
    config = load_wall_e_config(config_path)
    if args.seconds is not None:
        config["environment"]["max_episode_seconds"] = args.seconds
    first_seed = args.seed if args.seed is not None else (int(config["seed"])
                  + int(config["training"]["independent_seed_offset"]))
    tilts = None if args.tilts_deg is None else [float(item) for item in args.tilts_deg.split(",")]
    result = evaluate_saved(model_path, config, args.episodes, first_seed, tilts,
                            args.viewer, args.playback_speed)
    output = args.output or run_dir / f"heldout_evaluation_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Saved simulation-only evaluation: {output.resolve()}")
    print(json.dumps(result["summary"], indent=2))
    if args.viewer:
        # On this desktop GLFW has segfaulted during Python interpreter
        # shutdown after the passive viewer closed. All results have already
        # been saved and the viewer context has exited; skip Python's native
        # module finalizers for this GUI-only process.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
