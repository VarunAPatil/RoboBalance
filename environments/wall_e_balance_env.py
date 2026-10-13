"""Single-instance, headless MuJoCo Wall-E balancing task for PPO.

Wheel speeds are privileged simulator observations: the real TT motors have no
encoders, so policies trained here are not directly deployable.
"""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
import yaml

from simulation.balance_config import ROOT, load_config
from simulation.imu_model import G_M_S2, WallEImuModel
from simulation.motor_model import WallEMotorModel


DEFAULT_CONFIG = ROOT / "configs/wall_e_ppo.yaml"


def load_wall_e_config(path: Path | str = DEFAULT_CONFIG) -> dict[str, Any]:
    path = Path(path).expanduser().resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or not isinstance(config.get("environment"), dict):
        raise ValueError("Wall-E PPO config needs an environment mapping")
    config = deepcopy(config)
    for key in ("model_path", "motor_config_path"):
        asset = Path(config[key]).expanduser()
        config[key] = str((asset if asset.is_absolute() else ROOT / asset).resolve())
        if not Path(config[key]).is_file():
            raise FileNotFoundError(config[key])
    env = config["environment"]
    # Pre-COM-reset experiment snapshots did not include either setting. Their
    # original control loop held motor torque for the whole control interval.
    legacy_reset = "reset_pitch_center_deg" not in env
    env.setdefault("reset_pitch_center_deg", 0.0)
    env.setdefault("motor_torque_update", "per_control_interval" if legacy_reset
                   else "per_physics_step")
    if env["motor_torque_update"] not in ("per_control_interval", "per_physics_step"):
        raise ValueError("environment.motor_torque_update must be per_control_interval or per_physics_step")
    for key in ("control_period_s", "max_pwm_percent", "reset_pitch_range_deg",
                "fall_angle_deg", "max_episode_seconds"):
        value = float(env[key])
        if not math.isfinite(value) or (value < 0 if key == "reset_pitch_range_deg" else value <= 0):
            raise ValueError(f"environment.{key} must be finite and positive")
        env[key] = value
    if env["max_pwm_percent"] > 100 or env["fall_angle_deg"] >= 180:
        raise ValueError("PWM must be <=100% and fall threshold below 180 deg")
    for key in ("target_pitch_deg", "reset_pitch_center_deg"):
        env[key] = float(env[key])
        if not math.isfinite(env[key]):
            raise ValueError(f"environment.{key} must be finite")
    env.setdefault("objective", "survival")
    if env["objective"] not in ("survival", "stationary"):
        raise ValueError("Unknown environment.objective")
    size = 6 if env["objective"] == "stationary" else 4
    scales = np.asarray(env["observation_scales"], dtype=float)
    if scales.shape != (size,) or not np.isfinite(scales).all() or np.any(scales <= 0):
        raise ValueError(f"Expected {size} positive finite observation scales")
    reward = env["reward"]
    for key in ("alive", "pitch_weight", "pitch_scale_deg", "pitch_rate_weight",
                "pitch_rate_scale_deg_s", "wheel_speed_weight", "wheel_speed_scale_rad_s",
                "action_weight"):
        value = float(reward[key])
        is_scale = key.startswith("pitch_scale_") or key.startswith("pitch_rate_scale_") or key.startswith("wheel_speed_scale_")
        if not math.isfinite(value) or value < 0 or (is_scale and value == 0) or (key == "alive" and value == 0):
            raise ValueError(f"Invalid reward.{key}")
        reward[key] = value
    if env["objective"] == "stationary":
        for key in ("position_scale_m", "velocity_scale_m_s", "stationary_bonus"):
            reward[key] = float(reward[key])
            if not math.isfinite(reward[key]) or reward[key] <= 0:
                raise ValueError(f"Invalid stationary reward.{key}")
    return config


def _name_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    index = mujoco.mj_name2id(model, kind, name)
    if index < 0:
        raise ValueError(f"Balance MJCF lacks {name}")
    return index


def _chassis_attitude(rotation: np.ndarray) -> tuple[float, float]:
    matrix = rotation.reshape(3, 3)
    pitch = math.degrees(math.atan2(matrix[2, 1], math.hypot(matrix[0, 1], matrix[1, 1])))
    tilt = math.degrees(math.acos(float(np.clip(matrix[2, 2], -1.0, 1.0))))
    return pitch, tilt


class WallEBalanceEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": []}

    def __init__(self, config_path: Path | str = DEFAULT_CONFIG,
                 config: dict[str, Any] | None = None):
        super().__init__()
        self.config = load_wall_e_config(config_path) if config is None else deepcopy(config)
        self.settings = self.config["environment"]
        self.model = mujoco.MjModel.from_xml_path(self.config["model_path"])
        self.data = mujoco.MjData(self.model)
        self.motor_config = load_config(self.config["motor_config_path"])
        if self.motor_config["motor"].get("mode") != "dc_motor":
            raise ValueError("The first PPO experiment requires the separate assumed DC-motor config")
        self.motor = WallEMotorModel(self.motor_config)
        self.imu = WallEImuModel(self.motor_config)
        self.control_dt = self.settings["control_period_s"]
        physics_dt = float(self.model.opt.timestep)
        self.substeps = round(self.control_dt / physics_dt)
        if self.substeps < 1 or not math.isclose(self.substeps * physics_dt, self.control_dt, abs_tol=1e-10):
            raise ValueError("Control period must be an integer multiple of model timestep")
        self.max_steps = round(self.settings["max_episode_seconds"] / self.control_dt)
        if self.max_steps < 1 or not math.isclose(self.max_steps * self.control_dt,
                                                   self.settings["max_episode_seconds"], abs_tol=1e-9):
            raise ValueError("Episode duration must be an integer number of control intervals")
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.stationary = self.settings.get("objective", "survival") == "stationary"
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf,
                                           shape=(6 if self.stationary else 4,), dtype=np.float32)
        self.obs_scales = np.asarray(self.settings["observation_scales"], dtype=np.float64)
        self.chassis_id = _name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.imu_site_id = _name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        self.free_joint_id = _name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
        if self.model.jnt_type[self.free_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("chassis_free is not a free joint")
        self.free_qadr = int(self.model.jnt_qposadr[self.free_joint_id])
        self.wheel_joint_ids = [
            _name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"wheel_{side}_hinge")
            for side in ("left", "right")]
        if any(self.model.jnt_type[index] != mujoco.mjtJoint.mjJNT_HINGE
               for index in self.wheel_joint_ids):
            raise ValueError("Wheel joints must be hinges")
        self.wheel_dofs = [int(self.model.jnt_dofadr[index]) for index in self.wheel_joint_ids]
        self.wheel_bodies = [int(self.model.jnt_bodyid[index]) for index in self.wheel_joint_ids]
        self.actuator_ids = [
            _name_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_wheel_torque")
            for side in ("left", "right")]
        if any(int(self.model.actuator_trnid[actuator, 0]) != joint
               for actuator, joint in zip(self.actuator_ids, self.wheel_joint_ids)):
            raise ValueError("Wheel actuator-to-joint mapping differs from named hinges")
        self.sensor_slices = {}
        for name in ("imu_accel", "imu_gyro"):
            index = _name_id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if int(self.model.sensor_dim[index]) != 3:
                raise ValueError(f"{name} must have three axes")
            start = int(self.model.sensor_adr[index])
            self.sensor_slices[name] = slice(start, start + 3)
        self.steps = 0
        self.last_reading = None
        self.last_action = 0.0
        self.last_torques = (0.0, 0.0)
        self.reset_pitch_deg = 0.0
        self.start_axle_y = 0.0
        self.forward_displacement_m = 0.0
        self.forward_velocity_m_s = 0.0
        self._done = False
        self._closed = False
        self._has_reset = False

    def _wheel_speeds(self) -> tuple[float, float]:
        return tuple(float(self.data.qvel[index]) for index in self.wheel_dofs)

    def _axle_y(self) -> float:
        return float(np.mean(self.data.xanchor[self.wheel_joint_ids, 1]))

    def _observation(self) -> np.ndarray:
        pitch = float(self.imu.pitch_deg)
        rate = 0.0 if self.last_reading is None else float(self.last_reading.pitch_rate_dps)
        values = np.array([pitch, rate, *self._wheel_speeds()], dtype=np.float64)
        if self.stationary:
            # Position/velocity are privileged simulator measurements, NOT an
            # encoderless-hardware estimator. No true chassis angle is exposed.
            values[0] -= self.settings["target_pitch_deg"]
            values = np.append(values, [self.forward_displacement_m, self.forward_velocity_m_s])
        observation = (values / self.obs_scales).astype(np.float32)
        if not np.isfinite(observation).all():
            raise FloatingPointError("Nonfinite Wall-E observation")
        return observation

    def _info(self, reason: str = "") -> dict[str, Any]:
        true_pitch, total_tilt = _chassis_attitude(self.data.xmat[self.chassis_id])
        left_speed, right_speed = self._wheel_speeds()
        return {
            "time_s": float(self.data.time), "episode_steps": self.steps,
            "imu_pitch_deg": float(self.imu.pitch_deg),
            "imu_pitch_rate_deg_s": (0.0 if self.last_reading is None
                                      else float(self.last_reading.pitch_rate_dps)),
            "true_pitch_debug_deg": true_pitch, "total_tilt_debug_deg": total_tilt,
            "left_wheel_rad_s": left_speed, "right_wheel_rad_s": right_speed,
            "action": self.last_action,
            "signed_pwm_percent": self.last_action * self.settings["max_pwm_percent"],
            "left_torque_nm": self.last_torques[0], "right_torque_nm": self.last_torques[1],
            "reset_pitch_deg": self.reset_pitch_deg,
            "forward_displacement_m": self.forward_displacement_m,
            "forward_velocity_m_s": self.forward_velocity_m_s,
            "termination_reason": reason,
        }

    def reset(self, *, seed: int | None = None,
              options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        if self._closed:
            raise RuntimeError("Environment is closed")
        super().reset(seed=seed)
        self.imu.rng = self.np_random
        mujoco.mj_resetData(self.model, self.data)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        center = self.settings["reset_pitch_center_deg"]
        limit = self.settings["reset_pitch_range_deg"]
        if options is not None and "pitch_deg" in options:
            angle = float(options["pitch_deg"])
            if not math.isfinite(angle) or abs(angle - center) > limit:
                raise ValueError("options.pitch_deg is outside configured reset range around center")
        else:
            angle = float(self.np_random.uniform(center - limit, center + limit))
        self.reset_pitch_deg = angle
        # Use the *named* left hinge's world axle direction. Its positive
        # source +X direction produces positive geometric pitch (backward lean);
        # the project's forward/nose-down convention is negative pitch.
        left_body = self.wheel_bodies[0]
        axis = self.data.xmat[left_body].reshape(3, 3) @ self.model.jnt_axis[self.wheel_joint_ids[0]]
        axis /= np.linalg.norm(axis)
        pivot = 0.5 * (self.data.xpos[self.wheel_bodies[0]] + self.data.xpos[self.wheel_bodies[1]])
        delta = np.empty(4)
        mujoco.mju_axisAngle2Quat(delta, axis, math.radians(angle))
        original_quat = self.data.qpos[self.free_qadr + 3:self.free_qadr + 7].copy()
        new_quat = np.empty(4)
        mujoco.mju_mulQuat(new_quat, delta, original_quat)
        lever = self.data.qpos[self.free_qadr:self.free_qadr + 3].copy() - pivot
        rotated = np.empty(3)
        mujoco.mju_rotVecQuat(rotated, lever, delta)
        self.data.qpos[self.free_qadr:self.free_qadr + 3] = pivot + rotated
        self.data.qpos[self.free_qadr + 3:self.free_qadr + 7] = new_quat
        mujoco.mj_forward(self.model, self.data)
        self.imu.reset()
        gravity_site = self.data.site_xmat[self.imu_site_id].reshape(3, 3).T @ np.array([0, 0, G_M_S2])
        self.imu.seed_from_gravity(gravity_site)
        self.last_reading = None
        self.last_action = 0.0
        self.last_torques = (0.0, 0.0)
        self.steps = 0
        self._done = False
        self._has_reset = True
        self.start_axle_y = self._axle_y()
        self.forward_displacement_m = 0.0
        self.forward_velocity_m_s = 0.0
        return self._observation(), self._info()

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        if self._closed or not self._has_reset:
            raise RuntimeError("Call reset() on an open environment before step()")
        if self._done:
            raise RuntimeError("Episode ended; call reset() before step()")
        command = np.asarray(action, dtype=float)
        if command.shape != (1,) or not np.isfinite(command).all() or np.any(np.abs(command) > 1):
            raise ValueError("Action must be one finite scalar in [-1,1]")
        self.last_action = float(command[0])
        # The motor map is speed-dependent. Refresh it at each MuJoCo physics
        # step while holding the policy's PWM command for the control interval.
        # Holding the torque itself for 10 ms can reverse a fast wheel before
        # the next update and create a numerical brake/accelerate oscillation.
        signed_pwm = self.last_action * self.settings["max_pwm_percent"]
        previous_axle_y = self._axle_y()
        if self.settings["motor_torque_update"] == "per_control_interval":
            torque = self.motor.pwm_to_torques(signed_pwm, *self._wheel_speeds())
            self.last_torques = (torque.left_nm, torque.right_nm)
            self.data.ctrl[self.actuator_ids] = self.last_torques
        for _ in range(self.substeps):
            if self.settings["motor_torque_update"] == "per_physics_step":
                torque = self.motor.pwm_to_torques(signed_pwm, *self._wheel_speeds())
                self.last_torques = (torque.left_nm, torque.right_nm)
                self.data.ctrl[self.actuator_ids] = self.last_torques
            mujoco.mj_step(self.model, self.data)
        if self.stationary:
            # mj_step integrates qpos/qvel after computing sensor/pose fields.
            # Synchronize all observations and reward metrics to the new state.
            mujoco.mj_forward(self.model, self.data)
        self.forward_displacement_m = self._axle_y() - self.start_axle_y
        self.forward_velocity_m_s = (self._axle_y() - previous_axle_y) / self.control_dt
        self.steps += 1
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError("Nonfinite MuJoCo state")
        self.last_reading = self.imu.update(
            self.data.sensordata[self.sensor_slices["imu_accel"]].copy(),
            self.data.sensordata[self.sensor_slices["imu_gyro"]].copy(), self.control_dt)
        observation = self._observation()
        info = self._info()
        values = self.settings["reward"]
        pitch_error = (info["imu_pitch_deg"] - self.settings["target_pitch_deg"]) / values["pitch_scale_deg"]
        pitch_rate = info["imu_pitch_rate_deg_s"] / values["pitch_rate_scale_deg_s"]
        wheel_speed = np.array([info["left_wheel_rad_s"], info["right_wheel_rad_s"]]) / values["wheel_speed_scale_rad_s"]
        reward = (values["alive"] - values["pitch_weight"] * pitch_error ** 2
                  - values["pitch_rate_weight"] * pitch_rate ** 2
                  - values["wheel_speed_weight"] * float(np.mean(wheel_speed ** 2))
                  - values["action_weight"] * self.last_action ** 2)
        if self.stationary:
            # Positive floor + bounded quality bonus: drifting never produces
            # unbounded negative rewards that make immediate falls attractive.
            cost = ((self.forward_displacement_m / values["position_scale_m"]) ** 2
                    + (self.forward_velocity_m_s / values["velocity_scale_m_s"]) ** 2
                    + values["pitch_weight"] * pitch_error ** 2
                    + values["pitch_rate_weight"] * pitch_rate ** 2
                    + values["wheel_speed_weight"] * float(np.mean(wheel_speed ** 2))
                    + values["action_weight"] * self.last_action ** 2)
            reward = values["alive"] + values["stationary_bonus"] * math.exp(-cost)
        terminated = info["total_tilt_debug_deg"] >= self.settings["fall_angle_deg"]
        truncated = self.steps >= self.max_steps
        self._done = bool(terminated or truncated)
        info["termination_reason"] = "fall" if terminated else "time_limit" if truncated else ""
        if not math.isfinite(reward):
            raise FloatingPointError("Nonfinite Wall-E reward")
        return observation, float(reward), bool(terminated), bool(truncated), info

    def close(self) -> None:
        if not self._closed:
            self.data = None
            self.model = None
            self._closed = True
