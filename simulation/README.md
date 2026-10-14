# Wall-E MuJoCo preview

Core conversion, model building, IMU/motor behavior, the PID runner, and the viewer live directly in `simulation/`. Optional offline sweeps and gravity logging live in `simulation/diagnostics/`; the generated XML and audits live in `simulation/models/`; saved inspection screenshots live in `simulation/renders/`. The `verify_*.py` commands remain at the package root so model checks are easy to find. The older CAD exports are preserved in `archive/cad/`; the active source is `Wall-E-URDF/`.

The active visual preview is generated from `Wall-E-URDF/urdf/assembly_1.urdf` and its sibling `Wall-E-URDF/meshes/` directory. The URDF and mesh export files are unchanged. Regenerate and check the model from the project root:

```sh
.venv/bin/python -m simulation.convert_urdf
.venv/bin/python -m simulation.verify_preview
.venv/bin/python -m simulation.mass_audit
.venv/bin/python -m simulation.verify_mass_audit
.venv/bin/python -m simulation.preview
```

For saved inspection images, use `.venv/bin/python -m simulation.preview --offscreen --camera three_quarter` or `--camera battery_side`. The active MJCF is [models/wall_e_preview.xml](models/wall_e_preview.xml), and [models/wall_e_preview_audit.json](models/wall_e_preview_audit.json) records its source hash, geometry counts, wheel frames, bounds, display transform, and dependency versions. Mesh paths in the MJCF are local relative paths; ROS package lookup is not needed.

The corrected export has 545 links, 542 fixed joints, two continuous wheel joints, and 404 visual instances using 396 STL assets. The converter follows each URDF parent-joint-visual transform and applies mesh scale once. It contracts the fixed links into three **static** MuJoCo bodies: a chassis with 400 colored visual geoms and two wheel bodies with two geoms each. The wheel hinge frames and axes are retained in the JSON for inspection, but the MJCF has no active joints, inertias, actuators, or contacts. A single display translation centers the robot and places its lowest point 3 mm above the visual floor. Source colors are used where present; `convert_urdf.py` supplies a fallback palette where absent.

The ordinary preview is geometry-only. The corrected [mass ledger](models/wall_e_mass_audit.json) assigns the user-provided 153 g battery/holder/springs group, four 4.5 g screws, structural links `part_1`=32 g and `part_1_1`=`part_2`=4.42 g each, two 40 g wheels, two 29 g motors, and the populated PCB. The selected 30 g PCB scenario estimates 379.839766 g total; the reconstructed PCB scenario estimates 379.767823 g, closely matching the 379.768 g Onshape screenshot. Running `mass_audit.py` adds group totals and **all 545 link mass/COM records** as MuJoCo `custom/numeric` fields to both preview XMLs. These are audit metadata, not active MuJoCo inertials. Open the COM marker scene with `.venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml`. Re-run `mass_audit.py` after regenerating the visual XML, because the converter rebuilds that derived file.

To watch the dynamic/balance model advance without a controller or motor command, run `.venv/bin/python -m simulation.preview --model simulation/models/wall_e_balance.xml --simulate`. This steps MuJoCo under gravity until the viewer is closed. For a headless snapshot after two seconds, add `--offscreen --simulate --seconds 2 --output /tmp/wall_e_gravity.png`. This is an unpowered passive-fall/contact demonstration, not a PID or physical validation; the chassis has no collision proxy and may pass through the floor after tipping.

To run the same unpowered gravity simulation and record firmware-style MPU6050 values, use `.venv/bin/python -m simulation.diagnostics.log_gravity_imu --seconds 2`. It writes a timestamped CSV under `experiments/mujoco_gravity/`; set `--output /tmp/imu_gravity.csv` to choose another path. The log contains MuJoCo accelerometer/gyro values, MPU-like integer counts, accelerometer pitch, complementary-filter pitch, debug chassis pitch, and contact count. No PID or motor torque is applied. Current zero bias/noise settings represent an idealized sensor, and the chassis collision proxy and physical sensor parameters are not fully validated.

For a diagnostic setpoint sweep with a rendered preview, run `.venv/bin/python -m simulation.diagnostics.sweep_balance_setpoints --ideal-test-torque-nm 0.02 --left-sign -1 --right-sign 1`. The runner defaults to 23–30°; choose another range with `--start`, `--stop`, and `--step`. Add `--no-preview` for broad sweeps to save only telemetry and summary CSVs. Each run also records `run_metadata.json` with the model hash, controller/sensor configuration, explicit test torque and signs, dependency versions, sweep range, and stability screen. The full sensor pitch range (−90° to +90°) was swept at 1° increments for 5 s per candidate on 2026-10-12; none passed the stability screen. The closest signed mean pitch error was misleading because the robot oscillated through near-fall angles. See `experiments/mujoco_setpoint_sweep/20261012T134526Z/`. Torque scale and signs are artificial/provisional test inputs, not measured motor characteristics. A passing numerical screen would still not establish a hardware PID setting. Adjust torque scale, signs, or gains only as separate documented experiments, not while comparing setpoints.

The corrected-export estimate uses other source masses and COMs under the user's stated assumption that they are correct; it is not a weighed or dynamically validated robot. Detailed assignments and limitations are in [MASS_AND_COM_REPORT.md](MASS_AND_COM_REPORT.md). The **static preview and COM inspection scenes** have fixed bodies and zero compiled body mass/inertia: their `custom/numeric` records are inspection metadata, not physical dynamics. The separate dynamic and balance XMLs have provisional inertials and joints. Invalid individual component inertias are not copied. [IMPORT_REPORT.md](IMPORT_REPORT.md) records the earlier visual import and is historical.

## Sensor and actuator scaffold

`build_balance_model.py` creates a separate `models/wall_e_balance.xml` from `models/wall_e_dynamic.xml`; it never edits the generated dynamics source or static preview. It adds one MPU site, MuJoCo accelerometer/gyro sensors, and one direct-torque actuator per wheel hinge. The current site uses the user-confirmed `Board_001` IMU body center and maps IMU X backward to chassis -Y, IMU Y right to chassis +X, and IMU Z up to chassis +Z. This assumes chassis +X points from the left wheel to the right wheel and chassis +Y is forward. The dynamic builder places the tire proxies in approximately 0.01 mm initial contact with the floor; this removes the previous free-fall gap but is not a validated tire preload or a stable initial balance state. `verify_dynamic.py` checks that contact. The separate balance XML is already generated from this configuration.

`imu_model.py` converts MuJoCo SI sensor readings into MPU6050-like counts and calculates pitch using the firmware's `atan2(-ax, sqrt(ay²+az²))`, gyro-Y integration, and alpha 0.995 complementary filter. At each initial/manual/fall reset, `run_balance.py` seeds the fused angle from gravity projected through the reset-pose IMU frame, then uses full MuJoCo accelerometer/gyro samples. This prevents a reset-time angular-acceleration spike from being mistaken for the initial pose; the raw `pitch_accel_deg` remains in telemetry so the difference is visible. The config explicitly sets `angle_zero_deg: 0.0` and `pitch_setpoint_deg: 0.0`, defining the model's `qpos0` reset pose as simulated upright. This is not a measured hardware balancing pose. The IMU model also supports other angle-zero calibrations and optional injected biases/noise. The default zero biases/noise are ideal-sensor settings, not measurements. `gyro_integer_truncation` defaults on to reproduce the C code's integer `gyro_counts / 131` conversion; disable it only for a deliberately non-firmware comparison.

The config records an empirical fused-output startup transient updated from the latest three hardware captures in `experiments/hardware_imu/repeat_20261012T132500Z/`: initial offsets of +13.86° roll and −1.50° pitch, exponentially decaying with a 9.2 s time constant toward approximately −13.70° roll / −0.31° pitch. Set `startup_transient_enabled: true` to replay that measured pose; it is **disabled by default** for balance runs because this was a horizontal, non-balancing pose. A prior capture set settled near −20.25° / +6.19°, indicating that the orientation was not reproduced across sessions. This overlay is separate from filter state and raw sensor biases because the captures recorded fused angles only; the physical cause is unknown. Do not generalize it to other poses. Simulation telemetry records both roll and pitch estimates.

`balance_controller.py` reproduces the firmware's sample-based PID: it accumulates error and differences consecutive errors without multiplying/dividing by `dt`. The default `firmware_clamped` output mode applies the firmware-style 60–80% PWM clamp outside the ±1° deadband; direction follows the pitch-error branch (positive error -> backward; negative error -> forward), not the sign of the PID sum. This means gain changes cannot request less than 60% PWM while error is active. The optional `continuous` mode sends the bounded PID correction magnitude (0–80%) to the motor model without that 60% floor, making gain response observable for simulation experiments. `--tune` selects `continuous` by default; pass `--pwm-output-mode firmware_clamped` to compare against the firmware-style clamp. Continuous mode is deliberately not firmware-equivalent and, with the current ideal-torque option, is not a physical motor model. `motor_model.py` separates signed PWM from torque mapping. Its ideal-torque mode is only an explicitly artificial sign/plumbing test and requires an explicit test torque and left/right hinge signs. Its DC-motor mode refuses to run until resistance, torque/back-EMF constants, battery voltage, ratio, efficiency, torque limit and dead zone are configured. No TT motor values are guessed.

`run_balance.py` advances physics at the model timestep and updates control at the configured period. It resets MuJoCo state, IMU filter state, and PID history when total chassis tilt reaches `simulation.fall_angle_deg` (45° by default), without closing the MuJoCo window. PID gains and pitch setpoint can be overridden for a run; the effective settings are saved with its telemetry under `experiments/mujoco_balance/`:

```bash
.venv/bin/python -m simulation.run_balance \
  --setpoint-deg 25 --kp 11 --ki 0 --kd 4 \
  --fall-angle-deg 45 --ideal-test-torque-nm 0.02 \
  --left-sign -1 --right-sign 1 --viewer
```

The torque value and signs above are demonstration inputs only, not characterized Wall-E motor parameters or a recommended tuning. For normal use, select verified signs and a measured DC motor model before interpreting balance behavior. Rebuild the configured balance model and run its plumbing checks with:

```sh
.venv/bin/python -m simulation.build_balance_model
.venv/bin/python -m simulation.verify_balance_io
```

For live tuning, add `--tune` and omit `--seconds`. This opens the MuJoCo viewer and a companion **Wall-E PID Tuner** window that stays open until either window is closed. It uses continuous motor effort by default so Kp/Ki/Kd changes can produce sub-60% commands; the panel labels this as simulation-only behavior. Its sliders change setpoint, Kp, Ki, Kd, and fall-reset angle while the simulation is running. Pause to inspect a pose, use `Reset pose` to restart the current episode without closing either window, and continue; automatic fall resets preserve the selected tuning and reseed the IMU from the same model reset pose. The per-sample telemetry records the live settings, IMU seed angle, and reset reason, while `config_final.yaml` plus `run.json` save the final values and output mode. The PID panel is a separate window because MuJoCo's Python viewer does not expose custom PID widgets in its native side panels.

```bash
.venv/bin/python -m simulation.run_balance --tune \\
  --setpoint-deg 0 --kp 2 --ki 0 --kd 0.2 \\
  --ideal-test-torque-nm 0.02 --left-sign -1 --right-sign 1
```

The gains are just gentle starting values for seeing the continuous output change; they are not a recommended balance tune. The example torque and signs are only for exercising the UI/reset plumbing; they are not verified motor parameters. For a firmware-clamp comparison, use `--pwm-output-mode firmware_clamped`. A headless diagnostic run defaults to the configured firmware clamp and requires an explicitly chosen ideal torque scale and simulated hinge signs:

```sh
.venv/bin/python -m simulation.run_balance --ideal-test-torque-nm <test_torque_nm> --left-sign <verified_left_sign> --right-sign <verified_right_sign>
```

`VALUE` and the signs are intentional test choices, not measured parameters. For a hardware-style PWM-to-torque simulation, characterize the motors and set `motor.mode: dc_motor`. The user-provided N20-12V-200 spec image has a separate **simulation-only assumed** curve in `configs/mujoco_balance_n20_assumption.yaml`; the default `configs/mujoco_balance.yaml` is unchanged. The image's rated-speed and no-load-speed values both say 200 RPM, so the curve uses 200 RPM as the no-load endpoint and does not treat the listed 0.52 kg-cm rated torque as a hard limit. Do not interpret this config as measured motor data.

To launch the slider preview with that assumption, use `.venv/bin/python -m simulation.run_balance --tune --config configs/mujoco_balance_n20_assumption.yaml --pwm-output-mode firmware_clamped`. The finite-grid runner can consume the same DC config without ideal-torque/sign CLI overrides. Its 2026-10-13 full grid tested 52,521 combinations at a fixed 0° setpoint; none survived 2 s, and the best candidate fell at 0.35 s. See `docs/MUJOCO_N20_ASSUMED_PID_REPORT.pdf` and the saved CSV/telemetry under `experiments/mujoco_pid_grid/20261013_n20_assumption_full_firmware_grid/`. `verify_balance_io.py` and the unit tests check software plumbing, not physical response or balancing. The CAD dynamics, impact geometry, contact, motor identity/signs, and inferred motor parameters remain unvalidated.

An offline randomized PID/setpoint search is available as `.venv/bin/python -m simulation.diagnostics.tune_balance_pid`. It requires explicit artificial torque and hinge signs, and writes candidate tables, validation, metadata, and best-candidate telemetry under `experiments/mujoco_pid_search/`. It ranks survival first, then late-window pitch error/span, final wheel speed, and effort. Example search command (the signs and torque below are only the particular simulation fixture tested):

```sh
.venv/bin/python -m simulation.diagnostics.tune_balance_pid \
  --trials 700 --seconds 2.5 --validation-seconds 12 --top 20 \
  --setpoint-min-deg 15 --setpoint-max-deg 40 \
  --ideal-test-torque-nm 0.02 --left-sign 1 --right-sign -1
```

The 2026-10-13 search found a 12 s non-falling candidate at setpoint 30.637°, Kp 3.7754, Ki 0.00897, Kd 11.2316; a 60 s follow-up kept four contacts but drove the wheels to about 491 rad/s and the chassis to about 28° pitch. This is not balance: the setpoint is a lean target, and the artificial constant-torque actuator has no measured motor back-EMF or speed limit. An upright-zero search found no candidate surviving longer than 0.63 s. Do not copy the leaned candidate into the default config or hardware. This provisional plant cannot yet support a meaningful “best PID” claim; measure/calibrate motor response and the balance reference, then retune.

## First simulation-only PPO workflow

The separate Gymnasium environment is `environments/wall_e_balance_env.py`; it uses `models/wall_e_balance.xml`, `imu_model.py`, and `motor_model.py`. It never calls the PID controller or opens a window during training. `configs/wall_e_ppo.yaml` chooses a **10 ms control interval** (five 2 ms MuJoCo steps), **±3°** starting-pitch randomization, **45°** fall termination, a **5 s** episode limit, and up to **80% signed PWM**. The motor model and assumed left/right signs come from `configs/mujoco_balance_n20_assumption.yaml`, not measured hardware.

One PPO action is a float in `[-1,1]`. The environment multiplies it by the configured 80% PWM limit, passes the signed value and current wheel speeds to `WallEMotorModel.pwm_to_torques`, writes the two returned N·m values to the named MuJoCo torque actuators, then advances five physics steps. Afterwards the simulated accelerometer and gyro update `WallEImuModel`; PPO receives fused pitch, pitch rate, and two wheel angular speeds. Before PPO sees them, these four values (raw units **deg, deg/s, rad/s, rad/s**) are divided by `[20,100,30,30]`. The reward adds 1 per alive control step and subtracts configurable squared penalties for pitch error (upright target), pitch rate (oscillation), wheel speed (runaway travel), and action magnitude (effort). Debug chassis angles only decide fall termination and evaluation statistics; they are never observations.

Reset uses the MJCF `qpos0` pose with zero velocity, rotates it around the named wheel-axle line, clears the IMU filter, and seeds fused pitch from gravity in the reset-pose IMU frame. Positive reset pitch leans backward; negative is forward/nose-down by the project's convention. Fall **terminates** an episode; reaching 5 s **truncates** it. PPO uses a single headless environment and separate validation and held-out seeds. Wheel speeds are privileged simulator readings: this policy **cannot be deployed** to the current encoderless TT-motor robot.

From the project root:

```sh
.venv/bin/python -m pytest -q tests/test_wall_e_balance_env.py
.venv/bin/python -m training.train_wall_e_ppo --timesteps 256
.venv/bin/python -m training.evaluate_wall_e_ppo experiments/wall_e_ppo/<run>/final_model.zip --episodes 30 --seed 72042 --tilts-deg=-3,-1,0,1,3
# Optional GUI during evaluation only:
.venv/bin/python -m training.evaluate_wall_e_ppo experiments/wall_e_ppo/<run>/final_model.zip --episodes 5 --viewer
```

The 256-step smoke run under `experiments/wall_e_ppo/seed42042_20261013T184433896222Z/` saved the resolved config, seed/versions, model and motor hashes, a motor-config snapshot, checkpoint, validation policy/log, TensorBoard data, final policy, and independent evaluation. Its 10 fresh-seed final-policy episodes all fell (mean survival **0.187 s**). A separate 30-episode evaluation at seeds 72042–72071 with reset tilts cycling through −3°, −1°, 0°, +1°, +3° also had **100% fall rate**, mean survival **0.188 s**, and maximum observed wheel speed about **5.82 rad/s**. These are software-pipeline results, not learned balancing or physical evidence. Phase 3 plant/contact validation and a deployable observation strategy remain open.


## Stationary PPO experiment (simulation only)

`configs/wall_e_stationary_ppo.yaml` selects a separate six-observation task:
fused IMU pitch error / 5 degrees, pitch rate / 100 degrees/s, each wheel speed /
10 rad/s, axle displacement from reset / 0.05 m, and interval-average axle speed /
0.1 m/s. Position and speed are privileged simulator measurements, not an
encoderless hardware estimator. Existing four-observation survival policies
still use their saved configurations.

Train with `.venv/bin/python -m training.train_wall_e_ppo --config configs/wall_e_stationary_ppo.yaml`.
To inspect the selected saved run, pass its full model path; the evaluator finds
the run's `config.yaml` automatically:

```sh
.venv/bin/python -m training.evaluate_wall_e_ppo \
  experiments/wall_e_ppo/seed42042_20261013T215415483354Z/best/best_model.zip \
  --seconds 60 --episodes 1 --seed 112042 --viewer
```

Each invocation of the trainer starts fresh.

The stationary reward is `0.1 + 0.9 * exp(-cost)`, where cost includes squared
scaled displacement, speed, fused pitch error, pitch rate, wheel speed and action.
The positive floor removes the previous unbounded negative reward problem;
position and velocity terms make cruising less rewarding than remaining nearby.
The current config strengthens the speed scale to 0.05 m/s and uses learning
rate 0.0001. Saved run configs retain earlier settings.

One action becomes signed PWM limited to 40%, then the existing motor map
recomputes torque from wheel speed every 2 ms physics step while PWM stays
constant for 10 ms. The stationary task calls `mj_forward` after integration to
synchronize sensor/pose fields before reading observations. Static friction at
rest is now limited to the requested electromagnetic torque: a sub-breakaway
command produces zero net motor torque instead of incorrectly producing reverse torque. This does not lock the joint against external forces. Published
N20 endpoints were not changed; the interpolation, friction/contact model and
wheel signs remain provisional.

The CAD COM requires about 32.514 degrees of chassis pitch to lie above the axle;
the experiment resets at 32.5 +/- 3 degrees. This is a CAD-derived balance pose,
not a measured hardware offset or an arbitrary reward-only lean. Stationary
success requires a complete episode, <= 5 cm maximum axle excursion, <= 2 cm/s
settled RMS speed and <= 2 degrees settled RMS pitch error, measured after 1 s.
These chosen tolerances are recorded in evaluation JSON. They are stricter than
merely not falling. Only forward station keeping is trained; disturbance,
turning and hardware robustness are not established. Chassis impact proxies
remain absent; active collision geometry is the floor and two tire proxies.

For numerical sensitivity, run `.venv/bin/python -m simulation.diagnostics.check_stationary_policy RUN/best/best_model.zip --config RUN/config.yaml --seconds 30 --output RUN/timestep_check`.
It saves CSV traces at 2, 1 and 0.5 ms physics intervals with the same 10 ms policy
interval and same reset seed. The trainer saves source snapshots/hashes as well
as config, versions, checkpoints and independent results. Refer to these when
replaying older experiments; the motor friction correction changes the code.


## Stationary simulation PPO result — 2026-10-14

The separate `configs/wall_e_stationary_ppo.yaml` adds simulator-only axle position
and velocity to the original IMU/wheel observations (six values), with bounded
position/speed reward and 40% PWM cap. It uses the CAD COM-over-axle reset angle
32.5 +/- 3 degrees. Per-physics-step motor torque updates are retained; stationary
observations synchronize pose/sensor fields after integration. A zero-speed motor
friction bug was corrected: sub-breakaway electrical torque no longer produces
reverse net motor torque. Manufacturer N20 endpoints and physical model XML are
unchanged; motor interpolation/contact and encoderless sensing remain unvalidated.

Selected run: `experiments/wall_e_ppo/seed42042_20261013T215415483354Z`; use `best/best_model.zip` at 114,688 steps, selected by
validation reward. Its final checkpoint at 200,704 steps degraded (up to 32.5 cm
excursion in 10 s), so do not substitute `final_model.zip`. Fresh deterministic
tests passed 20/20 at 30 s and a further 20/20 at 60 s: no falls, maximum axle
excursion 4.32 cm/4.30 cm, worst settled RMS speed 1.884/1.793 cm/s respectively.
Success thresholds remain 5 cm excursion, 2 cm/s settled RMS speed, 2 degrees
settled RMS pitch error after 1 s, and full-duration survival. These are chosen
simulation tolerances, not hardware specifications. One-seed 30 s checks at 2,
1 and 0.5 ms physics steps all survived with no warnings and <=4.16 cm excursion.
27 tests plus 6 subtests, SB3 checker/random-action checks and balance IO/dynamic
verification passed. Source snapshots, logs and independent results are saved;
`stationary_result.json` summarizes the experiment and limitations. Commands are
in `simulation/README.md`. Small corrective oscillation remains, and no disturbance
or sim-to-real robustness is established. CartPole, firmware, CAD and XMLs were
not modified in this stationary experiment.
