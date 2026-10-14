# What each directory does

The project has three stages of work. **Training** runs on the laptop and changes a neural network's weights. **Inference** runs a finished network to choose motor commands; the ESP32 will eventually do inference only. `simulation/` contains the visual import, provisional articulated model, sensor/controller scaffold, and diagnostic experiments. CartPole remains the verified first RL example. A separate **simulation-only** Wall-E PPO workflow now exercises the provisional MuJoCo model; robot deployment remains pending.

## The project root: `wall-e-rl/`

This is the Python project's home. Run the commands in `README.md` from here so relative paths such as `configs/ppo.yaml` and `experiments/` resolve correctly. `README.md` is the quick start; `pyproject.toml` declares the Python version and packages to install. `AGENTS.md` records project decisions and working rules for coding agents, including the confirmed hardware and the MuJoCo choice. `.gitignore` prevents generated environments and training runs from being included with source files.

`Wall-E-URDF/` is the active CAD source bundle used by the MuJoCo builders; it contains `urdf/`, `meshes/`, and `launch/` directly. `archive/cad/` holds earlier weighted and unweighted exports plus the original `Assembly 1.zip`; they are retained as source history, not selected by the active converter. The root-level empty `mujoco` file, empty placeholders in populated folders, and regenerated cache/database files were removed during the 2026-10-14 layout cleanup. The VS Code C/C++ browse index was scoped to `deployment/` to keep it from indexing the whole workspace.

Two generated directories may appear here after installation: `.venv/` contains an isolated copy of Python packages for this project, and `wall_e_rl.egg-info/` is package metadata created by the editable install. Neither contains robot logic. You can ignore both while learning the code.

## `configs/`: settings, not algorithms

- `ppo.yaml` tells `training/train.py` which experiment to run: CartPole, PPO settings, seed, number of training steps, and how often to save and evaluate. Changing this file changes an experiment without editing Python code.
- `wall_e_ppo.yaml` configures the separate Wall-E Gymnasium/PPO software experiment: MuJoCo model, motor-assumption file, action/observation scales, reward, episode limits, seed, checkpoints and evaluation.
- `robot.yaml` is a worksheet for the physical robot. Confirmed items include the ESP32-S3 board design, MPU6050, TT motors, and lack of encoders. It now also records user-provided component weights; unmeasured assembled mass, dimensions, battery type and voltage range, and verified installed screw/battery counts remain `null`. The CAD export shows four screw instances and three cells, which are not a physical count check. CartPole does not read this file. The future MuJoCo robot model will use measured values recorded here or in a dedicated simulation configuration.
- [`COMPONENT_WEIGHTS.csv`](COMPONENT_WEIGHTS.csv) records user-provided per-component weights and spreadsheet formulas. Its current four-screw and three-cell entries reflect CAD instances, not confirmed installed counts or a complete assembled mass. The mass audit instead treats cells, holder, and springs as one 153 g uniform-density group; it does not apply separate 45 g/18 g masses on top of that group.

The seed is a starting point for random-number generators. It makes runs easier to compare, though different computers or library versions can still produce somewhat different results.

## `training/`: teach and judge the policy on the laptop

An RL **policy** is the rule that maps a sensor observation to an action. In this experiment a neural network maps CartPole's position and motion measurements to a left or right push.

- `train.py` creates the CartPole environment and a PPO learner, runs training, saves model checkpoints, and evaluates the final model on separate episodes. PPO changes network weights here. For this small example, the program uses the CPU.
- `evaluate.py` loads a saved network and measures how well it works on new episodes. It does not change the weights. This resembles the future ESP32's inference role, though it still runs on the laptop.
- `demo.py` traces one episode, prints the actor's action probabilities, and optionally animates CartPole. It supports fixed commands and deterministic or stochastic saved-policy inference.
- `common.py` contains small functions shared by training and evaluation: loading settings, creating CartPole, running episodes, and writing results.
- `__init__.py` tells Python that these files form a package, enabling commands such as `python -m training.train`.
- `train_wall_e_ppo.py` and `evaluate_wall_e_ppo.py` separately train and evaluate a one-environment, continuous-action policy on the provisional Wall-E model. They do not alter CartPole code or the PID loop.

Why separate training and evaluation? A policy can look good on the episodes it learned from. Testing with different starting conditions gives a more credible measure of what it learned.

## `experiments/`: outputs from actual runs

Each training run creates its own subdirectory here. The verified 100,000-step CartPole run is `experiments/cartpole-ppo-seed42-100k/`. Within a run:

- `config.yaml`, `versions.json`, and `requirements-freeze.txt` record what was run and which Python packages were installed.
- `train_monitor.csv` and `validation_monitor.csv` record episode rewards and lengths.
- `tensorboard/` holds learning curves that you can view with `tensorboard --logdir experiments`.
- `checkpoints/` contains policies saved during training; `best/` contains the best policy according to validation episodes; `final_model.zip` is the policy at the end of training.
- `final_evaluation.json` records the final policy's results on held-out episodes. `holdout_evaluation_20042.json` came from a second evaluation command with another set of seeds.

These files are generated results. The source code lives in the other directories. A CartPole score of 500 means the virtual pole survived the maximum 500 steps; it says nothing yet about the physical robot.

`experiments/wall_e_ppo/` holds separate simulation-only runs. The first 256-step smoke run saved its config, versions, motor assumption snapshot, checkpoint, TensorBoard events, validation output, final policy, and held-out evaluations. All 30 episodes in the follow-up evaluation fell; the run verifies software flow, not balance.

## `simulation/`: visual import and future virtual robot

The main folder contains the converter, mass/dynamic/balance builders, viewer, PID runner, IMU/motor models, and `verify_*.py` checks. Optional search and telemetry scripts are grouped in `simulation/diagnostics/`; saved camera renders are in `simulation/renders/`. Run the optional tools with commands such as `.venv/bin/python -m simulation.diagnostics.log_gravity_imu --seconds 2` or `.venv/bin/python -m simulation.diagnostics.sweep_balance_pid_grid --help`. Historical report PDFs may show the former module paths; use the commands in `simulation/README.md` for current runs.

`convert_urdf.py` uses `Wall-E-URDF/urdf/assembly_1.urdf` and its sibling `meshes/` directory to build `models/wall_e_preview.xml`, a static geometry-only MuJoCo scene with three fixed groups. The corrected export has 404 visual instances: 400 colored chassis geoms and two per wheel. `preview.py` opens the viewer or saves offscreen renders; `verify_preview.py` checks source/mesh integrity, model compilation, grouping, placement, and key instances. `models/wall_e_preview_audit.json` records visual-import assumptions and unresolved dynamics.

`mass_audit.py` recalculates [models/wall_e_mass_audit.json](../simulation/models/wall_e_mass_audit.json) and [models/wall_e_com_inspection.xml](../simulation/models/wall_e_com_inspection.xml). With the supplied 32 g, 4.42 g, and 4.42 g structural-link corrections, the export-based total is about **379.8 g** depending on the selected PCB scenario. These static preview XMLs store audit metadata; their bodies remain physically massless. [simulation/MASS_AND_COM_REPORT.md](../simulation/MASS_AND_COM_REPORT.md) records assignments and limitations.

`simulation/IMPORT_REPORT.md` records the original visual conversion and window evidence. `simulation/MASS_AND_COM_REPORT.md` records the separate mass/COM derivation and validation. The estimate has not been checked against a physically weighed assembly and is not a validated full-body inertia model.

The root-level `environments/wall_e_balance_env.py` wraps the provisional balance XML in Gymnasium without changing the PID runner. It observes fused IMU pitch/rate plus simulator wheel speeds, turns one PPO action into bounded PWM and assumed N20 torques, and ends episodes on falls or time limits. See `simulation/README.md` for the exact path and caveats. This is a software interface milestone; the plant and deployable observation strategy remain unvalidated.

The dynamic MJCF is a provisional CAD/screenshot model, not a hardware-ready balance simulation. The sensor/controller/actuator scaffold is in `simulation/build_balance_model.py`, `balance_config.py`, `imu_model.py`, `balance_controller.py`, `motor_model.py`, `run_balance.py`, and `verify_balance_io.py`. The controller follows the firmware direction branch and integer gyro arithmetic is enabled by default. Each episode reset now seeds the complementary filter from gravity projected through the model reset pose, avoiding the known reset-time acceleration spike from being treated as the initial attitude; the unchanged CAD-derived `qpos0` is not a measured hardware stance. Normal runs retain firmware-style 60–80% PWM clamping. Optional `--tune` opens the MuJoCo viewer with a companion Tkinter panel for live setpoint, gains, fall angle, pause/resume, and manual reset, and defaults to a separately labeled simulation-only continuous-magnitude mode so gain changes can command less than 60% effort. `--pwm-output-mode firmware_clamped` restores the firmware-style mapping for comparison. Telemetry records the IMU seed and effective output mode. These changes make the software experiment interpretable; they do not calibrate the CAD-derived COM, identify motor physics, or establish balance. The IMU config also has an optional empirical one-pose startup-output transient updated to the latest capture set; it remains disabled by default and is not universal. Raw logs and summaries are under `experiments/hardware_imu/`; see `docs/HARDWARE.md` for measurements and limitations. `simulation/diagnostics/log_gravity_imu.py` records unpowered gravity-only IMU data; `simulation/diagnostics/sweep_balance_setpoints.py` runs reset-from-same-state setpoint trials and saves CSV plus an animated MuJoCo preview. The earlier sweeps used artificial torque/provisional signs and are not hardware evidence. Hardware-style motor mode still requires measured parameters, and contact/friction plus chassis impact geometry remain provisional. The separate `environments/` package now exercises the provisional model for software testing; it is not a characterized, deployable robot environment.

Think of MuJoCo as the physics engine and Gymnasium as the standard interface connecting that engine to PPO. The current static preview verifies visual import only. A future training model must include motor limits, latency, sensor noise, and uncertain parameters; a polished render alone would not make it a useful training model.

For an explicitly assumed N20-12V-200 motor experiment, `configs/mujoco_balance_n20_assumption.yaml` holds a spec-derived output-shaft curve without changing the default balance config. `simulation/diagnostics/sweep_balance_pid_grid.py` supports either that configured DC model or the older explicitly supplied ideal-torque fixture. The 2026-10-13 52,521-candidate run found no gains surviving 2 seconds; its best-ranked candidate fell at 0.35 seconds. See `docs/MUJOCO_N20_ASSUMED_PID_REPORT.pdf`; this remains simulation-only and does not complete hardware characterization.

`simulation/diagnostics/tune_balance_pid.py` is an offline randomized search utility. Its first wide search found a 12 s survivor only by targeting a ~30° lean; a 60 s follow-up spun wheels to ~491 rad/s. A separate upright-zero search fell within 0.63 s. These results show the ideal-torque plant and uncalibrated IMU reference are not yet suitable for claiming PID balance. Experiment details are in `docs/PLAN.md` and `simulation/README.md`.

The MuJoCo balance config now explicitly sets `angle_zero_deg: 0.0` and `pitch_setpoint_deg: 0.0`, making its CAD-derived `qpos0` pose the simulated upright reference. This is not a hardware balance-angle calibration.

## `deployment/`: future ESP32 inference

This area is also intentionally empty. A later `export_policy.py` will extract the trained actor's weights and required observation normalization from Python. `esp32/` will then hold ESP-IDF code for MPU6050 acquisition, pitch estimation, neural-network inference, motor output, telemetry, and independent safety cutoffs. PPO training and its critic will stay on the laptop.

We cannot choose a final robot network input list yet: the TT motors have no encoders, so wheel position and velocity are not directly measured. We must resolve that feedback problem before exporting a robot policy.

## `tests/`: checks that prevent misleading success

`test_wall_e_balance_env.py` checks Gymnasium reset/step behavior, seed reproducibility, pitch sign, motor action mapping, fall/time-limit endings, sensor reset, and finite random rollouts. A later policy-export test will compare Python actions against the same network executed in C on identical observations. Passing software tests does not establish physical validation.

## `docs/`: explanations and engineering decisions

The readable guides stay directly in `docs/`; scripts that rebuild their PDFs are grouped in `docs/builders/`. The source files and generated PDFs have not moved.

- [`MUJOCO_PREVIEW_PROMPT.pdf`](MUJOCO_PREVIEW_PROMPT.pdf) is the detailed implementation prompt for correcting the CAD import and opening a MuJoCo preview. `MUJOCO_PREVIEW_PROMPT.md` is the copyable source; `builders/build_mujoco_preview_prompt.py` regenerates the PDF. It specifies future implementation and does not constitute an implemented preview.
- [`MUJOCO_IMPORT_STEPS.pdf`](MUJOCO_IMPORT_STEPS.pdf) records the earlier installation check, raw URDF import failure, mass/inertia preparation, MJCF conversion, and viewer commands. `builders/build_mujoco_import_steps.py` rebuilds that historical guide. Since it was written, MuJoCo 3.7.0 has been installed into `.venv`, and the static visual model and separate mass/COM audit have been added; dynamic inertia and calibration remain pending.
- [`MUJOCO_STEPS.pdf`](MUJOCO_STEPS.pdf) gives the four-page Onshape assembly-to-MuJoCo workflow, required measurements, export options, and verification checkpoints. `builders/build_mujoco_steps.py` is its editable source and rebuild command; this is planning material, not an implemented robot simulator.
- `FUNDAMENTALS.md` explains observations, actions, rewards, episodes, policies, PPO, and training versus inference.
- [`RL_GLOSSARY.pdf`](RL_GLOSSARY.pdf) is a compact quick reference of basic RL terms with CartPole examples; `RL_GLOSSARY.md` is its editable source and `builders/build_rl_glossary.py` rebuilds the PDF.
- [`BELLMAN_EXPLAINED.pdf`](BELLMAN_EXPLAINED.pdf) explains value functions, Bellman equations, worked examples, and their connection to PPO; `BELLMAN_EXPLAINED.tex` is its editable source.
- [`CARTPOLE_WORKFLOW.pdf`](CARTPOLE_WORKFLOW.pdf) follows the exact training workflow and gives hands-on commands for the demo, evaluation, training, and TensorBoard; its editable source is `CARTPOLE_WORKFLOW.tex`.
- `CARTPOLE_GYMNASIUM.md` walks through the exact first experiment, including observations, actions, one step, one episode, and where PPO fits.
- `PLAN.md` gives the seven phases and the evidence needed to finish each one.
- `HARDWARE.md` separates confirmed physical facts from missing measurements.
- `PHASE1_RESULT.md` records the CartPole result and what it does and does not prove.
- `SESSION_CONTEXT.md` saves the 2026-09-27 chat handoff, including the live example, verification results, and where to resume.
- This file is the map of the repository and should change when its layout or purpose changes.

## `papers/`: a staged RL reading library

`README.md` gives a reading order, short explanations, and questions to answer after each stage. It links official beginner guides and holds local copies of the PPO, generalized advantage estimation, and dynamics-randomization papers. Start with the beginner guides; the research papers are for later understanding and sim-to-real design, not prerequisites for running CartPole.

## How one CartPole run moves through the folders

1. `training/train.py` reads `configs/ppo.yaml`.
2. It asks Gymnasium for CartPole, then PPO repeatedly observes the cart and pole, chooses a push, receives a reward, and updates the policy.
3. It writes curves, checkpoints, and results into a new `experiments/` subdirectory.
4. `training/evaluate.py` reloads a saved policy and tries new episodes without training.

For the provisional robot experiment, `environments/wall_e_balance_env.py` uses `simulation/models/wall_e_balance.xml`; `training/train_wall_e_ppo.py` is separate from CartPole. Phase 5 can move only a policy trained with real, available observations into `deployment/esp32/`; the current PPO observation includes unmeasured wheel speeds, so it is not export-ready.


Stationary simulation PPO additionally uses `configs/wall_e_stationary_ppo.yaml`
(six simulator observations and bounded position/speed reward),
`tests/test_wall_e_stationary.py`, and
`simulation/diagnostics/check_stationary_policy.py` (saved-policy timestep sweeps).
See `simulation/README.md` for commands and nondeployable observation limits.
