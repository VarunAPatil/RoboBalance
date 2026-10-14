# Chat handoff — 2026-09-27

## Current status — URDF bundle rename, 2026-10-14

The active CAD package is now flattened into the root-level `Wall-E-URDF/` folder with `urdf/`, `meshes/`, and `launch/` directly inside it. The URDF/STL contents are byte-identical; only their paths changed. `simulation/convert_urdf.py`, the current project docs, audit metadata, and generated preview/COM/dynamic/balance MJCF mesh paths were updated. The converter still resolves the URDF's `package://assembly_1/meshes/` entries against `Wall-E-URDF/meshes/`. No runtime dependencies changed. Verification passed: preview, mass-audit, dynamic, and balance-IO checks; 27 pytest tests and 6 subtests.

## Latest status — structural mass reconciliation (2026-10-10)

The user supplied individual masses for the three structural URDF links: `part_1`=32 g, `part_1_1`=4.42 g, and `part_2`=4.42 g. `simulation.mass_audit` assigns these exact values and uses each transformed mesh centroid for COM; together they total 40.84 g. The selected 30 g PCB scenario estimates 379.839766 g for the assembly; the reconstructed-PCB alternative estimates 379.767823 g, within 0.000177 g of the Onshape screenshot's 379.768 g. The residual in the 30 g scenario comes from PCB reconstruction at 29.928057 g versus the rounded 30 g target. Both scenarios remain explicit. The static MJCFs carry mass and COM as `custom/numeric` audit metadata only; they still have zero compiled body mass/inertia and no dynamic joints. The active source is now `Wall-E-URDF/urdf/assembly_1.urdf`; `simulation.convert_urdf` resolves its meshes from the sibling `Wall-E-URDF/meshes/`. `verify_preview`, `verify_mass_audit`, dynamic and balance-IO checks, and 27 tests plus 6 subtests passed after regeneration.

## Purpose and working rules

The user is learning reinforcement learning while building a physical balancing Wall-E robot. Explain concepts from first principles with concrete observations, actions, and code; do not assume familiarity with PPO terminology. Read `AGENTS.md` before continuing and keep phases incremental.

The assembled robot uses `/home/varun/Wall-E-2027-Board-Design/sra_board.kicad_sch`, with ESP32-S3-WROOM-1 and an installed MPU6050. The schematic includes a TB6612FNG module; its physical marking remains to be verified. The TT motors have no encoders. Do not assume deployable wheel position or velocity measurements. Ask for measured hardware parameters before implementing a physical model or hardware-specific firmware. MuJoCo is the preferred Phase 3 engine. Keep safety outside the policy and simulation, training, and deployment separate.

## Existing project stage

Phase 1 CartPole PPO is documented as complete. Hardware characterization is next; the robot simulator and deployment code remain deferred. Consult `docs/PLAN.md`, `docs/HARDWARE.md`, `docs/PROJECT_MAP.md`, and `papers/README.md` for the plan, unknown measurements, layout, and staged reading path.

The saved experiment is `experiments/cartpole-ppo-seed42-100k/`, with `final_model.zip`, configuration, dependency records, checkpoints, curves, and evaluation outputs. The recorded 100,000-step request completed 100,352 steps. The documented independent evaluations each scored 500.0 ± 0.0 over 30 episodes. These are simulation/software results, not physical robot validation.

## What this chat covered

1. Read the working rules and project documentation to establish context.
2. Explained PPO's blocks: environment, observations, actor, critic, rollout buffer, advantages/GAE, probability ratio and clipping, losses, optimizer, minibatches, and epochs. The user said they did not understand PPO; continue with concrete examples rather than assuming the explanation settled everything.
3. Opened the project in VS Code.
4. Added a runnable live example to `docs/CARTPOLE_GYMNASIUM.md`, under “Live example: compare a fixed command with our trained PPO policy”.

## Live example and verification

The walkthrough includes a terminal command using `.venv/bin/python` that compares an always-right controller with the saved deterministic PPO policy. Both reset CartPole with seed 20042. It prints the initial observation, first five steps, final step, reward, termination flags, and total return. `RENDER = False` is the default; changing it to `True` enables a simulation window. The graphical mode was not tested in this chat.

The exact code extracted from the Markdown was executed successfully on 2026-09-26:

- Always right: 10 steps, return 10.0, `terminated=True`, `truncated=False`. Final angle −0.2561 rad exceeded the pole-angle limit.
- Saved PPO: 500 steps, return 500.0, `terminated=False`, `truncated=True`. It reached the episode time limit.
- Both initial observations, rounded: `[+0.0266, −0.0253, +0.0061, +0.0322]`.

The document explains units, reset and seeds, current versus next observation, why velocity matters, the 0.02-second Euler update in the installed CartPole source, failure reward, and inference versus training. The network's first actions alternate in this trace, but no alternating rule was programmed into the learned controller. Do not claim the trace reveals the network's internal reasoning.

## Relevant configuration

`configs/ppo.yaml`: CartPole-v1, seed 42, CPU, MlpPolicy, learning rate 0.0003, n_steps 1024, batch_size 64, n_epochs 10, gamma 0.99, gae_lambda 0.95, clip_range 0.2, ent_coef 0.0. One environment gives 16 minibatches per epoch and 160 optimizer steps per rollout. PPO clipping limits incentives in the objective; it is not a hard clamp on probabilities or weights.

`training/train.py` calls `PPO(...)` and `model.learn(...)`. `training/common.py` implements environment creation and deterministic evaluation. The installed virtual environment uses Python 3.10 (`.venv/lib/python3.10/`). A Git status attempt reported that this directory was not a Git repository; verify before claiming any commit or branch state.

## Where to resume

The latest request was to save progress in `AGENTS.md` and exit. That file now contains the updated handoff. Later discussion covered stochastic policies, Bellman equations, the training script and callback arguments, seeds, and training versus evaluation. Added `docs/BELLMAN_EXPLAINED.pdf`, `docs/CARTPOLE_WORKFLOW.pdf`, their editable LaTeX sources, and `training/demo.py` for numerical traces, action probabilities, optional rendering, and stochastic inference.

Headless demo modes were verified. A fresh 30-episode deterministic evaluation with seeds 30042–30071 scored 500.0 ± 0.0; this result was printed, not saved to a new evaluation JSON. GUI rendering remains untested, and pygame was absent at checking time.

The last conceptual clarification: rerunning the current training script starts fresh weights, rather than continuing an old model. No resume support has been implemented. Evaluation and demos do not change weights, even when actions are sampled stochastically. Additional training does not guarantee improvement.

No new training, hardware work, or simulator implementation is pending or authorized by the save-and-exit request. Continue explaining or demonstrating CartPole as requested; do not advance to hardware modeling until the required measurements are supplied.

## 2026-10-10 visual import update

The weighted assembly import has been implemented under `simulation/`; see `simulation/README.md` and `AGENTS.md`. It is a static geometry-only preview, with no dynamic inertias or PPO robot environment. `simulation/models/wall_e_preview_audit.json` is the reproducible source/assumptions record. Offscreen renders from two viewpoints were inspected. The interactive MuJoCo window was also opened and confirmed by its X11 window title/process on 2026-10-10; rerun `.venv/bin/python -m simulation.preview` if it has since been closed.

## 2026-10-10 mass/COM audit update

After clarifying that the hundreds of non-wheel visuals are grouped into one MuJoCo chassis body but retain zero mass in the preview, the user asked to calculate weights and COM and document the result in detail. `.venv/bin/python -m simulation.mass_audit` writes `simulation/models/wall_e_mass_audit.json` and a translucent, static `simulation/models/wall_e_com_inspection.xml` scene with colored COM sites. Open it with `.venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml`. The audit uses 153 g at one shared effective density across the complete three-cell, holder, and six-spring geometry; four 4.5 g screws; and the user's 30 g target for the populated PCB and mounted components, excluding screws. It restores 133 missing repeated electronic instances from 30 visual STLs, bringing the PCB from 26.432988 g to 29.999964 g before a tiny normalization to 30 g. Other URDF per-instance masses and inertial-origin COMs remain accepted under the user's statement. The selected corrected export-based estimate is **339.033080 g** with source-frame assembly COM `[0.117845321, -0.065601121, 0.006471398]` m; the chassis group is **259.033080 g**, each wheel 40 g. These are model-derived estimates, not physically validated robot mass/COM or a full inertia model. Both MuJoCo scenes remain massless and static; the separate colored geoms have not become one literal mesh because that would lose appearance and traceability. See `simulation/MASS_AND_COM_REPORT.md` for the exact per-family mapping, formulas, tests, and commands before advancing to dynamics.

The COM inspection scene was also rendered from three-quarter and battery-side cameras, visually checked, and opened in the interactive MuJoCo viewer on display `:1`. Its viewer session remained active when polled. Reopen with the command above if it has since been closed. `simulation.verify_preview`, `simulation.verify_mass_audit`, seven focused tests, and `pip check` passed. No additional PDF, PPO training, firmware, or dynamic physical model was created.

## Corrected assembly preview — 2026-10-10

The user replaced the assembly export with `Updated-Correct-Wall-E-Assembly/assembly_1/urdf/assembly_1.urdf`. `simulation.convert_urdf` now selects it by default and regenerates the active `simulation/models/wall_e_preview.xml`. The new source has 545 links, 542 fixed and two continuous joints, 404 visuals, and 396 referenced STLs. `simulation.verify_preview` passed for the corrected source, including three-body grouping, source hash, key components, wheel axes, shell counts, and compiled placement. Both camera renders were inspected. The corrected interactive viewer was opened on display `:1`, its X11 title was confirmed, and the process remained active when checked. The scene remains static and massless. At this point the old mass/COM audit had not yet been recomputed; the following section records that completed re-audit. `simulation/README.md` is the current handoff.

## Corrected assembly masses added — 2026-10-10

After the prior visual-only handoff, the user asked to add masses from the prior URDF. `simulation.mass_audit` has now been rerun against the corrected `Updated-Correct-Wall-E-Assembly` source, retaining the accepted CAD masses except for the confirmed 153 g battery/holder/springs group, four 4.5 g screws, and 30 g populated PCB target. The selected corrected-export result is **339.033080 g**, chassis group **259.033080 g**, assembly COM in the new source frame **`[0.117844354, -0.069104306, 0.006103539]` m**. The PCB reconstructs to 29.928057 g, then scales by 1.002403855 to 30 g. The JSON contains all 545 selected link mass/COM records; both preview XML files store group masses as custom numeric metadata, and the corrected COM inspection XML displays COM sites. `simulation.verify_mass_audit`, `simulation.verify_preview`, seven tests, and `pip check` passed; two COM renders were inspected. The interactive corrected COM viewer was opened on display `:1`, its title confirmed with `xwininfo`, and it remained active when polled. The earlier paragraph above is historical and has been superseded. The scenes remain physically massless and fixed because aggregate dynamic inertias are not validated.
