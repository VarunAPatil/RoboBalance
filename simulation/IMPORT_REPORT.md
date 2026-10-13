# Weighted Wall-E assembly: MuJoCo import report

**Historical report:** This records the earlier `Assembly-Wall-E` visual import. The active preview now uses `Updated-Correct-Wall-E-Assembly`; see [README.md](README.md) and [wall_e_preview_audit.json](models/wall_e_preview_audit.json) for its current counts and source. The old mass/COM figures below do not apply to the corrected assembly.

**Completed 2026-10-10. Result: an openable, static geometry-only MuJoCo preview and a separate corrected mass/COM audit.** This report records the visual import. [MASS_AND_COM_REPORT.md](MASS_AND_COM_REPORT.md) documents every later mass assignment, repeated-instance correction, COM calculation, and verification. Neither result is a dynamics model, motor simulation, or RL result.

## Source selected and preserved

I used `Assembly-Wall-E/assembly_1/urdf/assembly_1.urdf` and its sibling `meshes/` folder. Both existed at conversion time. I did **not** use the older `Wall-E-Assembly/` export, and I did not modify the source URDF or STLs. The converter prints the selected source and writes its SHA-256 to [wall_e_preview_audit.json](models/wall_e_preview_audit.json); the verifier checks that hash before checking the model.

The current weighted URDF has 553 links and 552 joints: 550 fixed joints and two continuous wheel joints. It references 412 visual mesh instances but only 404 unique binary STL files. There is no source collision geometry. Some links have no visual mesh and exist only as exporter placeholders; I did not create phantom shapes for them.

## Why direct URDF import was not the solution

The earlier direct MuJoCo URDF load stopped at `0603_resistor_004_2` with an inertia-eigenvalue error; enabling fixed-body fusion did not solve it. The precise Onshape exporter defect remains unconfirmed. The source also reports approximately **0.238833 kg** total mass, which is incomplete for the full visible battery group and undervalues the actual screws. Copying those individual inertias or merely increasing their mass numbers would not yield a defensible physical model.

The implemented result uses the source **visual geometry and placement**, while deliberately discarding source inertial blocks. It has no MuJoCo joints, actuators, contact simulation, or moving base. The MJCF compiler uses `inertiafromgeom="false"`; the verifier confirms that all compiled body masses and inertias are exactly zero. Every detailed visual geom uses `contype="0"` and `conaffinity="0"`. These choices make the scene static and prevent the visual meshes from inventing dynamics.

## What the converter does

[convert_urdf.py](convert_urdf.py) parses the URDF and checks that link names are unique, every joint parent and child exists, each child has one parent, the links form one connected tree, the joint types are supported, numeric transforms and inertial fields are finite, and mesh references resolve inside the selected local package. It checks binary STL size and finite vertices. The observed mesh scale is `1 1 1` for every source visual; the converter still handles positive per-visual scales and applies them once.

For each visual, the converter calculates the transform along the URDF tree:

```text
world visual pose = parent link pose × joint origin × visual origin
```

URDF roll/pitch/yaw is interpreted in radians as `Rz(yaw) × Ry(pitch) × Rx(roll)`. The resulting rotations are emitted as MuJoCo quaternions in `w x y z` order. Wheel joint axes are rotated through the joint frame and retained in the audit file. The converter leaves original STL vertices in place. MuJoCo compensates for its own internal mesh-asset recentering when compiling a geom; adding a second manual offset would move parts incorrectly.

All 550 fixed joints are contracted into **three named static bodies** in [wall_e_preview.xml](models/wall_e_preview.xml): `chassis_static` with 408 visual geoms, `wheel_left_static` with a rim and tire, and `wheel_right_static` with a rim and tire. Both wheel body frames are placed at the original continuous-joint hinge poses. The source hinge origins are about `[0.051316, -0.056690, -0.010970]` m and `[0.184316, -0.056690, -0.010970]` m; their world axes point approximately `+X` and `-X`. These frames are for later inspection and reconstruction; the preview has **no active wheel hinges**.

The XML uses 404 uniquely named mesh assets with relative paths back to the untouched STL folder. No ROS `package://` lookup is required. Reused assets remain separate visual instances: two motor assemblies, two rim/tire pairs, and four M3 screws. Conversely, the single `Battery18650__NONE_2.stl` asset already contains three cells, and the single `BatteryCaseSpring__Cut_Extrude2.stl` asset already contains six springs; neither was tripled or sextupled again.

Source RGBA colors are retained for 382 of the 412 visuals. The remaining 30 receive a fallback palette in the converter: blue cells, a dark holder, metallic springs/screws/motors, green PCB, and neutral gray elsewhere. The scene adds a softly lit sky, a visual-only checker floor, and fixed `three_quarter` and `battery_side` cameras. A single global translation of approximately `[-0.117816, +0.045935, +0.048430]` m centers the assembly in XY and puts its lowest mesh point 3 mm above the floor. The source geometry spans about **193 × 140 × 110 mm**; the translation changes display placement, not relative component placement.

## Mass information in the visual preview and follow-up audit

The visual-preview audit file records the user's targets without assigning them to the static MuJoCo scene. A later, **separate** calculation in [mass_audit.py](mass_audit.py) applies corrected masses and calculates an export-based center of mass (COM); it does not add mass, inertia, or joints to [wall_e_preview.xml](models/wall_e_preview.xml).

| Weighed item or group | Intended mass | Geometry represented |
| --- | ---: | --- |
| Three cells, holder plastic, and six springs together | 0.153 kg | One three-cell STL, one holder STL, one six-spring STL; use the user's single uniform effective density across the complete group |
| Each actual M3 screw | 0.0045 kg | Four separate visual instances of the screw STL |
| Each complete wheel | 0.040 kg | One rim plus one tire per wheel |
| Each complete motor | 0.029 kg | Shell, motor core, and modeled output shaft per side |
| PCB | 0.030 kg target | The user's populated-board measurement covers the board, daughterboard, and mounted components, excluding the separately weighed screws |

The battery/holder/spring STLs have a combined closed signed volume of approximately **64.805 cm³**. Under the agreed uniform-density assumption, 0.153 kg over that complete geometry corresponds to roughly **2360.9 kg/m³**. This density is used only in the mass audit, not as a body density in the MuJoCo preview. The audit also restores the mass and full-geometry COM of 30 repeated electronics visuals by matching their closed solids to 133 empty instance links. This raises the populated PCB branch from **26.432988 g** to **29.999964 g**. A tiny normalization to the user's measured **30 g** then completes that group. [MASS_AND_COM_REPORT.md](MASS_AND_COM_REPORT.md) gives the full component mapping and math.

In the selected 30 g populated-PCB scenario, the audit estimates **0.339033080 kg** for the complete exported assembly, including two 0.040 kg wheel groups. It estimates **0.259033080 kg** for the fused chassis group and places the assembly COM near `[0.117845321, -0.065601121, 0.006471398]` m in the source URDF root frame. These are **model-derived estimates**, not measurements of the assembled robot. The unnormalized, instance-reconstructed PCB scenario is also saved in [wall_e_mass_audit.json](models/wall_e_mass_audit.json). Neither scenario resolves the known invalid source inertias or proves that all source mass/COM data match real hardware.

## Files created and changed

| File | Purpose |
| --- | --- |
| [convert_urdf.py](convert_urdf.py) | Source audit and reproducible static MJCF conversion |
| [models/wall_e_preview.xml](models/wall_e_preview.xml) | Derived native MuJoCo scene |
| [models/wall_e_preview_audit.json](models/wall_e_preview_audit.json) | Source hash, counts, bounds, hinge frames, mass assumptions, remaining limits, and dependency versions |
| [mass_audit.py](mass_audit.py), [models/wall_e_mass_audit.json](models/wall_e_mass_audit.json) | Reproducible corrected-mass and COM calculation, separate from the static scene |
| [repeated_solids.py](repeated_solids.py), [verify_mass_audit.py](verify_mass_audit.py) | Explicit repeated-electronics reconstruction and checks of the final ledger |
| [models/wall_e_com_inspection.xml](models/wall_e_com_inspection.xml) | Static copy of the preview with calculated COM marker sites; still no dynamic inertials or joints |
| [MASS_AND_COM_REPORT.md](MASS_AND_COM_REPORT.md), [wall_e_com_inspection.png](renders/wall_e_com_inspection.png), [wall_e_com_battery_side.png](renders/wall_e_com_battery_side.png) | Detailed derivation and reviewed translucent COM views |
| [verify_preview.py](verify_preview.py) | Automated model, mesh-solid, and world-placement checks |
| [preview.py](preview.py) | Interactive viewer and offscreen render launcher |
| [wall_e_three_quarter.png](renders/wall_e_three_quarter.png), [wall_e_battery_side.png](renders/wall_e_battery_side.png) | Saved inspection renders |
| [requirements-preview.txt](requirements-preview.txt) | Pinned preview dependencies, separate from the RL package settings |

I also updated the existing project `README.md`, `AGENTS.md`, `docs/PLAN.md`, `docs/PROJECT_MAP.md`, and `docs/SESSION_CONTEXT.md` to distinguish this visual import from Phase 3 physical-model validation. No additional PDFs were created, and no PPO training or firmware was changed.

## Verification actually performed

1. Installed MuJoCo **3.7.0** into the project `.venv` without changing the RL package requirements. The recorded environment uses Python 3.10.12, NumPy 2.2.6, MuJoCo 3.7.0, and Pillow 12.3.0; `pip check` found no broken requirements.
2. Compiled the generated MJCF and ran `mj_forward`. It has **three fixed bodies**, **zero joints**, **412 mesh visual geoms**, **404 mesh assets**, and two cameras. All visual collision flags and all compiled body masses/inertias are zero.
3. Compared the compiled world-space mesh-vertex bounds against the bounds independently computed from the source URDF transforms, within `1e-5` m. The displayed bounds are approximately `[-0.09650, -0.06995, 0.00300]` to `[+0.09650, +0.06995, 0.11341]` m.
4. Checked key visual instances: two wheels with rims and tires, two motor shells/cores/shafts, main PCB and daughterboard, battery cells, holder, springs, and four screws. An exact-edge audit in the verifier finds **three closed cell solids**, **one closed holder solid**, **six closed spring solids**, and **one closed solid** in the reused screw STL; these key meshes have consistent winding and positive signed volume.
5. Rendered and visually inspected 1280 × 960 offscreen images from the three-quarter and battery-side cameras. The robot is recognizable, fully framed, upright, and above the floor. The batteries, holder, PCB, motors, and both wheels are visible.
6. Opened the original interactive MuJoCo window on X11 display `:1` and confirmed its process and X11 title `MuJoCo : wall_e_geometry_only_preview` at the time of that original visual-import run. This is historical evidence of a GUI launch, not a claim that the prior window remains open.
7. The later mass audit reconstructed **12 repeated-electronics families**, validated **30 merged visuals** and **133 no-visual placeholders**, and checked that the recovered populated PCB mass agrees with 30 g within about 36 micrograms. `verify_mass_audit.py` checks the corrected total, source and mesh hashes, MuJoCo compilation, and COM site coordinates. Seven focused analytic/integration tests passed. The COM inspection screenshot was rendered and inspected; the mass audit still supplies no dynamic inertial blocks.

## Reproduce or reopen

Run from the project root:

```bash
.venv/bin/python -m pip install -r simulation/requirements-preview.txt
.venv/bin/python -m simulation.convert_urdf
.venv/bin/python -m simulation.verify_preview
.venv/bin/python -m simulation.mass_audit
.venv/bin/python -m simulation.verify_mass_audit
.venv/bin/python -m unittest discover -s tests -p 'test_mass_audit.py' -v
.venv/bin/python -m simulation.preview
```

The viewer remains available until closed. To regenerate the two images without a GUI:

```bash
MUJOCO_GL=egl .venv/bin/python -m simulation.preview --offscreen
MUJOCO_GL=egl .venv/bin/python -m simulation.preview --offscreen --camera battery_side
```

To inspect the calculated COM markers in the separate translucent static viewer, run `.venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml`. The red, yellow, and cyan markers show the estimated chassis, complete assembly, and battery-group COM respectively. Marker sites and custom numeric mass metadata do not add physical mass or turn this into a dynamics model.

The remaining work for a dynamic balancing model includes physical measurement of assembled mass/COM, a defensible aggregate inertia tensor, motor response and polarity, battery voltage, contact/friction, sensor timing, and a deployable observation design that does not assume encoder-derived wheel state. None of that is claimed complete by this preview or export-based mass audit.
