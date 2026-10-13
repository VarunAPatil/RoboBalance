# Wall-E assembly mass and center-of-mass audit

## Active corrected assembly — 2026-10-10 (updated after structural mass reconciliation)

The saved [mass audit](models/wall_e_mass_audit.json) and [COM inspection MJCF](models/wall_e_com_inspection.xml) now use `Assembly 1/assembly_1/urdf/assembly_1.urdf` and its 396 referenced meshes. Both the ordinary [preview XML](models/wall_e_preview.xml) and COM inspection XML embed group totals and all 545 link mass/COM records in MuJoCo `custom/numeric` fields. Each `audit_link_<link>` numeric stores `[mass_kg, COM_x_m, COM_y_m, COM_z_m]` in the source URDF frame; the COM inspection XML also shows calculated COMs as sites. These are audit metadata, not active body inertials. [mass_audit.py](mass_audit.py) saves the same selected records in JSON. Regenerate with `.venv/bin/python -m simulation.convert_urdf` followed by `.venv/bin/python -m simulation.mass_audit`, then check with `.venv/bin/python -m simulation.verify_mass_audit`. Open the markers with `.venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml`.

| Assignment | Corrected-assembly result |
| --- | ---: |
| Three cells, holder, six springs at one shared effective density | 153.000 g |
| Four screws, 4.5 g each | 18.000 g |
| Structural links `part_1`, `part_1_1`, `part_2` | 40.840 g |
| Populated PCB and mounted components, excluding screws | 30.000 g |
| Two motor assemblies, approximately 29 g each from the source solids | 57.999766 g |
| Two complete wheels, 40 g each | 80.000 g |
| Remaining source-assigned mass | approximately 0 g (included in structural correction) |
| **Assembly estimate (`pcb_30g`)** | **379.839766 g** |

The current source exports 238.761124 g before corrections. The battery branch's original mass is replaced once with 153 g, the four screw masses are replaced once with 18 g, the three structural link masses are replaced with their supplied individual masses (32 g, 4.42 g, and 4.42 g), and 133 repeated PCB solids embedded in 30 visual meshes are restored once. The corrected source also replaces the earlier eight extra `1x8_male_header_001` links with one complete closed header mesh; its source inertial COM agrees with that mesh centroid within 0.2 µm. The populated PCB branch reconstructs to **29.928057 g**; the selected `pcb_30g` scenario scales that branch by **1.002403855** to the user's approximate 30 g measurement.

The `pcb_30g` estimate is **379.839766 g**, about **0.071766 g above** the Onshape screenshot's 379.768 g. The separately saved `pcb_reconstructed` scenario is **379.767823 g**, about **0.000177 g below** that screenshot. This near-match suggests the Onshape total corresponds to the reconstructed 29.928057 g PCB ledger, while the user's rounded 30 g target adds about 0.072 g. Both scenarios remain visible; no other part is altered to force agreement.

For the selected `pcb_30g` scenario, the chassis group is **299.839766 g** at **`[0.117845030, -0.064973528, 0.010288299]` m**; the whole assembly is **379.839766 g** at **`[0.117838915, -0.067946574, 0.005621407]` m**. Each wheel remains a separate 40 g group. The `pcb_reconstructed` scenario estimates **379.767823 g**, within 0.000177 g of the Onshape screenshot's 379.768 g; the 30 g PCB-target scenario is 0.071766 g heavier. The selected three-part group uses the supplied exact masses: `part_1` 32 g, `part_1_1` 4.42 g, and `part_2` 4.42 g. Their COMs come from each transformed mesh centroid in the corrected URDF transform tree. The saved JSON records every selected link mass and COM so totals can be traced without double counting.

**Physical limit:** these are CAD-and-measurement-based estimates, not a weighed or balanced complete robot. The current MuJoCo preview and COM inspection XML still compile to zero body mass and inertia and have no active wheel joints or motor physics. The XML mass fields are inspection metadata; physically active body inertials have not been added because many electronics meshes are unsuitable for trustworthy aggregate inertia and the source contains invalid per-link inertia tensors. Validate assembled mass/COM and inertias before building a dynamic balance model.

## Earlier `Assembly-Wall-E` export — historical derivation

The section below records the original export and its earlier calculation. Its old coordinates and counts do not describe the active corrected assembly; links below to generated artifacts now open the new results.

This is the detailed record for the 2026-10-10 follow-up to the visual import. The selected source is the **weighted** `Assembly-Wall-E/assembly_1/urdf/assembly_1.urdf` plus its sibling `meshes/` directory. Neither source file nor any STL was edited. The source URDF SHA-256 is `9ba6b8e8227a92db7b93f3ab2c466906da33fcb68995453a1c2e4ef52c66d984`. The older `Wall-E-Assembly/` export is not used.

The result is a reproducible **mass and COM ledger**, [wall_e_mass_audit.json](models/wall_e_mass_audit.json), and a separate static [COM inspection scene](models/wall_e_com_inspection.xml). The original [visual preview](models/wall_e_preview.xml) still has zero compiled body mass and inertia. Neither MJCF file is a dynamic balancing model. Values below are calculated from CAD geometry, the user's stated group weights, and other URDF per-instance mass/COM entries; they are **not a physical measurement of the finished robot**.

## What is merged and what is not

The URDF has 553 links, 550 fixed joints, and two continuous wheel joints. The MuJoCo preview contracts all fixed links into **one rigid chassis body** with 408 attached visual geoms, plus one rim-and-tire body for each wheel. The wheels remain separate bodies at their original hinge frames. All 412 visual mesh instances and their placements remain present.

The chassis is one **body**, not one triangle-mesh asset. A literal single mesh would discard the source part colors in the current MuJoCo representation: [MuJoCo meshes carry shape while referencing geoms supply their color/material](https://mujoco.readthedocs.io/en/3.7.0/overview.html#mesh). It would also erase the useful link-to-part mapping used to audit mass. The many original visual geoms already move rigidly together because they share the chassis body. We therefore kept them separate, as allowed by the user's “if no problem” condition. The COM inspection copy makes the chassis geoms translucent to show the internal markers; it does not change the original colored preview.

## Which masses are used

| Group | Rule used in this audit | Result |
| --- | --- | ---: |
| Three cells, holder plastic, six springs | Complete closed geometry at **one shared effective density**, as requested | 153.000 g |
| Four M3 screws | Four actual visual instances, **4.5 g each** | 18.000 g |
| Populated main PCB, daughterboard, mounted modules/components | Recover repeated physical solids from the exported CAD per-instance masses; normalize the tiny rounding gap to the user's measured group target | 30.000 g |
| Two complete motors, shell/core/shaft on each side | Retain the source CAD subpart masses; each sum is 28.999883 g, which rounds to the supplied 29 g | 57.999766 g |
| Two complete wheels, rim and tire on each side | Retain the source CAD subpart masses; each sum is exactly 40 g | 80.000 g |
| Other source-assigned links | Retain source mass and inertial-origin COM under the user's statement that remaining per-instance values are correct | 0.033314 g |
| **Corrected exported assembly** | Sum of the rows above | **339.033080 g** |

The last “other” row is just **33.314 mg** in this export. This small remainder is a reason to check the real assembled robot for unmodeled wires, mounting parts, hardware, or CAD omissions. The [component CSV](../docs/COMPONENT_WEIGHTS.csv) lists 339 g for its named items at the CAD-visible counts; it does not independently verify a complete assembled mass. Its 18 g holder and three 45 g batteries also total 153 g, but **those separate weights are not added again** here. The requested battery/holder/spring group has one shared density, so its internal allocation differs from 18/45 g separate-part assignments.

### Battery, holder, and springs

`Battery18650__NONE_2.stl` contains **three closed cell solids**, not one. `BatteryCasePlastic__Fillet1.stl` contains one closed holder, and `BatteryCaseSpring__Cut_Extrude2.stl` contains **six closed springs**, not one. Their complete combined signed volume is about **64.8054 cm³**. The shared effective density is `0.153 kg / combined volume = 2360.914 kg/m³`. This is a user-specified *effective* density for this whole group, not a claim about the true material density of battery metal, plastic, or spring steel.

Applying that same density to each solid's mesh volume gives **116.873914 g** in all three cell solids, **35.327376 g** in the holder, and **0.798709 g** in all six springs. The sum is exactly 153 g to floating-point precision. The group COM in the source URDF root frame is approximately **`[117.734405, -66.056837, 12.428754] mm`**. The original battery branch represented only one cell and one spring in its mass data; its raw branch total was about 74.36455 g. The seven empty battery/spring placeholder links are set to zero in this corrected group, so they do not create phantom masses or a second copy of a solid.

### Four screws

The STL `Screw_M3_75mm.stl` is a single closed screw solid reused at four **different** URDF visual poses. Each visual instance receives 4.5 g once, for 18 g total. The old mass was only `6.03186e-7 kg` per screw, so changing the mass scalar alone would have left the wrong center of mass if its center were not checked. The audit uses each transformed screw mesh's own volume centroid. The four-screw group's source-frame COM is approximately **`[117.771237, -67.901805, 12.889911] mm`**.

### Populated PCB and repeated electronics

The user's 30 g PCB measurement includes the main PCB, daughterboard, and modules/components mounted on it. The URDF tree has a clear main-board branch rooted at `sra_board_unsaved_export_ok3bnj_pcb`: **529 links**, of which the four directly attached M3 screws are excluded because they have their own measured masses. The remaining **525 links** sum to only **26.432987624 g** if raw URDF link masses are naively added.

The missing mass is explained by the export's repeated-mesh representation. Some single STL visual assets contain multiple identical disconnected closed solids, while companion URDF links have **no visual** and a placeholder `1e-9 kg` mass. The visual link's original mass and inertial-origin COM refer to **one** of the identical solids. The audit validates each of these families: closed edges, opposite edge winding, positive per-shell volume, almost equal copied-shell volumes, and an original CAD COM within **0.646 micrometers** of one shell's mesh centroid. It then applies that link's per-instance mass to **every** actual shell and replaces the link COM with the full mesh's volume-weighted COM. The empty placeholders become zero. This reconstructs physical multiplicity without adding duplicate visual geometry.

| Repeated family | Visual STLs | Empty instance links | Mass restored (g) |
| --- | ---: | ---: | ---: |
| `8x1_male_header_pins` | 9 | 9 | 0.231152200 |
| `c_0805_2012metric` | 1 | 11 | 0.054950830 |
| `c_1206_3216metric` | 1 | 5 | 0.065596000 |
| `d_smb` | 1 | 1 | 0.061939400 |
| `jst_b2b_eh_a` | 1 | 2 | 0.268978000 |
| `led_0805_2012metric` | 1 | 66 | 0.240743580 |
| `msk12c02` | 8 | 8 | 0.047559339 |
| `pinheader_` | 3 | 7 | 1.021334000 |
| `pinsocket_` | 2 | 2 | 1.428544000 |
| `r_0805_2012metric` | 1 | 20 | 0.034566800 |
| `sot_223` | 1 | 1 | 0.065221900 |
| `sw_spst_pts810` | 1 | 1 | 0.046390500 |
| **Total** | **30** | **133** | **3.566976549** |

For example, the LED mesh has **67** closed LED solids, while its source mass describes one LED and 66 companion links have placeholder masses. The source LED COM is near one LED, not the whole 67-LED mesh. Counting only one LED would underweight the board and shift its COM. The maximum copied-shell volume ratio among the audited families is only **1.00001863**. [repeated_solids.py](repeated_solids.py) pins the exact family names and counts; it fails if the source structure changes. The JSON includes every visual link, placeholder, source and corrected mass, shell count, COM check, and mesh hash.

Restoring the repeats changes the populated-board branch from **26.432987624 g** to **29.999964173 g**. This independently agrees with the user's 30 g assembled-PCB measurement within **0.000035827 g**, or about 36 micrograms. The selected `pcb_30g` scenario scales the already reconstructed board entries by approximately **1.000001194** to make the group exactly 30 g; it does **not** apply a broad 26.433→30 g scale that would leave repeated mass at the wrong locations. The reconstructed-only scenario is saved as `pcb_reconstructed` alongside the selected `pcb_30g` scenario. The board COM is approximately **`[118.589651, -71.164981, 52.473723] mm`** in the source frame.

### Motors and wheels

Each wheel contains a rim and tire in one original wheel-joint group. The left wheel is `rim_1` + `tyre`; the right is `rim` + `tyre_1`. Each pair's source masses add to exactly 40 g. Each motor comprises a shell, core, and output shaft, and each set sums to **28.999883 g**. The suffixes do **not** pair left motor parts in the obvious way: the left shell's core and shaft have `_1` suffixes, while the right shell's core and shaft do not. [mass_audit.py](mass_audit.py) fixes this source-specific grouping explicitly and verifies complete closed solids and positive mass properties for the motor and wheel groups. The audit records their separate COMs, full component lists, volumes, effective per-solid densities, and mesh-derived group inertia tensors. These valid **group** tensors do not make the whole chassis inertia known.

## How COM and the validated group inertias are calculated

Every position is first expressed in the source URDF root frame, in **metres**. A visual mesh point is transformed by the full parent-link chain, the joint origin, and its visual origin. Mesh scale is applied exactly once. Roll/pitch/yaw follows `Rz(yaw) × Ry(pitch) × Rx(roll)` in radians. For uncorrected single-instance links, the source inertial-origin position supplies that link's COM, with its source mass retained under the user's instruction.

For a closed corrected mesh, each outward-facing triangle and the origin form a signed tetrahedron. Summing tetrahedron volume and first moments yields its solid volume and volume centroid; this uses **the actual triangle surface, not a convex hull**. For the shared-density battery group, `mass_i = shared_density × volume_i`. For a repeated electronic mesh, `mass_visual = shell_count × source_mass_per_instance`, and the full mesh volume centroid replaces the one-shell source COM. The PCB's tiny final normalization leaves its COM unchanged.

For any group, including chassis and whole assembly, the mass-weighted center is

```text
group COM = sum(component mass × component COM) / sum(component mass).
```

For the closed battery, screw, motor, and wheel groups only, the script also integrates each mesh's central inertia, rotates it into the common source frame, and adds the parallel-axis term `m × (|d|² I − d dᵀ)` for displacement `d` from the group COM. It checks finite, symmetric tensors; positive principal inertias; and the physical triangle inequalities. All calculations use **kg, m, and kg·m²**. The complete chassis inertia is **not** assembled or placed in MuJoCo because several other electronics meshes are nonmanifold and source individual inertias are invalid.

## Final numbers and coordinate frames

The selected `pcb_30g` ledger gives:

| Quantity | Corrected export-based value |
| --- | ---: |
| Chassis rigid group, excluding wheels | **259.033080 g** |
| Left wheel | **40.000000 g** |
| Right wheel | **40.000000 g** |
| Whole exported assembly | **339.033080 g** |
| Chassis COM in source URDF root frame | **`[117.854322, -68.353390, 11.858051] mm`** |
| Whole assembly COM in source URDF root frame | **`[117.845321, -65.601121, 6.471398] mm`** |
| Whole COM in display frame | **`[0.029216, -19.665983, 54.901033] mm`** |
| Whole COM above the model's wheel-axle line | **17.441495 mm** |

The display frame adds one global translation `[-0.117816105, +0.045935139, +0.048429635] m` so the mesh bounds fit the preview above the floor. It does **not** change the assembly's relative positions or the COM height above the axle. Source `z = 0` is not the floor. The mass estimate's milligram/micrometer display precision is computational precision, **not measurement accuracy**.

## What was verified, and what remains open

- `simulation.verify_preview` compiles the unchanged visual MJCF, runs `mj_forward`, confirms **3 fixed bodies, 0 joints, 412 visual mesh geoms, 404 mesh assets**, all visual collision flags zero, and world-space mesh bounds against the URDF transforms.
- `simulation.mass_audit` checks the corrected battery, screw, motor, wheel, and 30 repeated-electronics mesh solids. The repeated visual counts match the 133 no-visual PCB placeholders and the populated-board target after reconstruction.
- `simulation.verify_mass_audit` checks source and mesh SHA-256 hashes, the mass accounting identity (including exclusions), the 30 g board, 153 g battery group, four 4.5 g screws, two 40 g wheels, and the compiled static COM site positions. The inspection MJCF still has **zero body mass/inertia and zero joints**; its custom numeric entries are audit metadata, not physics parameters.
- Seven focused tests check analytic cube/tetra volumes and inertias, watertightness/winding rejection, disconnected-solid counting, rotated and parallel-axis aggregation, bad mass assignments, and saved audit mass balance. Offscreen inspection images were rendered and visually reviewed. The original visual and battery-side renders remain available.
- The interactive COM inspection MJCF was launched on desktop display `:1` with MuJoCo 3.7.0. The viewer reported that it was open and its command session remained active when polled. The GUI remains available until closed; this is separate from the offscreen PNG evidence.
- The project `.venv` records Python **3.10.12**, NumPy **2.2.6**, MuJoCo **3.7.0**, and Pillow **12.3.0** in the audit. No RL training, firmware, motor actuation, or contact dynamics was added.

What is **not** verified: the source CAD still has 40 nonmanifold mostly `COMPOUND_*` assets, and mesh edge checks do not prove every part is free of self-intersections or overlaps. The original raw URDF load failed MuJoCo's inertia-eigenvalue check at `0603_resistor_004_2`; its individual electronic inertias were not copied. The user's assertion that the other per-instance masses and COMs are correct is the working assumption for this *export-based* total. A physical scale measurement of the assembled robot, a physical COM check with batteries installed, and validated full-body inertia are still needed before using the robot for dynamics or RL. The TT motors have no encoders; wheel position/velocity are not deployable policy observations without measurement or a validated estimator.

## Reproduce and inspect

Run from the project root:

```bash
.venv/bin/python -m simulation.convert_urdf
.venv/bin/python -m simulation.verify_preview
.venv/bin/python -m simulation.mass_audit
.venv/bin/python -m simulation.verify_mass_audit
.venv/bin/python -m unittest discover -s tests -p 'test_mass_audit.py' -v
.venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml
```

The last command opens a **static**, translucent inspection scene with a red site at the chassis COM, a yellow site at the whole-assembly COM, and a cyan site at the battery-group COM. Thin matching leader sites reach above the PCB so the nearby COM positions can be found in the viewer; the spheres at their bases are the actual computed positions. [wall_e_com_inspection.png](renders/wall_e_com_inspection.png) and [wall_e_com_battery_side.png](renders/wall_e_com_battery_side.png) are the saved three-quarter and battery-side screenshots. For a fresh offscreen render without a desktop window:

```bash
MUJOCO_GL=egl .venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml --offscreen --output simulation/renders/wall_e_com_inspection.png
MUJOCO_GL=egl .venv/bin/python -m simulation.preview --model simulation/models/wall_e_com_inspection.xml --offscreen --camera battery_side --output simulation/renders/wall_e_com_battery_side.png
```

To return to the fully colored geometry preview, run `.venv/bin/python -m simulation.preview`.
