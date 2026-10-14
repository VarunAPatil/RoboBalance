# Onshape inertia screenshots: read-only audit

Source images: `inertias/Full_body_inertia.png`, `inertias/wheel_left_inertia.png`, and `inertias/wheel_right_inrtia.png`. The user directs us to retain the corrected masses in the current MuJoCo mass audit. **No dynamic model or inertial blocks were created from these screenshots.**

## What the images actually show

All three dialogs name **“Mate connector 1 of Rim <2>”** as the reference frame in the same Onshape Assembly 1 document. This is evidence of the same *named* reference, but the screenshots do not expose its pose relative to the exported URDF root or prove that the assembly/configuration and connector orientation stayed unchanged between captures. The wheel images explicitly select `Rim <1>` + `Tyre <1>` and `Tyre <2>` + `Rim <2>`, respectively. The full image shows a scrolling selection list (visible entries start with `COMPOUND <94>`, `<87>`, `<92>`, `<96>`); highlighted batteries and screws are visible in the assembly tree. `Instances (544)` is the tree's instance count, **not proof that all 544 are selected**. Part-level mass assignments and overrides are not exposed by these images.

The following values are transcribed exactly as displayed. Mass is in **g**, COM in **mm** relative to the selected connector, and matrix entries in **g·mm²** about that reference. Rows and columns are X, Y, Z; symmetric entries retain the displayed signs.

| Selection | Mass (g) | COM X, Y, Z (mm) | Volume (mm³) | Surface (mm²) |
| --- | ---: | --- | ---: | ---: |
| Scrolling full-body selection | 379.768 | `19.05, -8.18, -91.98` | 211213.04 | 180343.24 |
| Rim <1> + Tyre <1> | 40 | `2.93e-5, 0, -176.63` | 21401.7 | 30744.28 |
| Tyre <2> + Rim <2> | 40 | `0, 2.94e-5, -7.37` | 21401.7 | 30744.28 |

```text
I_full about connector (g·mm²):
[[4.275e+6,   67658.407,  664548.976],
 [67658.407,  4.376e+6,  -284794.478],
 [664548.976, -284794.478, 497115.056]]

I_left about connector (g·mm²):
[[1.264e+6, -0.405,      0.202],
 [-0.405,    1.264e+6,   0.063],
 [0.202,     0.063,  27124.084]]

I_right about connector (g·mm²):
[[18312.789, 0.001,     -0.005],
 [0.001,     18311.904,  0.013],
 [-0.005,    0.013,  27124.084]]
```

Convert **g → kg with `1e-3`**, **mm → m with `1e-3`**, and **g·mm² → kg·m² with `1e-9`**. The scientific-notation diagonals `4.275e+6`, `4.376e+6`, and `1.264e+6` have only four displayed significant digits; treating their last visible digit as exact would invent precision. [Onshape's mass-properties help](https://cad.onshape.com/help/Content/View/mass_properties_tool.htm) says selecting a mate connector changes the inertia reference from the selected parts' center of gravity. The huge left-wheel transverse terms also demonstrate the parallel-axis offset from the selected Rim <2> connector.

## Same-frame calculations, conditional on an unchanged connector

Using the displayed values as rounded inputs, `I_COM = I_connector - m[(d·d)I - d dᵀ]`. The **right wheel** has a 7.37 mm Z offset: its approximate COM-frame diagonal is `[16140.113, 16139.228, 27124.084] g·mm²` (`[1.6140113e-5, 1.6139228e-5, 2.7124084e-5] kg·m²`). The **left wheel** has a 176.63 mm Z offset: its approximate COM-frame diagonal is `[16073.724, 16073.724, 27124.084] g·mm²`. Subtraction of `40 × 176.63² = 1,247,926.276 g·mm²` from a displayed `1.264e+6` makes the left transverse values particularly sensitive to rounding. Neither screenshot matrix may be copied directly into a wheel COM inertial block.

If the full selection and both wheels really share one unchanged connector frame, subtracting the two **connector-origin** wheel tensors from the full tensor yields a screenshot-derived chassis mass of **299.768 g** and COM `[24.133928, -10.363025, -91.974663] mm` in that connector frame. Shifting the remainder to its own COM gives this approximate chassis tensor in **kg·m²**:

```text
[[ 4.246554388e-4, -7.313317730e-6, -8.492041072e-7],
 [-7.313317730e-6,  3.832502860e-4,  9.250493629e-7],
 [-8.492041072e-7,  9.250493629e-7,  2.360753022e-4]]
```

Its principal values are positive (`2.36066e-4`, `3.82001e-4`, `4.25914e-4 kg·m²`), but **this is the chassis implied by the 379.768 g screenshot selection**, not the 259.033080 g chassis in the chosen local audit. It is not a tensor for the chosen mass model. The displayed precision and missing connector-to-URDF transform also prevent safe placement in a MuJoCo local body frame.

## Mass and source reconciliation

The current [mass audit](models/wall_e_mass_audit.json) selects **339.033080 g** total and **259.033080 g** chassis, with two separate 40 g wheels. Therefore the screenshot exceeds both corresponding local totals by **40.734920 g**. The local battery/holder/springs group is 153 g, four screws are 4.5 g each, and the populated PCB is 30 g once; these are explicit replacements, not masses to add again. The right/left wheel screenshots agree with the two 40 g groups. The screenshot does not reveal the full selected-instance list or per-part mass assignments, so the residual cannot be uniquely assigned.

There is a strong **unconfirmed** lead in three structural export links: `part_1`, `part_1_1`, and `part_2` have watertight mesh volumes totaling **33.310610 cm³**, but source/audit masses totaling only **0.0333142 g** (implied density about **1 kg/m³**). If *only* these three parts explained the gap, their combined mass would need to be **40.768234 g**, equivalent to one effective density of **1.223881 g/cm³**. This exactly closes the mass arithmetic and is geometrically plausible, but their Onshape material/mass settings are not visible. It is **not** an authorized or verified mass correction. The two wheel COMs are 169.254 mm apart in the local audit and 169.260 mm apart in the screenshots, which supports comparable geometry but does not resolve the mass source.

At the time of this audit, the source was `Assembly 1/assembly_1/urdf/assembly_1.urdf`, with the **same SHA-256** as the audit's URDF (`2b36828c3e11cd8cc90e3f3cd164ebaa30394eceb374e10859ab6bfdb632a0ef`); all 396 same-named mesh files existed there, and 47 mesh hashes recorded for corrected groups matched. The export bundle has since moved, unchanged, to `Wall-E-URDF/`; the converter and generated MuJoCo models were regenerated with the new mesh paths, resolving the stale-path condition described at the time. This historical audit still does not validate the physical inertia model.

The current MJCF has **three fixed bodies, no joints, no `<inertial>` blocks, and no active collision geoms**. Its 556 `custom/numeric` entries carry mass/COM audit metadata, including all 545 links; they do **not** set `MjModel.body_mass` or body inertia. The checker floor is visual-only. No floating base, wheel hinges, contacts, actuators, or dynamic validation currently exists.

## Decision and precise missing readings

Per the user's direction, keep the **339.033080 g local audit** as the chosen mass ledger; do not scale the 379.768 g tensor to it. A tensor cannot be corrected by a mass ratio without knowing the differing solids and their locations. No dynamic MJCF is created from the inconsistent screenshot data.

To revisit inertials, capture (1) the actual Onshape mass/material or override for each of the three `Part_*` structural components and confirm the full selection list/configuration; (2) full-precision copied COM and all inertia entries, especially full-body and left-wheel `Lxx`/`Lyy`; and (3) the Rim <2> mate connector's full position **and orientation relative to the corrected URDF root**, or a direct CAD export of mass/COM/inertia in that URDF frame. A physical complete-robot mass and COM measurement remains required before calling a dynamic model hardware-validated.
