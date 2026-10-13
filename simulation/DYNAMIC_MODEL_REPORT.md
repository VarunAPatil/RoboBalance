# Provisional Wall-E MuJoCo dynamics inspection

The separate [dynamic MJCF](models/wall_e_dynamic.xml) has a floating chassis and two hinged wheels. It is derived from the corrected `Assembly 1/assembly_1/urdf/assembly_1.urdf`, the current mass ledger, and the three Onshape screenshots. It is **CAD/screenshot-derived, not hardware-validated**. The source URDF/STLs and the geometry-only preview remain unchanged. [The machine-readable audit](models/wall_e_dynamic_audit.json) records every full-precision calculation and source hash; regenerate with `.venv/bin/python -m simulation.build_dynamic` and check with `.venv/bin/python -m simulation.verify_dynamic`.

## Chosen mass scenario

`pcb_reconstructed` is the scenario paired with the screenshots. Its ledger total is **0.379767823293 kg**, versus **0.379768 kg** displayed by Onshape, a difference of **0.000000176707 kg (0.000177 g)**. The MJCF assigns **0.299767823293 kg** to the chassis and **0.040 kg** to each wheel. `pcb_30g` remains an alternative in the ledger at **0.379839766 kg**; it is not used by this model. The structural link masses stay at `part_1` **0.032 kg**, `part_1_1` **0.00442 kg**, and `part_2` **0.00442 kg**, with their mesh-derived COMs. The 0.153 kg complete battery/holder/spring group and four 0.0045 kg screws remain in the chassis ledger. The screenshot tensor is **not scaled** to absorb the 0.000177 g mass difference. Screenshot masses are used in all screenshot parallel-axis shifts; the corrected ledger masses are assigned to the MuJoCo bodies.

## Screenshot data and reference

All three images display `Mate connector 1 of Rim <2>` as the mass-property reference. The following are the values **as displayed**, in X/Y/Z order. Inertia entries include off-diagonal signs and are about that connector origin, not about each group's own COM.

| Selection | Mass (g) | COM in connector (mm) | Inertia at connector (g·mm²): `[Lxx,Lyy,Lzz,Lxy,Lxz,Lyz]` |
| --- | ---: | --- | --- |
| Full selection | 379.768 | `[19.05, -8.18, -91.98]` | `[4.275e6, 4.376e6, 497115.056, 67658.407, 664548.976, -284794.478]` |
| Rim <1> + Tyre <1> | 40 | `[2.93e-5, 0, -176.63]` | `[1.264e6, 1.264e6, 27124.084, -0.405, 0.202, 0.063]` |
| Tyre <2> + Rim <2> | 40 | `[0, 2.94e-5, -7.37]` | `[18312.789, 18311.904, 27124.084, 0.001, -0.005, 0.013]` |

The screenshots do not show the mate connector pose directly. We inferred its **complete** pose using three corresponding points: both wheel COMs and the assembly COM, each available in the screenshot connector frame and in the corrected URDF mass audit. A proper rigid Kabsch fit gives the connector origin in the source URDF root frame as approximately **`[0.209816903, -0.079089414, -0.011870400] m`** and the connector-to-source rotation matrix:

```text
[[ 0.000003494,  0.000001511,  1.000000000],
 [ 0.161177294, -0.986925468,  0.000000928],
 [ 0.986925468,  0.161177294, -0.000003692]]
```

The largest COM fit residual is **0.003895 mm**. The three pairwise distance errors are all below **0.006 mm**, matching the screenshots' two-decimal millimetre display precision. Thus the pose is a strong, *inferred* mapping from existing evidence; it is not a direct Onshape connector-pose export. The screenshot's full selected-instance list is scrollable, so identical instance selection/configuration is supported by mass and geometric consistency but not directly visible in full. A direct CAD pose export would remove this residual uncertainty. The hinge frames and axes are taken from the active URDF tree, **not from the screenshot camera axes**.

## Tensor construction and coordinates

For each component with connector-frame COM displacement `d` in **m** and mass `m` in **kg**, we calculate `I_COM = I_connector − m[(d·d)I − ddᵀ]`. The chassis connector-origin tensor is `I_full − I_left − I_right`. Its COM is `(m_full c_full − m_left c_left − m_right c_right)/(m_full − m_left − m_right)`, using the screenshot masses. We then apply the same parallel-axis shift to the chassis. Screenshot **g·mm²** values are multiplied by **1e-9** to get **kg·m²**; **g** by **1e-3** to get **kg**; **mm** by **1e-3** to get **m**. Off-diagonal signs are retained.

Each COM tensor is rotated into the source URDF root axes with `I_source = R_connector_to_source I_connector_COM R_connector_to_sourceᵀ`. For a wheel, `I_body = R_wheel_to_sourceᵀ I_source R_wheel_to_source`; its COM is expressed relative to the audited wheel hinge frame. The chassis body axes match the source URDF axes. The mapped screenshot COMs differ from the ledger COMs by at most **0.003895 mm**; MJCF inertial positions use the ledger COMs. This difference is far below the screenshot COM display precision, but it remains part of the approximation.

The resulting **local-body** properties written to `<inertial>` are shown below. Tensor rows/columns are X/Y/Z in that body frame; all entries are in **kg·m²**. The `fullinertia` order in XML is `Ixx Iyy Izz Ixy Ixz Iyz`. MuJoCo diagonalizes these tensors internally. The input inertial frame is aligned with each body (identity quaternion `w,x,y,z = [1,0,0,0]`); numerical principal-axis rotations are in the JSON audit. Wheel transverse principal axes are sensitive to rounding because the two transverse moments are nearly equal.

| Body | COM local (m) | Approximate local COM tensor (kg·m²) | Principal moments (kg·m²) |
| --- | --- | --- | --- |
| Chassis | `[0.117844854, -0.064972038, 0.010278179]` | `[[2.360753e-4,-1.049917e-6,-6.883331e-7],[-1.049917e-6,3.866526e-4,1.351967e-5],[-6.883331e-7,1.351967e-5,4.212532e-4]]` | `[2.360661e-4,3.820006e-4,4.259144e-4]` |
| Left wheel | `[0.000000030,-0.000000004,0.018127169]` | `[[1.607360e-5,3.839569e-10,6.263057e-12],[3.839569e-10,1.607385e-5,1.339129e-10],[6.263057e-12,1.339129e-10,2.712408e-5]]` | `[1.607332e-5,1.607413e-5,2.712408e-5]` |
| Right wheel | `[-0.000000029,-0.000000009,0.018127169]` | `[[1.614009e-5,-1.398319e-10,1.350510e-11],[-1.398319e-10,1.613925e-5,2.404217e-11],[1.350510e-11,2.404217e-11,2.712408e-5]]` | `[1.613923e-5,1.614011e-5,2.712408e-5]` |

These printed decimals are calculation output, **not additional screenshot precision**. The full-body and left-wheel transverse diagonal screenshot entries have only four significant digits. Their rounding uncertainty propagates into the chassis and left-wheel tensors; the physical triangle inequalities and positive principal moments still hold for the displayed values. Independently integrated closed rim/tire STL groups have principal moments within about **0.25%** of the corresponding screenshot-derived values, which supports the assignment but does not make the screenshots or physical robot exact.

## MJCF structure and verification

The dynamic XML has one free chassis joint and two wheel hinges (three joints, eight velocity DOFs), with 404 original coloured visual mesh instances. Their meshes remain non-colliding. The wheel hinges keep the corrected URDF locations and child-frame axes (`[0,0,-1]`); in the initial source frame the left axle is approximately `+X` and the right `−X`. Gravity is `[0,0,-9.81] m/s²`. The initial display translation is `[-0.117815971,0.045935139,0.046319733] m`. The collision proxies initially overlap the floor by approximately 0.01 mm so MuJoCo has tire-floor contacts at time zero instead of a 0.5 mm free-fall gap. This is a tiny numerical contact preload, not measured tire deformation. The floor plane and two transparent tire cylinders are the only active collision geoms. Each cylinder has radius **0.03446 m** and half-width **0.01245 m**. Contact friction `1 0.005 0.0001` is provisional and unmeasured. There is no chassis collision proxy, motor, actuator, sensor, RL code, or safety logic in the base dynamics XML.

The static preview's `custom/numeric` mass annotations describe its selected `pcb_30g` view and are deliberately omitted from the new dynamic XML. The dynamic JSON audit and physical `<inertial>` tags describe `pcb_reconstructed` instead.

Verified with project `.venv` MuJoCo **3.7.0**: compilation succeeds with no invalid-inertia warning; compiled mass totals **0.379767823293 kg**, body masses and COMs match the ledger, and compiled principal inertias/full tensors match the intended tensors within MuJoCo's internal diagonalization precision. `mj_forward` produces finite state. A short unpowered gravity/contact check produces tire-floor contacts and finite state; it is a numerical plumbing check, not a fall/balance or hardware validation. Changing either hinge angle moves only that wheel. The dynamic model's 404 initial visual geoms match the static preview in orientation and relative placement, with only the documented global Z shift. Three-quarter and battery-side offscreen renders were inspected. The existing static preview and mass audit verification remain separate.

Open the **paused inspection viewer** (gravity enabled in the model, but the launcher does not advance the simulation):

```bash
.venv/bin/python -m simulation.preview --model simulation/models/wall_e_dynamic.xml
```

Physical assembled mass/COM, the exact Onshape connector pose, higher-precision screenshots, contact/friction, motor response, and all chassis impact geometry remain unvalidated. This model must not be used as evidence that the robot can balance or as a deployable policy environment; the TT motors still provide no wheel encoder measurements.

## Sensor/actuator software scaffold — 2026-10-10

`build_balance_model.py` generates a separate `models/wall_e_balance.xml`; the existing dynamic and geometry-only models remain unchanged. The derived model adds the configured `imu_site`, MuJoCo accelerometer/gyro sensors, and direct-torque actuators for the two wheel hinges. The user identified `Board_001` as the IMU body and supplied its center location and axis directions; `configs/mujoco_balance.yaml` maps those to the chassis frame. The dynamic builder now lowers the initial chassis pose by an additional 0.51 mm so both wheel collision proxies have about 0.01 mm initial overlap with the floor. `verify_dynamic.py` checks the initial contact count and each tire-floor distance. This prevents a free-fall gap at startup, but does not make the unpowered robot stable or validate physical tire deformation. A run still requires explicit left/right actuator signs.

`imu_model.py` reproduces the firmware's accelerometer pitch equation and complementary-filter pattern, with optional explicitly configured biases/noise and neutral-angle calibration. `balance_controller.py` separates the firmware-compatible per-sample PID from a time-corrected variant. The firmware implementation uses integer gyro division, a default 9-degree setpoint, per-sample error differencing/summing, a ±1-degree deadband, and 60–80% PWM clamping; these are software behaviors, not validated hardware tuning. `motor_model.py` provides an explicitly artificial ideal-torque mode for actuator plumbing and a physical DC motor interface that requires characterized parameters. It does not infer motor physics from motor mass.

The IMU layer has an optional empirical fused-output startup replay. It was updated from the second three-run hardware capture set at `experiments/hardware_imu/repeat_20261012T132500Z/`: +13.86° roll and −1.50° pitch initial offsets, decaying with a 9.2 s time constant toward late means of −13.70° roll and −0.31° pitch. The first capture set instead settled near −20.25° / +6.19°, so the physical orientation was likely different between sessions. The replay is **disabled by default** because neither capture pose is a measured balancing reference; set `startup_transient_enabled: true` only to reproduce the latest capture's approximate fixed-pose curve. It is separate from raw accel/gyro bias and complementary-filter state because raw channels were not logged and the cause remains unknown. The gravity logger and balance telemetry record roll as well as pitch.

`.venv/bin/python -m simulation.verify_balance_io` passed using a synthetic identity sensor pose and artificial torque scale. This verifies XML compilation, sensor/actuator names and dimensions, pitch sign/filter calculations, missing-parameter guards, and torque transmission only. It is not physical model validation or a balance result. Use `simulation/README.md` for build/run commands and remaining measurement gates.

## Reset-pose estimator seed and live tuning effort — 2026-10-13

The last available live run showed chassis debug pitch about `0.05°` on its first sample but accelerometer pitch near `39.73°`; the configured MPU site acceleration was approximately `[-6.75, 0.00, 8.12] m/s²`. At the reset pose, the model is already accelerating because of its CAD-derived mass distribution and tire contacts, so using that first full accelerometer vector to initialize the complementary filter mislabeled dynamic acceleration as tilt. `run_balance.py` now initializes/reinitializes the fused estimate using gravity projected through the configured IMU site's orientation at the MuJoCo `qpos0` reset pose, then applies the normal filter to subsequent full accelerometer/gyro samples. Both the raw accelerometer angle and the seeded fused estimate remain in telemetry (`pitch_accel_deg`, `pitch_estimate_deg`, and `imu_seed_pitch_deg`). This is a simulation reset convention analogous to placing/holding the model at its known pose; it is not claimed to emulate the unknown MPU startup behavior or to correct the model's underlying COM/contact dynamics.

`BalanceController` now has a separate `pwm_output_mode`. Normal runs retain `firmware_clamped`: outside ±1° they apply the firmware-style 60–80% magnitude range. `--tune` defaults to `continuous`, which preserves the firmware pitch-error direction rule and upper limit but removes the 60% lower clamp, allowing PID changes to request 0–80% magnitude. Select `--pwm-output-mode firmware_clamped` for a side-by-side software comparison. This continuous mode is intentionally not an exact firmware reproduction; pairing it with `ideal_torque_test_only` remains an interface experiment, not measured motor simulation. The reset seed and both output modes are covered by the focused tests and `simulation.verify_balance_io`.
