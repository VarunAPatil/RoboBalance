# Hardware status, 2026-09-25

User-confirmed: two L-shaped TT motors, no wheel encoders, and an MPU6050 IMU. The assembled robot uses the 2027 local board design.

The user confirmed `/home/varun/Wall-E-2027-Board-Design/sra_board.kicad_sch` is the assembled board design and that the installed IMU is an MPU6050. The schematic's placed symbols include ESP32-S3-WROOM-1, a TB6612FNG motor-driver module, and an MPU-6050 connector. The motor-driver module's physical marking still needs checking. The older linked [SRA 2026 board](https://github.com/SRA-VJTI/sra-board-hardware-design) uses ESP32-WROOM-32E according to its [BOM](https://github.com/SRA-VJTI/sra-board-hardware-design/blob/master/sra_dev_board_2026/bom/sra_dev_board_2026.csv); do not use its pinout for this robot.

The user confirms the MPU6050 is centered on the `Board_001` body and its X axis points backward, while its Y axis is parallel to the wheel axle and points to the robot's right. The balance MJCF places the IMU site at the compiled `Board_001` mesh center and maps these axes to chassis `-Y`, `+X`, and `+Z` respectively (assuming chassis `+X` points right and `+Y` forward). This is a user-provided mounting description plus a CAD-frame mapping; confirm the chassis forward/right convention and pitch signs with a small simulated tilt before PID tuning. The IMU position is a center approximation, not a measured chip-datum coordinate.

Still needed before the robot simulator: MPU6050 interface/timing, measured motor gear ratio or response, battery type and measured voltage range, robot and wheel masses, wheel radius, axle spacing, center-of-mass height, and intended strategy for wheel motion feedback. Before deployment also confirm the motor wiring/signs, line sensor type, host OS and ESP-IDF setup.

## Component weights provided 2026-10-09

The user supplied the following per-component weights; measurement method and uncertainty were not specified. The editable [component weight sheet](COMPONENT_WEIGHTS.csv) calculates totals as quantities are filled in.

| Component | Quantity used in sheet | Unit weight | Listed weight |
| --- | ---: | ---: | ---: |
| Wheel | 2 | 40 g | 80 g |
| Motor | 2 | 29 g | 58 g |
| M3 screw | 4 CAD instances; installed count not independently checked | 4.5 g | 18 g for those instances |
| Populated PCB and mounted modules | 1 target group | 30 g | 30 g |
| Battery holder | 1 | 18 g | 18 g |
| Battery | 3 CAD cells; installed count not independently checked | 45 g | 135 g as a standalone sum |

The original known subtotal is **186 g**, excluding screws, batteries, and all unlisted parts. With the CAD-visible four screws and three cells, the [CSV](COMPONENT_WEIGHTS.csv) shows **339 g** of listed weights. This is not a measurement of the complete assembled robot or its center of mass. The CSV's standalone 18 g holder plus three 45 g cells happens to total 153 g; the simulation audit instead assigns **153 g to the complete cells-plus-holder-plus-six-springs group at one shared effective density**. Do not add the separate holder/cell figures again when using that grouped mass. Confirm the physical installed counts and account for any omitted parts during characterization.

## Export-based mass and center-of-mass estimate — 2026-10-10

The current [mass audit](../simulation/models/wall_e_mass_audit.json) and its detailed [derivation](../simulation/MASS_AND_COM_REPORT.md) include the user-supplied structural masses. The export-based total is about **379.8 g**, depending on the PCB scenario. This is a CAD-and-user-measurement estimate, not a weighed complete robot. The static MuJoCo scenes store the audit as metadata and COM sites; the separate dynamic model has screenshot-derived inertias. Physical mass/COM, motor response, and contact properties remain open.

## Firmware and simulation interface notes — 2026-10-10

Inspection of `/home/varun/Wall-E/6_self_balancing/main/self_balancing.c` and the SRA MPU driver confirms that firmware reads pitch from complementary-filter output index 1; pitch uses accelerometer X and gyro Y. The accelerometer formula is `atan2(-ax, sqrt(ay²+az²))`, and the gyro scale is configured for 131 counts per degree/second. The balancing task initializes its angle offset to zero and does not invoke `calibrate_mpu6050()`. The calibration helper's raw offsets are also not subtracted in the normal raw-reading path. The default controller setpoint is 9 degrees, so firmware does not target a zero-degree pitch by default.

The source gyro conversion divides integer counts by integer 131 before integration, truncating fractional degrees/second. The controller differentiates and accumulates error once per loop without scaling by measured dt; its effective gains therefore depend on loop rate. The MuJoCo scaffold makes compatibility explicit and does not characterize or fix the physical firmware. Before comparing simulation and hardware, record the actual MPU site pose, collect stationary and supported-motion IMU logs, choose/measure the upright angle reference, verify left/right wheel signs, and characterize the motors and battery voltage. Ideal torque tests are not PWM-to-motor validation.

The user recalls that the physical robot balanced with firmware setpoints in the **23–30°** range; the exact run conditions, sensor offset, motor command, and logs have not yet been recovered. A 2026-10-11 MuJoCo sweep over that range did not balance using a fixed, explicitly artificial 0.02 N·m ideal-torque scale and provisional actuator signs. This simulation mismatch does not invalidate the hardware observation; it shows the current sim sensor bias/initial transient and actuator/plant assumptions must be reconciled before transferring setpoints or PID gains.

Three live resets of the standalone MPU diagnostic firmware were logged on 2026-10-11 with the robot held in the same horizontal, non-balancing pose. Their first fused samples averaged about **−0.65° roll / −0.28° pitch**; each moved consistently toward late-capture means averaging **−20.25° roll / +6.19° pitch**. The first 10-second means were about **−7.9° / +2.1°**. The report and raw captures are in `experiments/hardware_imu/`. Because only fused angles were printed—not raw accelerometer/gyro channels—the cause is not identified. The simulation now reproduces this one-pose startup curve with a separately configurable empirical transient; this is signal-shape replay, not evidence of an MPU bias mechanism or a valid balancing-pose zero.

A second three-reset capture set was made on 2026-10-12 with the existing `mpu_test` firmware, using ESP-IDF Monitor to reset and record each run. It is saved under `experiments/hardware_imu/repeat_20261012T132500Z/`. The three runs agree reasonably within this set (0.29–0.98° pairwise RMS difference over the first 60 s), but their late means are about **−13.70° roll / −0.31° pitch**, unlike the previous set’s **−20.25° / +6.19°**. The held orientation therefore appears to have changed between sessions, or the earlier pose was not reproduced; orientation was not measured independently. In this repeat set, roll trends negative while pitch remains near −1° to −0.3° rather than rising to +6°. Do not treat the earlier startup replay constants as universal. These logs still contain fused angles only.

The MuJoCo empirical replay parameters were updated to this latest set (+13.86° roll, −1.50° pitch initial output offsets; 9.2 s decay), but remain disabled by default and pose-specific. The test verifies the approximate first reading, first-10-second mean, and settled value for this set. It is not a new IMU calibration; resolve the inter-session pose discrepancy and capture raw axes before using a transient for balancing.

## Motor specification image used for a simulation-only assumption — 2026-10-13

The user supplied a specification image for an N20-12V-200 motor: 12 V, 200 RPM rated/no-load speed, 0.52 kg-cm rated torque, 1.8 kg-cm stall torque, 0.06 A no-load current, and 0.75 A stall current. The user authorized using these values *as an experiment assumption*. The actual installed TT motor identity and loaded response have not been verified. A separate `configs/mujoco_balance_n20_assumption.yaml` derives an effective output-shaft DC curve at 11.8 V; its gear ratio, efficiency, zero dead zone and motor signs are assumptions. The table's rated-speed and no-load-speed entries are both 200 RPM, inconsistent with its listed rated torque for a conventional linear torque-speed curve. This scenario is not hardware characterization and does not change `configs/mujoco_balance.yaml`.


Stationary PPO (2026-10-14) additionally observes simulator axle displacement and
velocity. These are privileged simulation values, not MPU-derived measurements
or a validated encoderless estimator. Its stationary simulation result does not
supply the missing hardware observations or validate the motor/contact model.
