# Assumed N20 motor model and PID sweep

**Date:** 13 October 2026
**Scope:** simulation-only experiment using the user-provided motor specification image.

## Motor model used

The separate config `configs/mujoco_balance_n20_assumption.yaml` uses an effective output-shaft DC motor approximation at 11.8 V. The model infers winding resistance from 12 V / 0.75 A stall current, torque constant from 1.8 kg-cm stall torque divided by the 0.75 A–0.06 A current span, back-EMF from the 200 RPM no-load endpoint after accounting for no-load current, and Coulomb loss from 0.06 A no-load current. Gear ratio and efficiency are represented as 1 because the inferred electrical constants and torque are already expressed at the geared output shaft.

| Quantity | Value used | Basis |
| --- | ---: | --- |
| Supply voltage | 11.8 V | User's assumed operating voltage |
| Winding resistance | 16.0 Ω | 12 V / 0.75 A stall current |
| Effective torque constant | 0.25583 N·m/A | 1.8 kg-cm / (0.75 A − 0.06 A) |
| Effective back-EMF constant | 0.52712 V/(rad/s) | (12 V − R × 0.06 A) / 200 RPM |
| No-load loss current | 0.06 A | Supplier table; modeled as Coulomb friction |
| Torque bound | 0.17652 N·m | 1.8 kg-cm stall torque at 12 V |
| PWM dead zone | 0 | Not supplied; idealized assumption |

At the model's 80% maximum PWM, standstill torque is about 0.136 N·m per wheel and the zero-load speed is about 153 RPM. These are model calculations, not measured wheel behavior. The supplied table lists both rated speed and no-load speed as 200 RPM, which cannot describe a loaded operating point for a conventional linear DC torque-speed curve. The stated rated torque (0.52 kg-cm) is retained in the config as source data, not applied as a hard torque limit; the endpoint fit predicts about 142 RPM at that torque.

## PID search

The reset reference and setpoint were fixed at 0°. The firmware-compatible controller used the 60–80% PWM clamp. The finite grid covered 52,521 combinations: `Kp` 0–30 by 0.5, `Ki` 0–1 by 0.05, and `Kd` 0–20 by 0.5. Each candidate received a maximum 2-second trial; the top 30 were rerun for up to 10 seconds. A fall was total chassis tilt reaching 45°.

**Result: no candidate balanced.** Every candidate fell before 2 seconds. The top-ranked candidate was `Kp=26.5`, `Ki=0.35`, `Kd=18.5`; it fell at 0.35 seconds in both the screen and the longer validation. Its peak absolute pitch was 47.38°, pitch span 50.42°, mean absolute PWM 61.22%, minimum contact count zero, and final wheel speed about 14.15 rad/s (135 RPM). These gains are the best-ranked failed candidate, not recommended settings.

The result says the current simulation setup did not balance under this finite grid; it does not say the real robot cannot balance. The hinge signs remain assumptions (`left=+1`, `right=−1`), the spec-derived motor curve is not characterized on the actual robot, and the CAD-based inertias/contact/reset behavior are provisional. The saved trace shows rapid sensor/controller/motor reversals and loss of wheel-floor contacts after the fall begins. Do not transfer the gains or inferred constants to hardware.

## Reproducibility

```bash
.venv/bin/python -m simulation.diagnostics.sweep_balance_pid_grid \
  --config configs/mujoco_balance_n20_assumption.yaml \
  --kp-range 0 30 0.5 --ki-range 0 1 0.05 --kd-range 0 20 0.5 \
  --trial-seconds 2 --validation-seconds 10 --validation-top 30 \
  --setpoint-deg 0 --pwm-output-mode firmware_clamped \
  --output-dir experiments/mujoco_pid_grid/20261013_n20_assumption_full_firmware_grid
```

All grid rows, validation results, metadata, summary, and best-candidate telemetry are saved in [`experiments/mujoco_pid_grid/20261013_n20_assumption_full_firmware_grid/`](../experiments/mujoco_pid_grid/20261013_n20_assumption_full_firmware_grid/). Software checks passed: 15 tests plus 6 subtests, and `simulation.verify_balance_io`. These checks validate software plumbing, not physical accuracy.
