# Wall-E MuJoCo PID grid sweep

**Date:** 13 October 2026
**Result:** No tested PID combination balanced the simulated robot.

## What was tested

An exhaustive *finite grid* of 52,521 combinations, with the balance reference fixed at 0° to match the configured MuJoCo reset-pose zero:

- `Kp`: 0 to 30 in steps of 0.5 (61 values)
- `Ki`: 0 to 1 in steps of 0.05 (21 values)
- `Kd`: 0 to 20 in steps of 0.5 (41 values)
- Each initial trial ran for at most 2 seconds; the 30 highest-ranked candidates were then checked for up to 10 seconds.
- The controller used the firmware-compatible PWM clamp. A run was counted as a fall when total chassis tilt reached 45°.

“Exhaustive” applies only to these grid points. PID gains are continuous values, so this is not a search of every mathematically possible gain or setpoint. The setpoint was held at 0° rather than swept.

## Outcome

All 52,521 candidates fell during the initial trials. The highest-ranked point was `Kp=19.5`, `Ki=0.75`, `Kd=20.0`; it survived only 0.20 s of its 2 s trial and also fell after 0.20 s in the 10 s validation. It reached 48.91° peak absolute pitch, with a 48.65° pitch span and 23.59° RMS true pitch error. Its mean absolute PWM command was 71%.

This is only the least-bad point under the sweep’s ranking rule, not a recommended or stable PID setting. Since even the top candidate fails almost immediately, further gain optimization against this unchanged model is not a useful tuning result.

## Interpretation and limits

The sweep used MuJoCo 3.7.0, `simulation/models/wall_e_balance.xml`, and the reset-pose zero and 0° setpoint in `configs/mujoco_balance.yaml`. It used firmware-clamped output, a fall threshold of 45°, and an explicitly selected ideal torque scale of 0.02 N·m at 100% command. The wheel signs were explicitly set to left `+1`, right `−1` for this software experiment; those signs are not a physical motor-direction verification.

The motor is an artificial constant-torque fixture, not an identified TT motor model. Under the configured 60% minimum PWM clamp, nonzero commands can jump to at least 60% effort; the 71% mean effort for the top run is consistent with the controller spending much of the trial near that clamped output. The sweep therefore does not establish whether PID gains can balance the physical robot, and the top gains must not be flashed to hardware based on this result.

The evidence says the *current simulated plant/controller combination* is not balanceable under the tested setup. Likely next work is to inspect the short telemetry trace and controller output/sign path, then validate the wheel torque direction and replace the ideal torque fixture with measured motor response and verified contact/dynamics. Only after the plant responds in the restoring direction should a narrower gain search be meaningful. Keep hardware trials separately guarded and low-risk.

## Reproducibility

Run command:

```bash
.venv/bin/python -m simulation.diagnostics.sweep_balance_pid_grid \
  --kp-range 0 30 0.5 --ki-range 0 1 0.05 --kd-range 0 20 0.5 \
  --trial-seconds 2 --validation-seconds 10 --validation-top 30 \
  --setpoint-deg 0 --pwm-output-mode firmware_clamped \
  --ideal-test-torque-nm 0.02 --left-sign 1 --right-sign -1 \
  --output-dir experiments/mujoco_pid_grid/20261013T_full_firmware_grid
```

The complete candidate table, validation table, metadata, top-candidate telemetry, and selected-candidate summary are in [`experiments/mujoco_pid_grid/20261013T_full_firmware_grid/`](../experiments/mujoco_pid_grid/20261013T_full_firmware_grid/). The code checks passed after the run: 14 tests plus 6 subtests, and `simulation.verify_balance_io`.
