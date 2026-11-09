# Assignment 3: five-state EKF prototype and stationary preview

This package implements and tests an EKF with state `[x, y, theta, v, omega]`. It contains an offline replay and a stationary ROS preview. Neither script creates a motor-command publisher or an SDK connection. It does not contain the final feedback controller and must not be used as an evaluation runner.

## First step: offline replay in WSL

Leave robot nodes stopped. Extract into `~/assignment3_tools`, then:

```bash
cd ~/assignment3_tools/assignment3_ekf_preview
~/venvs/rm_flatness/bin/python -m unittest -v test_ekf.py
~/venvs/rm_flatness/bin/python replay_ekf.py \
  --calibration-dir ~/assignment3_tools/assignment3_calibration \
  --output replay_wsl01
```

The only offline dependency is NumPy, already used by the robot environment. Do not prefix these commands with the isolated OpenCV 5 PYTHONPATH; only the tag observer needs that directory. No SciPy or OpenCV is used by the EKF. Use a new output directory on repeat, e.g. `replay_wsl02`; existing results are never overwritten.

The calibration directory is the previously supplied package. Its physical sidecar measurements are authoritative for this replay; original empty final-gap metadata is not interpreted as a missing physical measurement. `verified_replay_final` contains the supplied replay outputs and plot. Each result is an in-sample calibration replay, not evidence that an EKF beats another controller on independent data.

The seven numerical tests check the nonlinear Jacobian against central differences, coordinate rotation, covariance symmetry/positive semidefiniteness, stale/duplicate rejection, R weighting, time reversal/nonfinite input handling, and a synthetic straight-motion case. These tests do not establish live ROS or physical safety performance.

Optional plotting requires matplotlib in the chosen interpreter:

```bash
~/venvs/rm_flatness/bin/python plot_replay.py replay_wsl01
```

## Second step: stationary live preview

Do this only after inspecting offline output. Keep the chassis stationary and the path clear. Driver startup moves the arm to its configured `(105,35)` mm pose. Do not manually reposition with the driver running.

Because observed clock steps caused prior aborts, temporarily stop `systemd-timesyncd` before starting robot nodes for the preview; this does not guarantee WSL host time cannot step. Restore it after stopping robot nodes at session end. Never increase freshness limits to hide a jump.

Driver terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rm_flatness_ws/install/setup.bash
~/venvs/rm_flatness/bin/python ~/rm_flatness_ws/install/rm_flatness_nav/lib/rm_flatness_nav/driver
```

Observer terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rm_flatness_ws/install/setup.bash
PYTHONPATH="$HOME/rm_tag_cv5_test${PYTHONPATH:+:$PYTHONPATH}" \
~/venvs/rm_flatness/bin/python /mnt/c/Users/khanm/Downloads/tag_observer.py
```

Free WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rm_flatness_ws/install/setup.bash
cd ~/assignment3_tools/assignment3_ekf_preview
~/venvs/rm_flatness/bin/python live_preview.py --run-id ekf_preview01 --duration 20
```

The preview requires zero `/rm/cmd_vel` publishers, zero applied commands, fresh ToF/tag/odometry/driver messages, tag 3 centered within the same +/-0.15 normalized image limit, range above 0.30 m and READY or pre-command COMMAND_TIMEOUT. It initializes after one healthy second. It reports `HEALTHY; x=... m` while stationary; a small nonzero x reflects sensor noise/model assumptions, not robot motion ground truth.

The only publisher is `/rm/ekf_preview` (`std_msgs/String`). Diagnostic output contains the state `[x_m,y_m,theta_rad,v_m_s,omega_rad_s]`, covariance diagonal and accepted observation counts. Raw input values, status and filter states are retained in `live_previews/<run-id>/events.csv`; `summary.json` records the last status, initialisation and any detected clock jump. A detected clock step over 0.05 s blocks the preview until restarted. No automatic origin reset hides a discontinuity. Ctrl+C stops the preview; it has no motion commands to cancel. Hardware execution has not been tested here.

## Model, calibration and limits

See `MODEL.md` for the equations and explicit Q/R assumptions. The fitted odometry bias is +0.006774102 m; ToF travel scale is 0.897081413. Corrections affect observations, not the independent raw-ToF safety guard. We use odometry pose, not raw SDK twist as a second sensor: slow05's raw SDK body-vx was predominantly negative despite forward pose progress.

Full Q/R are not uniquely identified by five endpoint measurements. Endpoint residual sample variance is a tuning proxy, inflated by four. Several response/diffusion/initialization settings are declared priors. Update frequency is limited to 4 Hz per sensor; temporal and cross-sensor dependence can remain. Initial-range variance is estimated from the prestart range spread and conservatively floored at a 2 mm SD, without claiming repeated snapshots are independent. The reported covariance can still be overconfident.

All five calibration pilots stay in the primary replay, including slow05's brief driver stop and fast07's pre-motion clock jump. This prototype applies accepted delayed measurements at receipt without rewinding the state; remaining latency error is a limitation. Callback stamps are not hardware acquisition stamps. The angular state is predicted from applied yaw commands and indirectly constrained by position observations; it is not independently validated by a heading ground truth.

The constant endpoint bias is smoothly introduced over the first 2 s to avoid starting the observation at a negative offset. This is a documented modeling choice, not a measured time law. At later pilot endpoints the full bias is applied. That choice and the provisional noise settings must be reviewed and frozen before evaluation.

## What comes after this preview

Implement identical cubic-reference feedback for unfiltered and filtered modes, preserve the independent driver guards and callback-latched stop behavior, verify the near-box conservative raw-ToF threshold physically, then freeze configuration. Only then collect alternating evaluation runs (at least five valid per method), retaining aborts and physical final gaps. The current pilots stop far from the final 0.20 m goal; they do not validate the final stopping clearance. Do not score calibration replays as evaluation.
