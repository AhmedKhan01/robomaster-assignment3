# Assignment 3: offline pilot calibration

This package contains the five recorded, completed pilots and an offline calibration script. It cannot connect to the robot, publish commands, or start a driver. Run it with the robot stopped. It is not the final feedback controller or EKF.

## Run in WSL

Extract the ZIP into `~/assignment3_tools`, then:

```bash
cd ~/assignment3_tools/assignment3_calibration
python3 calibrate.py --output results_wsl01
```

Only Python's standard library is required. Each output directory must be new; use `results_wsl02` for a repeat. Original pilot files are never edited. `verified_results` contains the already computed results supplied with the package.

Optional figures (requires matplotlib in the chosen interpreter):

```bash
python3 plot_calibration.py results_wsl01
```

## Pilot callback fault latch

With driver and observer stopped, install the included supervisor fix:

```bash
python3 install_pilot_supervisor_fix.py
```

It requires the exact original pilot SHA256, makes a uniquely named backup, and changes only the pilot subscriber supervision. Every newly received safety input is checked immediately. During motion a fault stops and latches the pilot even if READY arrives before the next timer tick. Before motion a fault resets the continuous healthy countdown. COMMAND_TIMEOUT is accepted only before motion; an active command timeout stops the pilot. An existing stop cannot be cleared by subsequent healthy messages. This does not alter driver thresholds, command limits or freshness limits. The installer is idempotent and never starts robot processes.

Run `python3 -m unittest -v test_calibration.py` for six ROS-free checks covering the recorded endpoint, input identity, transient fault followed by READY, missing tag, active command timeout and prestart reset behavior. These are software tests, not a hardware certification.

## Inputs and provenance

The five CSVs and `metadata_original.json` files are the original uploads. Their empty final-gap metadata fields are preserved. `physical_measurements.json` separately records the user's confirmed physical measurements: all start at 1.200 m; final gaps are fast07 0.810 m, slow05 0.820 m, medium01 0.803 m, short01 0.908 m and long01 0.700 m. Tape division is 0.001 m, which is resolution rather than guaranteed accuracy. The result manifest records SHA256 hashes of every input.

These were timed characterization segments with a 0.30 m pilot raw-ToF guard, not the final 0.20 m approach. They must not be scored as evaluation runs. Earlier `slow02`, with a different camera setup, is not included in this set. Retain the original unsuccessful runs separately in the submission archive; this bundle contains only the five completed pilots used here.

## Calculations

Body +X is forward. Odometry SDK coordinates are transformed by the driver, then displacement is projected onto each run's starting heading. The physical travel is measured initial front-edge-to-box-face gap minus settled final gap. Heading-projected odometry and front-clearance reduction can differ if the robot turns or moves sideways; these pilots have small steering corrections, not a verified continuously straight physical path.

The settled estimate is the median of the last ten odometry samples at least one second after STOP. The final range uses the same endpoint policy on the last ten ToF samples. The initial pose and initial range are the snapshots logged at MOTION_START. This reproduces the endpoint policy used throughout the analysis.

For each run, `e_o = x_odom_settled - physical_travel` and `delta_r = r0 - rf`. We calculate `b_o = median(e_o)` and `s_T = median(physical_travel / delta_r)`. All five pilots remain in the primary estimate. The JSON also shows a sensitivity estimate excluding the interrupted slow05, without replacing the primary result.

Intended observation corrections are `z_o = x_raw - b_o` and `z_T = s_T * (r0 - r)`. The additive odometry correction is a fitted endpoint approximation. It can produce a small negative observation near startup; do not silently clip or reinterpret it as a scale calibration. The final controller must document its initialization treatment before freezing calibration.

`speed_comparison` reports requested speed, a least-squares slope of odometry progress after excluding the first 0.5 s and last 0.2 s, and raw SDK body-vx summaries from unique velocity sequences. The physical endpoint-average speed includes acceleration and stopping effects. Odometry pose slope and SDK twist are not independent sensors. Raw body-vx sign/noise must be assessed before using twist.

The aligned sensor CSV interpolates odometry at the estimated ToF callback times; it rejects odometry gaps over 0.2 s and invalid source ages. Driver stamps are callback receipt times, not hardware acquisition times. ToF samples and plotted points may be correlated or repeated; their plotted counts are observation counts, not independent sample counts. In particular all travel observations share the same initial range error.

## Retained exceptions

- fast07 has an approximately 0.827 s clock jump before motion; the movement window completed. Do not interpret the whole log as one uninterrupted wall-clock timeline.
- slow05 has one driver `ODOMETRY_INVALID_OR_STALE` sample during motion with zero applied command, followed about 24 ms later by READY. Its pilot supervisor still reported the timed segment completed. This is retained and flagged, not described as uninterrupted constant speed.
- Later runs used temporarily stopped systemd-timesyncd to avoid observed clock steps. Restore that service after stopping robot nodes at the end of the session. Do not weaken freshness guards.

## What remains before evaluation

1. Validate the progress and velocity channels, review sensitivity and choose documented EKF noise assumptions. The endpoint residual sample spreads are calibration evidence, not uniquely identified per-sample R or process Q. Physical trajectories are not observed between endpoints.
2. Implement and test the five-state EKF and identical guarded cubic-reference feedback for unfiltered and filtered modes. No EKF code is included in this calibration package.
3. Install and stationary-preview the callback fault latch supplied here; ensure the final evaluation supervisor also retains this behavior. The original pilot can miss a transient between timer checks. Retain slow05's documented interruption in the calibration record.
4. Check the conservative raw-ToF guard near the actual 0.20 m physical goal. These pilots alone do not calibrate the final approach range.
5. Freeze calibration, controller, guard and measurement protocol before collecting at least five valid runs per evaluation method. Keep aborts and label retries. Use the physical settled final gaps to score results, never a sensor agreement plot as continuous ground truth.

`evaluation_ready` remains false in the generated JSON for these reasons.
