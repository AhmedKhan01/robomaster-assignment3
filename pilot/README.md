# Assignment 3: first timed motion characterization

This is a low-speed, bounded pilot to check odometry against commanded speed.
It is not the final cubic-reference controller, final EKF, or evaluation protocol.
No calibration or performance result is fabricated. Upright T3 and normalized
image centering are a change from the Phase 2 floor-tag bearing setup; describe
this accurately and resolve course acceptance before final evaluation.

## Setup

Use ROS 2 Jazzy and the existing ~/venvs/rm_flatness Python. Run the updated driver
and tag_observer.py. Stop the old planner and all other /rm/cmd_vel publishers.
The new pilot is the only command publisher during --execute. It uses no SDK
connection, changes no existing package, and needs no colcon rebuild.

The user's two preliminary static checks were physical 0.30 m -> raw 0.303 m and
physical 0.20 m -> raw 0.196 m, with T3 visible at 0.20 m. These are single readings,
not a complete noise/scale calibration. This pilot stops on raw ToF <=0.30 m,
missing/stale ToF/odometry/tag/driver health, excessive image-centering error,
multiple command publishers or bounded runtime. The driver's independent raw ToF
0.20 m latch and motor-command timeout remain active. Moving the box close during
static checks can latch the driver: restore the setup and restart the driver.

Mark the start and box positions. Keep camera mounting and upright tag height
fixed. Restore the box to a physically measured 1.87 m front clearance if unchanged.
Keep the robot level at power-on. Measure clearance before EVERY motion run.
Stop the driver before manually repositioning the chassis. Never use the old
planner's full mode for this experiment.

## 1. Preview, no motion

Open a sourced WSL terminal inside this extracted folder:

    source /opt/ros/jazzy/setup.bash
    source ~/rm_flatness_ws/install/setup.bash
    ~/venvs/rm_flatness/bin/python calibration_pilot.py --run-id preview02 --start-gap 1.87

This creates NO command publisher, runs for eight seconds, and should print HEALTHY.
If not healthy, retain the result and resolve the reported issue. While previewing,
/rm/cmd_vel must have no publishers. Driver COMMAND_TIMEOUT is expected here.

## 2. First physical run: slow forward segment

Driver and observer stay running. Old planner remains stopped. Read this command:
--execute enables movement after five continuously healthy seconds.

    ~/venvs/rm_flatness/bin/python calibration_pilot.py --run-id slow01 --start-gap 1.87 --speed 0.04 --duration 10 --execute

Expected command is approximately 0.04 m/s for 10 s (nominal travel 0.40 m), with
small image-centering steering (|omega|<=0.10 rad/s). It automatically publishes
zero commands then records 1.5 seconds settling. Actual travel is measured, not
assumed from speed*time. Physical final gap should be near 1.47 m if travel is
close to nominal; do not enter that as an actual measurement.

Ctrl+C stops this pilot and attempts zero commands; the driver also has its own
command timeout. All stops and aborts are recorded. Do not rerun with the same
run-id or silently replace a failed attempt.

## 3. Record the physical endpoint

Once settled, measure actual front-to-box clearance and measuring instrument
resolution. Record these with pilot_runs/slow01/metadata.json, or provide them alongside the logs.
Stop the driver before manually restoring the robot to the marked start.

Send raw_events.csv, metadata.json, measured start/final clearances and ruler/laser
resolution for analysis. Raw events contain odometry headers, raw body twist,
raw SDK channel timestamps and sequences, ToF range/sensor age, tag observations,
requested and applied commands, and all state transitions/stop reasons.

## Next characterization and calibration

After inspecting slow01, a comparable fast segment can use a fresh measured start
gap, --speed 0.12 and --duration 3.333333333 (nominal 0.40 m). Do not execute it
before checking the slow run. Later collect at least five designated low-risk
calibration runs; classify whether these characterization runs meet the selected
calibration protocol explicitly. None are final evaluation runs. Freeze fitted
calibration before the equal-count filtered/unfiltered evaluation trials.

The steering signal here is normalized image error, not calibrated tag bearing
in radians. A common final steering method and this methodological change must
be documented. Pose covariance in the driver is still uncalibrated placeholders.
SDK pose/twist are not independent sensors. Ground truth comes from physical
endpoint measurements only.

## Validation

    python3 -m unittest -v test_pilot.py

Offline tests cover timing/speed bounds, single-publisher enforcement, source and
receipt staleness, sensor/driver stops, tag loss and centering. ROS and hardware
execution cannot be tested in the preparation environment. Begin with preview.
