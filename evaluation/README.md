# Assignment 3: frozen evaluation workflow

The tested feedback core, EKF equations, gains, caps, noise settings and conservative raw guard are unchanged from commissioning. This release adds a freeze manifest, evaluation labels, planned run IDs, retry handling and progress reporting. Preview remains the default; only `--execute` creates the single motor-command publisher. The existing driver remains the sole SDK owner.

## Decisions before evaluation

- Nominal marked start: measured 0.60 m front-edge-to-box-face gap. Enter the actual measurement every time; this experiment accepts starts from 0.60 to 0.62 m. If the gap falls outside that band, reposition before running.
- Physical reference goal remains 0.20 m, L=d0-0.20, T=max(4,1.6L/0.16). At 0.60 m start, L=0.40 m and T=4 s.
- Raw ToF supervisor guard remains 0.25 m; driver raw 0.20 m latch and command timeout stay unchanged. Both methods use the same raw 0.50 m near-box threshold and 0.06 m/s speed cap.
- Commissioning stopped at physical gaps 0.250 m and 0.256 m. These early guard stops are retained as a known limitation, not relabeled as reaching the physical target. A final-gap error around +5 cm may remain in evaluation. Do not change the guard mid-experiment to improve a score.
- Calibration remains b_o=0.006774102 m and s_T=0.897081413, with the same declared Q/R priors and 2 s bias ramp. The far-distance ToF travel scale underestimated travel in shorter commissioning runs. This is a documented calibration-domain limitation; no EKF improvement is presumed. A supported negative result is acceptable under the handout.
- Keep the upright tag, arm pose (105,35) mm, image transport, lighting and physical measurement reference fixed. Upright placement differs from the earlier Phase 2 floor-tag geometry and must be disclosed in the report.

## Install, test and freeze while robot nodes are stopped

Extract into `~/assignment3_tools`, then:

```bash
cd ~/assignment3_tools/assignment3_evaluation
~/venvs/rm_flatness/bin/python -m unittest -v test_feedback.py test_freeze.py
~/venvs/rm_flatness/bin/python freeze_protocol.py
```

There are 17 tests: the unchanged 14 control checks plus method/retry identity, changed-manifest rejection and physical-endpoint counting. These are software checks, not physical certification.

The freeze helper does not connect to ROS or the robot. It checks the two existing commissioning metadata files under `~/assignment3_tools/assignment3_feedback/feedback_runs`, compares the commissioned controller core and numeric settings, and preserves confirmed physical measurements in a separate sidecar. It does not change the original commissioning records. An empty u01 physical endpoint in the old metadata is supported by the user's previously confirmed sidecar; a conflicting recorded endpoint raises an error.

It verifies that the built driver matches source, records driver/odometry-support/observer files and package versions, copies the source robot package and observer into `frozen_robot_sources`, then writes `freeze_manifest.json`. Repeating freeze verifies the existing manifest; it does not overwrite it. A changed frozen file, added source/config file, external driver/observer source or dependency environment prevents evaluation. Use the same virtual environment for freeze and every evaluation command.

Hash checks cannot verify which camera mount, physical tag or tape reference is in use, or whether the observer was launched with the correct OpenCV directory. Follow the working launch commands and maintain the physical setup yourself. Source hashes do not replace runtime health checks.

## Robot launch and preview

Keep systemd-timesyncd temporarily stopped during this robot session as previously tested; restore it after stopping robot nodes at session end. Clock/freshness stops remain enabled because WSL host adjustments can still occur.

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
cd ~/assignment3_tools/assignment3_evaluation
~/venvs/rm_flatness/bin/python feedback_runner.py \
  --run-id frozen_preview01 --method unfiltered --start-gap 0.60 --evaluation
```

This has no command publisher because `--execute` is absent. The preview uses the frozen-file checks and live health checks. Keep all other planners/pilots stopped. Preview records are not evaluation runs.

## Ten-run plan

Alternate methods in this order:

| Order | Run ID | Method |
|---|---|---|
| 1 | eval_u01 | unfiltered |
| 2 | eval_f01 | filtered |
| 3 | eval_u02 | unfiltered |
| 4 | eval_f02 | filtered |
| 5 | eval_u03 | unfiltered |
| 6 | eval_f03 | filtered |
| 7 | eval_u04 | unfiltered |
| 8 | eval_f04 | filtered |
| 9 | eval_u05 | unfiltered |
| 10 | eval_f05 | filtered |

For each run: stop driver before repositioning; restore the marked start; measure the actual front gap; restart driver with arm area clear; keep the same observer; execute the planned method; allow settling; measure final physical gap before reset; record it; retain the complete logs. If an already successful run looks exceptional, retain it in the primary results. Do not replace it with a nicer run.

Example first executed evaluation (only after frozen preview review):

```bash
~/venvs/rm_flatness/bin/python feedback_runner.py \
  --run-id eval_u01 --method unfiltered --start-gap 0.60 --evaluation --execute
```

It enables movement after five continuously healthy seconds. Ctrl+C stops the runner. All driver, source/receipt freshness, extra-publisher, callback-latched fault, clock-jump, no-reverse, speed-cap and bounded-runtime rules remain unchanged. Guard/reference stops are protocol completion; this does not mean the 0.20 m physical goal was reached. Fault/operator/runtime stops are aborts.

Measure and record the settled endpoint, in metres:

```bash
~/venvs/rm_flatness/bin/python record_endpoint.py \
  --run-id eval_u01 --final-gap YOUR_MEASURED_METRES --resolution 0.001
```

The 1 mm tape division is measurement resolution, not guaranteed accuracy. Record endpoints after aborts too. Use e.g. `eval_u01_retry01` for an aborted slot's retry, with the same method and a fresh physical start measurement. A completed evaluation slot cannot be rerun to replace its result. Existing run folders cannot be overwritten.

Progress check:

```bash
~/venvs/rm_flatness/bin/python evaluation_status.py
```

Only completed evaluation slots with physical endpoint/resolution and matching frozen-manifest identity count toward the five per method. All attempts remain in the output tree. After all ten measured slots, inspect raw logs for valid execution and perform the required analyses; counts alone are not a final data-quality audit.

## Send results for analysis

From this directory, archive the entire evaluation release and outputs after stopping robot nodes:

```bash
cd ~/assignment3_tools
python3 -m zipfile -c assignment3_evaluation_results.zip assignment3_evaluation
explorer.exe "$(wslpath -w "$HOME/assignment3_tools")"
```

This includes the frozen manifest and source snapshots, configuration, raw CSVs, metadata and physical measurements. Keep the earlier calibration and commissioning archives separately for the final consolidated submission. Do not make the repository public: the current handout asks for a ZIP or private repository and an explanation of reused code.

## Remaining report work

Use independent evaluation only for final physical endpoint scores. On the same filtered runs compare raw odometry, bias-corrected odometry and EKF error versus measured travel. Report mean, sample SD, MAE, RMSE, range and completion/abort counts. Produce reference/estimate/speed traces, estimated tracking-error mean +/- SD, endpoint validation, and aligned ToF residual/correlation figures. ToF is an EKF input, not independent continuous ground truth. Discuss the early hard stop, calibration mismatch, priors, covariance/latency limits, upright tag choice, outliers and negative findings. Finish the LaTeX report, full code/log archive and 4–5 minute presentation.
