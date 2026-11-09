# RoboMaster EP: State Estimation and Feedback Approach

SCE 672 — Assignment 3, Mohammad Ahmed Khan.

This repository contains the ROS 2 robot package, AprilTag observer, calibration pilots, five-state EKF, feedback controllers, tests, analysis scripts, original experimental logs and report sources.

## Results

Ten evaluation runs were completed: five unfiltered and five filtered. Every run stopped at the common raw ToF guard. Mean measured final clearances were 0.2550 m and 0.2606 m respectively; the physical goal was 0.200 m. The results do not demonstrate a meaningful improvement in physical stopping accuracy from the EKF. See the report for details, scatter plots and variance/covariance analysis.

## Contents

| Directory | Contents |
| --- | --- |
| `evaluation/frozen_robot_sources/rm_flatness_nav/` | Frozen ROS 2 package: driver, odometry, planner and logger |
| `evaluation/frozen_robot_sources/tag_observer.py` | Upright AprilTag ID 3 observer |
| `evaluation/` | Tested EKF, feedback controller, configuration, freeze checks, tests and ten raw evaluation logs |
| `pilot/` | Calibration motion controller and tests |
| `calibration/` | Calibration scripts, original pilot logs and measured endpoints |
| `ekf_development/` | Offline EKF replay, live preview and tests |
| `analysis/` | Offline log analysis, scatter plots, statistics and extracted CSVs |
| `report/` | Word/PDF reports, editable LaTeX source and rebuild scripts |
| `commissioning/`, `diagnostics/` | Commissioning measurements and available development abort logs |
| `presentation/` | Presentation reference and speaking notes |

## Reproduce the analysis without a robot

For the commands below, use the complete original tree extracted from `original_evidence.zip` on Linux. The visible Windows copy omits metadata sidecars referenced by the original freeze manifest.

Use Python 3.12 in a separate environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements_analysis.txt
python analysis/analyze_evaluation.py
python analysis/trial_statistics.py
```

These scripts do not import ROS or send robot commands. They validate the frozen experiment and regenerate the figures, endpoint tables, sample variances and covariance matrices. One endpoint per trial is used for statistics (five trials per method), rather than treating periodic samples as independent trials.

Controller tests:

```bash
cd evaluation
python -m unittest -v test_feedback.py test_freeze.py
cd ..
```

See `README.txt` for the full environment record, report rebuild commands and measurement policies. Original paths in metadata document the experiment computer and are not portable launch instructions.

## Hardware execution

The driver can move the arm at startup. Execution flags enable chassis motion; preview and offline analysis have separate purposes. Do not run hardware scripts merely to reproduce the report. The frozen evaluation software and its evidence must remain unchanged. The original ROS driver was the sole SDK owner, with competing planners stopped.

This is a Windows-compatible browsing/upload copy. Ten colon-containing Windows metadata sidecars are omitted from the visible tree; every Python file, runtime setting and raw CSV remains unchanged. The complete original tree is preserved in `original_evidence.zip`. For frozen-hash verification and offline reproduction, extract that evidence ZIP into a separate Linux directory and use its original tree: the Windows copy alone will fail checks for the omitted sidecars. See `WINDOWS_COMPATIBILITY.txt`. `.gitattributes` disables text conversion to preserve the recorded hashes.

## Attribution and visibility

The work reuses the earlier ROS 2 project and the DJI RoboMaster SDK, ROS 2, OpenCV, NumPy and Matplotlib. ChatGPT/Codex assisted implementation, debugging, analysis and report preparation; the student performed the real robot trials and supplied the physical measurements. Dependencies are installed separately, not bundled here. No blanket license is added for third-party or reused material.

The supplied assignment handout requests a private repository. Keep this repository private unless the instructor has explicitly superseded that requirement. `UPLOAD_TO_GITHUB.md` explains both visibility choices. The assignment handout itself is not included.
