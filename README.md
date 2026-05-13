# HoloMotion

HoloMotion is a small Python project around HoloOcean 2.2.2 for underwater
robotics simulation with a BlueROV2-style ROV. The goal is to build a clean,
interpretable navigation library for GPS-denied underwater motion, developed in
small validated steps.

## Current Status

The current validated baseline is Step 1: move the ROV forward by a target
distance using DVL-based distance estimation, then compare the estimate against
HoloOcean Pose ground truth.

Step 1.1 runs the same experiment in batches across multiple target distances.
Generated Step 2 results were removed for now; generated results are ignored by
git by default.

No altitude control, PID, EKF, current compensation, or A-to-B navigation is
implemented yet.

## Sensor Policy

- DVL is used for forward-distance estimation and stopping.
- Pose is ground truth only and is used for validation and metrics.
- Velocity is ground truth only.
- IMU and Depth are available for later estimation/control steps.
- RangeFinder/Ping-style altimeter will be used in later altitude-control steps.
- Cameras and imaging sonar are not required for the Step 1 baseline.

## Project Structure

```text
src/
  lib/                    HoloOcean scenario, rover, world, and sensor builders
  controllers/            Simple reusable control helpers
  estimators/             DVL distance estimator
  metrics/                Distance metrics
  experiment_logging/     CSV, JSON, and run-config output helpers
  visualization/          Plotting helpers
  telemetry/              Small telemetry parsing helpers
  utils/                  General utility helpers
examples/
  step_01_forward_distance_live.py
  step_01_batch_forward_distance.py
  step_02_strong_current_forward_distance_live.py
  step_02_batch_current_forward_distance.py
results/
  step_01_forward_distance/<timestamp>/
  step_01_forward_distance_batch/<timestamp>/
```

The `results/` directory contains generated experiment outputs and is not
tracked by git.

## Requirements

- Python 3.9 or compatible environment
- HoloOcean 2.2.2 with the Ocean package assets installed
- `numpy`
- `matplotlib`

The existing local environment is named `ocean`.

## Step 1 Live Run

Run from the project root:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5
```

The HoloOcean viewport is visible by default. To run without the viewport:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5 --headless
```

For a slow diagnostic distance check:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5 --diagnostic-distance-check
```

## Step 1.1 Batch Run

Default batch:

```bash
conda run -n ocean python examples/step_01_batch_forward_distance.py
```

Custom batch:

```bash
conda run -n ocean python examples/step_01_batch_forward_distance.py --target-distances 2 5 10 20 --repetitions 3
```

Headless batch:

```bash
conda run -n ocean python examples/step_01_batch_forward_distance.py --headless
```

## Outputs

Each Step 1 live run writes:

- `trajectory.csv`
- `summary.json`
- `distance_plot.png`
- `trajectory_plot.png`
- `run_config.yaml`

Each Step 1.1 batch run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `distance_error_vs_target.png`
- `percentage_error_vs_target.png`
- `lateral_drift_vs_target.png`
- `duration_vs_target.png`
- one subfolder per individual run

## Notes

- HoloOcean world coordinates and Pose displacements are treated as meters.
- The DVL forward velocity component defaults to index `0` with sign `+1`.
- The current default forward command is `2.0`; the older default was `12.0`.
- Generated Step 2 result folders were removed during cleanup. Step 2 scripts
  remain available for later validation, but their generated outputs should not
  be committed unless explicitly requested.
