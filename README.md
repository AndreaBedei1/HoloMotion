# HoloMotion

HoloMotion is a small Python project around HoloOcean 2.2.2 for underwater
robotics simulation with a BlueROV2-style ROV. The goal is to build a clean,
interpretable navigation library for GPS-denied underwater motion, developed in
small validated steps.

## Current Status

Step 1 is completed. It moves the ROV forward over target distances using
DVL-based distance estimation and stopping, then compares the estimate against
HoloOcean Pose ground truth.

Step 1.1 runs Step 1 in batches over multiple target distances and repetitions.

Step 2 is implemented. It runs the same forward-distance navigation under
controlled ocean-current disturbances and records drift, position error, DVL
distance, Pose displacement, timing, and stop-reason metrics.

No altitude control, PID, EKF, A-to-B navigation, obstacle avoidance, perception,
or current compensation is implemented yet.

## Sensor Policy

- DVL is allowed for estimation and stopping/control in Step 1 and Step 2.
- Pose is ground truth only and is used for validation and metrics.
- Velocity is ground truth only and is used for validation and metrics.
- IMU is available for future estimation/control work.
- Depth is available for future depth-control work.
- RangeFinder/Ping-style altimeter is reserved for future altitude-from-seabed control.
- Cameras and imaging sonar are not required for Step 1 or Step 2.

## Water Turbidity / Fog Note

The local HoloOcean 2.2.2 Python package was checked for water fog, turbidity,
visibility, and post-processing support. It exposes
`HoloOceanEnvironment.water_fog(...)` and `WaterFogCommand`, with documented
parameters for water-fog density, depth, and color. No separate turbidity
physics API was found in the local package.

The project includes `examples/step_00_water_visibility_check.py` and
`src/lib/visual_environment.py` for this visual check. The water-fog setting is
treated as visual only. It must not be interpreted as affecting DVL-based
navigation unless HoloOcean explicitly models that physical effect.

## Project Structure

```text
src/
  controllers/            Simple reusable control helpers
  estimators/             DVL distance estimator
  experiments/            Shared experiment runners used by examples
  experiment_logging/     CSV, JSON, and run-config output helpers
  lib/                    HoloOcean scenario, rover, current, visual helpers
  metrics/                Distance and drift metrics
  telemetry/              Small telemetry parsing helpers
  utils/                  General utility helpers
  visualization/          Plotting helpers
examples/
  step_00_water_visibility_check.py
  step_01_forward_distance_live.py
  step_01_batch_forward_distance.py
  step_02_current_forward_distance_live.py
  step_02_batch_current_distance_grid.py
results/
  step_00_water_visibility_check/<timestamp>/
  step_01_forward_distance/<timestamp>/
  step_01_forward_distance_batch/<timestamp>/
  step_02_current_forward_distance/<timestamp>/
  step_02_current_distance_grid/<timestamp>/
```

The `results/` directory contains generated experiment outputs and is ignored by
git by default.

## Requirements

- Python 3.9 or compatible environment
- HoloOcean 2.2.2 with the Ocean package assets installed
- `numpy`
- `matplotlib`

The existing local environment is named `ocean`.

## Step 0 Water Visibility Check

```bash
conda run -n ocean python examples/step_00_water_visibility_check.py
```

The viewport is visible by default. To run without the viewport:

```bash
conda run -n ocean python examples/step_00_water_visibility_check.py --headless
```

## Step 1 Commands

Step 1 live:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5
```

Step 1 batch:

```bash
conda run -n ocean python examples/step_01_batch_forward_distance.py --target-distances 5 10 20 --repetitions 3
```

Headless Step 1 live:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5 --headless
```

## Step 2 Commands

Step 2 live:

```bash
conda run -n ocean python examples/step_02_current_forward_distance_live.py --target-distance 5 --current-y 1.0
```

Step 2 batch grid:

```bash
conda run -n ocean python examples/step_02_batch_current_distance_grid.py --target-distances 5 10 20 --current-y-values 0.0 0.25 0.5 1.0 2.0 --repetitions 3
```

Headless Step 2 batch:

```bash
conda run -n ocean python examples/step_02_batch_current_distance_grid.py --target-distances 5 10 20 --current-y-values 0.0 0.25 0.5 1.0 2.0 --repetitions 3 --headless
```

Frontal and following current cases are included by default in the Step 2 batch
grid using `--frontal-current-values -0.5 0.5`. Use `--no-frontal-currents` to
run only the lateral-current grid.

Resume an interrupted Step 2 batch by pointing `--resume-dir` at the existing
timestamped batch directory. Completed runs with `summary.json` are skipped:

```bash
conda run -n ocean python examples/step_02_batch_current_distance_grid.py --resume-dir results/step_02_current_distance_grid/<timestamp> --target-distances 5 10 20 --current-y-values 0.0 0.25 0.5 1.0 2.0 --repetitions 3
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

Each Step 2 live run writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`
- `distance_plot.png`
- `trajectory_plot.png`
- `lateral_drift_plot.png`
- `speed_plot.png`

Each Step 2 batch-grid run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `lateral_drift_vs_current.png`
- `final_error_vs_current.png`
- `distance_error_vs_target.png`
- `lateral_drift_vs_target.png`
- `duration_vs_target_and_current.png`
- `forward_vs_euclidean_distance.png`
- one subfolder per individual run

## Notes

- HoloOcean world coordinates and Pose displacements are treated as meters.
- The DVL forward velocity component defaults to index `0` with sign `+1`.
- The default forward command is `2.0`.
- Ocean current is applied before every simulation step because the local
  HoloOcean 2.2.2 API documents current as an enqueued command and does not
  state that one call persists.
