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

Step 2A is implemented. It runs the same forward-distance navigation under
controlled ocean-current disturbances without compensation and records drift,
position error, DVL distance, Pose displacement, timing, and stop-reason metrics.

Step 2B is implemented. It is a P-only DVL body-frame velocity tracking
baseline. The controller tracks desired forward and lateral DVL velocities; it
does not blindly cancel lateral motion and it does not use the known HoloOcean
current vector.

Step 2C is implemented. It introduces a real-compatible body-frame command
abstraction and a PI DVL velocity controller. The Step 2C controller outputs
normalized body-frame commands; HoloOcean-specific 8-thruster mixing is isolated
in a simulation adapter.

Step 3 is implemented. It advances the BlueROV2 while maintaining
seabed-relative altitude with a PingAltimeter-style RangeFinder. Horizontal
motion uses DVL velocity tracking, and Pose.z is logged only as evaluation
ground truth.

Full PID, EKF, A-to-B navigation, obstacle avoidance, or perception is not
implemented yet.

## Sensor Policy

- DVL is allowed for estimation and stopping/control in Step 1, Step 2A, Step 2B, Step 2C, and Step 3 horizontal control.
- Pose is ground truth only and is used for validation and metrics.
- Velocity is ground truth only and is used for validation and metrics.
- IMU is available for future estimation/control work.
- Depth is available for future depth-control work.
- RangeFinder/Ping-style altimeter is used for Step 3 seabed-relative altitude control.
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
  actuation/              HoloOcean mixer and future actuation backend interfaces
  control/                Body-frame command and velocity dataclasses
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
  step_02b_dvl_velocity_compensation_live.py
  step_02b_lateral_axis_check.py
  step_02b_compare_compensation.py
  step_02c_dvl_pi_velocity_compensation_live.py
  step_02c_compare_pi_compensation.py
  step_03_altitude_hold.py
  run_step_03_altitude_hold_batch.py
  analyze_step_03_results.py
tests/
  run_unit_checks.py
results/
  step_00_water_visibility_check/<timestamp>/
  step_01_forward_distance/<timestamp>/
  step_01_forward_distance_batch/<timestamp>/
  step_02_current_forward_distance/<timestamp>/
  step_02_current_distance_grid/<timestamp>/
  step_02b_dvl_velocity_compensation/<timestamp>/
  step_02b_lateral_axis_check/<timestamp>/
  step_02b_compensation_comparison/<timestamp>/
  step_02c_dvl_pi_velocity_compensation/<timestamp>/
  step_02c_pi_compensation_comparison/<timestamp>/
  step_03_altitude_hold/<timestamp>/
```

The `results/` directory contains generated experiment outputs and is ignored by
git by default.

## Requirements

- Python 3.9 or compatible environment
- HoloOcean 2.2.2 with the Ocean package assets installed
- `numpy`
- `matplotlib`

The existing local environment is named `ocean`.

To recreate the environment from the minimal project file:

```bash
conda env create -f environment.yml
```

HoloOcean also requires the matching Ocean package assets to be installed for
live simulator runs.

## Validation Commands

Compile all source, examples, and lightweight tests:

```bash
conda run -n ocean python -m compileall src examples tests
```

Run non-HoloOcean unit checks:

```bash
conda run -n ocean python tests/run_unit_checks.py
```

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

## Step 2A Commands

Step 2A live without compensation:

```bash
conda run -n ocean python examples/step_02_current_forward_distance_live.py --target-distance 5 --current-y 1.0
```

Step 2A batch grid without compensation:

```bash
conda run -n ocean python examples/step_02_batch_current_distance_grid.py --target-distances 5 10 20 --current-y-values 0.0 0.25 0.5 1.0 2.0 --repetitions 3
```

Headless Step 2A batch:

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

## Step 2B Commands

Step 2B live with DVL velocity tracking compensation:

```bash
conda run -n ocean python examples/step_02b_dvl_velocity_compensation_live.py --target-distance 5 --current-y 1.0
```

Step 2B intentional lateral motion. This tracks a nonzero lateral body-frame
velocity; it is not a drift-cancel-only mode:

```bash
conda run -n ocean python examples/step_02b_dvl_velocity_compensation_live.py --target-distance 5 --current-y 1.0 --desired-lateral-velocity 0.2
```

Step 2B lateral axis diagnostic. Run this before relying on the default
`--dvl-lateral-sign` in a full comparison:

```bash
conda run -n ocean python examples/step_02b_lateral_axis_check.py --headless
```

Step 2B comparison against Step 2A:

```bash
conda run -n ocean python examples/step_02b_compare_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3
```

Resume an interrupted Step 2B comparison by pointing `--resume-dir` at the
existing timestamped comparison directory. Completed mode runs with
`summary.json` are skipped, and partial aggregate files are regenerated:

```bash
conda run -n ocean python examples/step_02b_compare_compensation.py --resume-dir results/step_02b_compensation_comparison/<timestamp> --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --headless
```

## Step 2C Commands

Step 2C adds a PI body-frame velocity controller. It returns normalized
body-frame commands (`surge`, `sway`, `heave`, `yaw`) rather than direct motor
outputs. In simulation, `HoloOceanBlueROV2Mixer` converts those commands to the
HoloOcean `control_scheme=0` 8-thruster vector. That mixer preserves the Step 2B
HoloOcean convention, but it is simulation-specific and is not a verified real
BlueROV2 motor-order mapping.

Real BlueROV2 support should use a future ArduSub/MAVLink backend that sends
body-axis commands to ArduSub's configured frame mixer. It should not hard-code
real motor ordering or send unsafe direct motor commands.

Step 2C live with PI DVL velocity tracking:

```bash
conda run -n ocean python examples/step_02c_dvl_pi_velocity_compensation_live.py --target-distance 5 --current-y 1.0 --max-duration 120 --headless
```

Step 2C focused comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 10 --current-y-values 0.5 1.0 2.0 --repetitions 3 --max-duration 120 --headless
```

Step 2C full comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --max-duration 120 --headless
```

The Step 2C live and comparison scripts default to `120.0` seconds because the
older 60 second timeout is too short for fair 20 m comparisons under strong
lateral current. The comparison script passes the same `max_duration` to
no-compensation, Step 2B, and Step 2C runs.

Reduction percentages in Step 2C comparison summaries are considered valid only
when both compared modes reach the target for that target/current group. Timeout
runs remain in the raw metrics, but their reduction-validity flags are false.

## Step 3 Commands

Step 3 moves forward while maintaining seabed-relative altitude. Horizontal
control reuses DVL velocity tracking, while vertical control uses only the
PingAltimeter RangeFinder measurement. Pose, including Pose.z, is never used by
the altitude controller and remains evaluation ground truth.

The current Step 3 controller variant is `dvl_p_altitude_hold`: DVL P velocity
tracking in surge/sway plus conservative P altitude hold in heave. Positive
vertical command is treated as upward in the HoloOcean BlueROV2
`control_scheme=0` interface. The main validated batch uses `current_y` values
`0.0`, `0.5`, and `1.0` m/s; `current_y=2.0` is outside the validated envelope
from Step 2 and is reserved for the separate stress-test batch.

Step 3 smoke/live run:

```bash
conda run -n ocean python examples/step_03_altitude_hold.py --target-distance 5 --desired-altitude 1.5 --current-y 0.0 --headless
```

Step 3 main validity batch:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type main --headless
```

Optional altitude sweep:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type altitude_sweep --headless
```

Optional stress test outside the main validated envelope:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type stress --headless
```

Regenerate Step 3 aggregate files and plots from an existing batch:

```bash
conda run -n ocean python examples/analyze_step_03_results.py results/step_03_altitude_hold/<timestamp>
```

## Step 3B OpenWater Hole-Crossing

Step 3B validates terrain following over documented OpenWater seabed
depressions. It uses the HoloOcean OpenWater documentation coordinates
(`https://byu-holoocean.github.io/holoocean-docs/v2.2.2/packages/Ocean/OpenWater/openwater.html`)
for named transects and drives across each depression along the x axis. The
vertical controller still uses only the PingAltimeter RangeFinder; Pose is used
only offline to reconstruct the seabed profile as `pose_z - ping_altitude`.

The initial tests are intentionally conservative: higher altitude, low forward
speed, no current, a larger PingAltimeter range, and the normal collision,
unsafe-altitude, invalid-Ping, and timeout checks enabled. Later runs can add
`current_y` once the no-current terrain-following behavior is validated.

Step 3B OpenWater hole-crossing run:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless
```

One-transect smoke test:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless --max-transects 1
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

Each Step 2A live run writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`
- `distance_plot.png`
- `trajectory_plot.png`
- `lateral_drift_plot.png`
- `speed_plot.png`

Each Step 2A batch-grid run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `lateral_drift_vs_current.png`
- `final_error_vs_current.png`
- `distance_error_vs_target.png`
- `lateral_drift_vs_target.png`
- `duration_vs_target_and_current.png`
- `forward_vs_euclidean_distance.png`
- one subfolder per individual run

Each Step 2B live run writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`
- `distance_plot.png`
- `trajectory_plot.png`
- `lateral_drift_plot.png`
- `velocity_tracking_plot.png`
- `command_plot.png`

Each Step 2B lateral axis diagnostic writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`

Each Step 2B comparison run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `lateral_drift_comparison.png`
- `final_position_error_comparison.png`
- `duration_comparison.png`
- `velocity_error_summary.png`
- one subfolder per individual run

Each Step 2C live run writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`
- `distance_plot.png`
- `trajectory_plot.png`
- `lateral_drift_plot.png`
- `velocity_tracking_plot.png`
- `command_plot.png`

Each Step 2C comparison run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `lateral_drift_comparison.png`
- `final_position_error_comparison.png`
- `duration_comparison.png`
- `target_reached_rate_comparison.png`
- `saturation_summary.png`
- one subfolder per individual run

Each Step 3 live run writes:

- `trajectory.csv`
- `summary.json`
- `run_config.yaml`

Each Step 3 batch run writes:

- `summary.csv`
- `aggregate_by_condition.csv`
- `metadata.json`
- `metadata.yaml`
- `logs.txt`
- `altitude_representative_runs.png`
- `altitude_error_representative_runs.png`
- `final_lateral_drift_vs_target.png`
- `rmse_altitude_error_vs_target.png`
- `time_inside_altitude_band_vs_target.png`
- one subfolder per individual run

Each Step 3B OpenWater hole-crossing run writes:

- `all_runs_summary.csv`
- `aggregate_summary.json`
- `metadata.json`
- one subfolder per named transect
- `openwater_hole_profile.png` in each transect folder

## Notes

- HoloOcean world coordinates and Pose displacements are treated as meters.
- The DVL forward velocity component defaults to index `0` with sign `+1`.
- The default forward command is `2.0`.
- Ocean current is applied before every simulation step because the local
  HoloOcean 2.2.2 API documents current as an enqueued command and does not
  state that one call persists.
- Step 2B uses the current vector only to disturb the simulator. The controller
  receives DVL velocities and desired body-frame velocities only.
- Step 2B clips forward and lateral command components separately, then clips
  the final 8-thruster command vector with `--max-thruster-command`.
- Positive Step 2B lateral command currently uses the same horizontal thruster
  pattern as keyboard left strafe: `[+, -, +, -]` on thrusters 4..7. Validate
  the DVL lateral sign with `step_02b_lateral_axis_check.py` before long runs.
- Step 2C keeps body-frame control separate from actuation. The controller does
  not know HoloOcean or real motor order; the simulation mixer is the only layer
  that builds an 8-thruster HoloOcean command vector.
- Step 3 success means the vehicle reaches the requested forward distance while
  keeping PingAltimeter altitude close to the requested altitude. The main
  metrics are altitude RMSE, max absolute altitude error, time inside the
  altitude tolerance band, final lateral drift, target reached, and timeout.
