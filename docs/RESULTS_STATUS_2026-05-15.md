# Results Status - 2026-05-15

## Executive Summary

The latest local results support moving toward Step 4 under conservative conditions. Step 2C is clearly better than the Step 2B P-only baseline for `current_y=0.5` and `current_y=1.0`, but `current_y=2.0` remains outside the robust envelope. Step 3B has one conservative OpenWater transect that reached the target while reconstructing a changing seabed profile, but a locally recorded risky transect collided and should stay outside default validation.

Step 4 should not add new ambitions yet. It should combine Step 2C horizontal PI DVL velocity compensation with Step 3 / Step 3B PingAltimeter altitude hold under conservative currents and safe transects only.

## Step 2C Full Comparison

Command represented by the reused local result:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --desired-forward-velocity 0.5 --desired-lateral-velocity 0.0 --max-duration 120 --headless
```

Output directory:

```text
results/step_02c_pi_compensation_comparison/20260515_111553
```

Completeness:

- completed mode runs: 81 / 81
- complete flag: true
- `max_duration_s`: 120.0
- modes: `no_compensation`, `dvl_velocity_compensation`, `dvl_pi_velocity_compensation`

| target_distance | current_y | no_compensation_target_reached_rate | dvl_velocity_compensation_target_reached_rate | dvl_pi_velocity_compensation_target_reached_rate | no_compensation_timeout_rate | dvl_velocity_compensation_timeout_rate | dvl_pi_velocity_compensation_timeout_rate | no_compensation_mean_lateral_drift | dvl_velocity_compensation_mean_lateral_drift | dvl_pi_velocity_compensation_mean_lateral_drift | dvl_velocity_compensation_lateral_drift_reduction_percentage | dvl_velocity_compensation_lateral_drift_reduction_valid | dvl_pi_velocity_compensation_lateral_drift_reduction_percentage | dvl_pi_velocity_compensation_lateral_drift_reduction_valid | dvl_pi_velocity_compensation_saturation_fraction | dvl_pi_velocity_compensation_sway_saturation_fraction |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|---:|:---:|---:|---:|
| 5 | 0.5 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 0.498 | 0.244 | 0.148 | 51.1 | true | 70.4 | true | 0.002 | 0.000 |
| 5 | 1.0 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 1.560 | 0.887 | 0.579 | 43.2 | true | 62.9 | true | 0.002 | 0.000 |
| 5 | 2.0 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 5.350 | 6.371 | 6.687 | -19.1 | true | -25.0 | true | 0.787 | 0.781 |
| 10 | 0.5 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 0.997 | 0.468 | 0.195 | 53.0 | true | 80.5 | true | 0.002 | 0.000 |
| 10 | 1.0 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 3.093 | 1.761 | 0.699 | 43.0 | true | 77.4 | true | 0.001 | 0.000 |
| 10 | 2.0 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 10.616 | 12.655 | 13.183 | -19.2 | true | -24.2 | true | 0.848 | 0.841 |
| 20 | 0.5 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 1.998 | 0.906 | 0.210 | 54.6 | true | 89.5 | true | 0.001 | 0.000 |
| 20 | 1.0 | 1.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 6.166 | 3.491 | 0.734 | 43.4 | true | 88.1 | true | 0.000 | 0.000 |
| 20 | 2.0 | 1.0 | 1.0 | 0.0 | 0.0 | 0.0 | 1.0 | 21.047 | 25.159 | 25.849 | -19.5 | true | -22.8 | false | 0.876 | 0.871 |

Interpretation:

- Best Step 2C condition by lateral drift reduction was `target_distance=20`, `current_y=0.5`, with 89.5% lateral drift reduction.
- Worst Step 2C condition was `target_distance=20`, `current_y=2.0`, where Step 2C timed out in all repetitions and the reduction validity flag was false.
- `current_y=2.0` is still outside the robust envelope. Step 2C showed high total saturation and high sway saturation at all tested distances, and lateral drift was worse than no compensation.
- Step 2C is clearly better than Step 2B at `current_y=0.5` and `current_y=1.0`. It reached all targets and produced lower lateral drift than Step 2B in every moderate-current group.
- Step 2C should be the horizontal controller candidate for Step 4, but Step 4 should initially exclude `current_y=2.0`.

## Step 3B OpenWater Hole-Crossing

Command represented by the reused local result:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless
```

Output directory:

```text
results/step_03_openwater_holes/20260515_154236
```

Run summary:

- number of transects: 1
- target_reached_count: 1
- timeout_count: 0
- collision_count: 0
- unsafe_altitude_count: 0
- invalid_ping_failure_count: 0
- mean_rmse_altitude_error: 0.3077756766379239
- max_estimated_seabed_z_range: 2.4433481693267822

| transect_name | target_reached | stop_reason | collision | unsafe_altitude | invalid_ping_failure | rmse_altitude_error | max_abs_altitude_error | time_inside_altitude_band_percent | estimated_seabed_z_range | final_lateral_drift | output_dir |
|---|:---:|---|:---:|:---:|:---:|---:|---:|---:|---:|---:|---|
| openwater_depression_landmark_12 | true | target_reached | false | false | false | 0.3078 | 1.2756 | 83.19 | 2.4433 | 0.0669 | `results/step_03_openwater_holes/20260515_154236/openwater_depression_landmark_12` |

Interpretation:

- The PingAltimeter altitude controller handled a changing seabed profile in the conservative default transect. The estimated seabed range was 2.4433 m, the target was reached, and no safety failure occurred.
- Altitude tracking was not perfect: max absolute altitude error reached 1.2756 m, while time inside the altitude band was 83.19%. This is acceptable for conservative validation, but it leaves room for improving the vertical controller later.
- The default validation should keep only `openwater_depression_landmark_12` enabled. A separate local result at `results/step_03_openwater_holes/20260515_153630` shows `openwater_depression_landmark_7` stopped with `collision`, so that transect should remain outside the conservative default set.
- Step 3B is ready to be combined with Step 2C only for the safe default transect and conservative speeds. Risky transects should wait.

## Readiness for Step 4

Step 4 can proceed with conservative conditions:

- current_y: 0.0, 0.5, and 1.0 first
- desired_altitude: 3.0 for OpenWater terrain tests
- low desired_forward_velocity for terrain following
- no `current_y=2.0` initially
- no risky OpenWater transects in the first Step 4 validation batch

Step 4 should be: "Combine Step 2C horizontal PI DVL velocity compensation with Step 3/3B PingAltimeter altitude hold."

## Remaining Limitations

- Step 2C may still struggle under strong lateral current.
- Step 3 vertical controller is still P-only.
- Step 3B currently uses documented transects only.
- No EKF, planner, obstacle avoidance, perception, or real hardware backend exists.
- Pose is still evaluation-only.
