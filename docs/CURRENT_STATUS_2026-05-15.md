# Current Status - 2026-05-15

## Project Summary

HoloMotion is a staged HoloOcean / BlueROV2-style experimental project for underwater navigation. It currently validates sensor-limited forward motion, current disturbance handling, DVL-based horizontal velocity control, PingAltimeter seabed-relative altitude hold, and conservative OpenWater terrain-following transects.

Step 4 is not implemented yet.

Latest experimental results status: [RESULTS_STATUS_2026-05-15.md](RESULTS_STATUS_2026-05-15.md).

## Implemented Steps

| Step | What it tests | Control sensors | Evaluation-only sensors | Current status | Main limitation |
|---|---|---|---|---|---|
| Step 0 | Water visibility / fog check. | None | Viewport / scene output | Implemented | Qualitative environment check only. |
| Step 1 | Forward-distance stopping from DVL integration. | DVL | Pose, Velocity | Implemented | No current compensation or altitude control. |
| Step 2A | Forward-distance runs under current without compensation. | DVL for stopping | Pose, Velocity | Implemented | Baseline only; lateral drift is expected. |
| Step 2B | P-only DVL body-frame velocity tracking. | DVL | Pose, Velocity | Implemented | P-only baseline; legacy public controller returns HoloOcean thruster vectors for compatibility. |
| Step 2C | PI DVL velocity tracking with body-frame command abstraction. | DVL | Pose, Velocity | Implemented | Strong lateral current can still saturate and timeout, especially long targets at `current_y=2.0`. |
| Step 3 | Forward motion while holding seabed-relative altitude. | DVL, PingAltimeter | Pose, Velocity | Implemented | Altitude control is P-only; main validation excludes `current_y=2.0`. |
| Step 3B | OpenWater hole-crossing / terrain-following validation. | DVL, PingAltimeter | Pose, Velocity | Implemented | Needs more full-run analysis over all documented transects. |

## Latest Known Experimental Results

- Step 2B old 60 s runs are historical and are not directly comparable to Step 2C runs that use the modern 120 s timeout policy.
- Step 2C smoke result is available locally at `results/step_02c_dvl_pi_velocity_compensation/20260515_101628/summary.json`: target reached under `current_y=1.0`, `target_distance=5.0`, with `mean_abs_sway_velocity_error=0.0388773277537977`, `saturation_fraction=0.0017301038062283738`, `surge_saturation_fraction=0.0017301038062283738`, and `sway_saturation_fraction=0.0`.
- Step 3 main batch result is available locally at `results/step_03_altitude_hold/20260515_135217/summary.csv`: 27/27 runs reached the target with 0 timeouts, 0 collisions, 0 unsafe-altitude failures, and 0 invalid-Ping failures. Across grouped conditions, mean RMSE altitude error ranges from 0.140281132769696 m to 0.213780420829695 m, and mean final lateral drift ranges from 0.0090057612057232 m to 3.66083118785173 m.
- Step 3B smoke result is available locally at `results/step_03_openwater_holes/20260515_154236/aggregate_summary.json`: `openwater_depression_landmark_12` reached the target with no collision, no unsafe-altitude failure, and no invalid-Ping failure. It reported `rmse_altitude_error=0.3077756766379239`, `max_abs_altitude_error=1.27555513381958`, `estimated_seabed_z_range=2.4433481693267822`, and `time_inside_altitude_band_percent=83.19035947712419`.

## Current Limitations

- Step 2B is only a P-controller baseline.
- Step 2C still needs a full uniform batch comparison at 120 s.
- Step 3 altitude hold is P-only.
- Step 3B needs more full-run analysis.
- Step 4 is not implemented.
- No EKF, A-to-B navigation, obstacle avoidance, perception, or real BlueROV2 backend exists yet.

## Recommended Next Commands

Step 2C full uniform comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --desired-forward-velocity 0.5 --desired-lateral-velocity 0.0 --max-duration 120 --headless
```

Step 3B full OpenWater hole run:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless
```

Step 3B visible inspection:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py
```

Validation:

```bash
conda run -n ocean python -m compileall src examples tests
conda run -n ocean python tests/run_unit_checks.py
```

## Next Development Decision

The next development step should be Step 4: combine Step 2C horizontal PI current compensation with Step 3 / Step 3B PingAltimeter altitude hold. This should happen only after reviewing the full Step 2C comparison results and the Step 3B OpenWater results.
