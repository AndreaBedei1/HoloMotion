# Experiments

This file collects the runnable commands for the staged HoloMotion experiments.
Use `--headless` for unattended runs and omit it when visual inspection in the
HoloOcean viewport is useful.

## Step 0: Water Visibility Check

```bash
conda run -n ocean python examples/step_00_water_visibility_check.py --headless
```

## Step 1: DVL Forward Distance

Live run:

```bash
conda run -n ocean python examples/step_01_forward_distance_live.py --target-distance 5 --headless
```

Batch:

```bash
conda run -n ocean python examples/step_01_batch_forward_distance.py --target-distances 5 10 20 --repetitions 3 --headless
```

## Step 2A: Current Without Compensation

Live run:

```bash
conda run -n ocean python examples/step_02_current_forward_distance_live.py --target-distance 5 --current-y 1.0 --headless
```

Batch grid:

```bash
conda run -n ocean python examples/step_02_batch_current_distance_grid.py --target-distances 5 10 20 --current-y-values 0.0 0.25 0.5 1.0 2.0 --repetitions 3 --headless
```

## Step 2B: DVL Velocity Tracking Baseline

Live run:

```bash
conda run -n ocean python examples/step_02b_dvl_velocity_compensation_live.py --target-distance 5 --current-y 1.0 --headless
```

Lateral-axis diagnostic:

```bash
conda run -n ocean python examples/step_02b_lateral_axis_check.py --headless
```

Full comparison at the modern 120 s timeout:

```bash
conda run -n ocean python examples/step_02b_compare_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --desired-forward-velocity 0.5 --desired-lateral-velocity 0.0 --max-duration 120 --headless
```

Older Step 2B comparison outputs that used the previous 60 s timeout are
historical. They should not be compared directly against Step 2C 120 s runs.

## Step 2C: PI DVL Velocity Tracking

Live run:

```bash
conda run -n ocean python examples/step_02c_dvl_pi_velocity_compensation_live.py --target-distance 5 --current-y 1.0 --max-duration 120 --headless
```

Focused comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 10 --current-y-values 0.5 1.0 2.0 --repetitions 3 --desired-forward-velocity 0.5 --max-duration 120 --headless
```

Full comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 5 10 20 --current-y-values 0.5 1.0 2.0 --repetitions 3 --desired-forward-velocity 0.5 --desired-lateral-velocity 0.0 --max-duration 120 --headless
```

Step 2C comparison passes the same `max_duration` to no-compensation, Step 2B,
and Step 2C modes.

## Step 3: Seabed-Relative Altitude Hold

Smoke/live run:

```bash
conda run -n ocean python examples/step_03_altitude_hold.py --target-distance 5 --desired-altitude 1.5 --current-y 0.0 --headless
```

Main validity batch:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type main --headless
```

Optional altitude sweep:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type altitude_sweep --headless
```

Stress test outside the main validated envelope:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type stress --headless
```

Regenerate aggregate files and plots:

```bash
conda run -n ocean python examples/analyze_step_03_results.py results/step_03_altitude_hold/<timestamp>
```

## Step 3B: OpenWater Hole-Crossing Terrain Following

Headless batch command:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless
```

Visible viewport command for qualitative inspection:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py
```

One-transect smoke test:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless --max-transects 1
```

Conservative slow run:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --max-transects 1 --desired-forward-velocity 0.10 --desired-altitude 3.0 --current-y 0.0 --max-duration 220
```
