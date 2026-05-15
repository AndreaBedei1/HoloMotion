# HoloMotion

HoloMotion is a small HoloOcean 2.2.2 project for staged underwater navigation
experiments with a BlueROV2-style ROV. The repository focuses on reproducible,
sensor-limited control experiments before moving toward more autonomous
navigation.

## Current Status

Implemented phases:

- Step 0: water visibility / fog check.
- Step 1: DVL forward-distance estimation and stopping.
- Step 2A: forward-distance runs under ocean current without compensation.
- Step 2B: P-only DVL body-frame velocity tracking baseline.
- Step 2C: PI DVL body-frame velocity tracking with real-compatible body-frame command abstraction.
- Step 3: DVL horizontal tracking plus PingAltimeter seabed-relative altitude hold.
- Step 3B: OpenWater hole-crossing terrain-following validation with offline seabed reconstruction.

Step 4 is not implemented yet.

## Requirements

- Python 3.9 or compatible environment
- HoloOcean 2.2.2 with Ocean package assets installed
- `numpy`
- `matplotlib`

The local environment used by the project is named `ocean`.

```bash
conda env create -f environment.yml
```

## Validation

```bash
conda run -n ocean python -m compileall src examples tests
conda run -n ocean python tests/run_unit_checks.py
```

These checks do not run long HoloOcean experiments.

## Recommended Commands

Step 2C focused comparison:

```bash
conda run -n ocean python examples/step_02c_compare_pi_compensation.py --target-distances 10 --current-y-values 0.5 1.0 2.0 --repetitions 3 --max-duration 120 --headless
```

Step 3 main altitude-hold batch:

```bash
conda run -n ocean python examples/run_step_03_altitude_hold_batch.py --batch-type main --headless
```

Step 3B OpenWater hole-crossing terrain-following run:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py --headless
```

Visible Step 3B inspection run:

```bash
conda run -n ocean python examples/run_step_03_openwater_holes.py
```

## Documentation

- [Experiment commands](docs/EXPERIMENTS.md)
- [Architecture and sensor policy](docs/ARCHITECTURE.md)
- [Results and metrics guide](docs/RESULTS_GUIDE.md)
- [Project state table](docs/PROJECT_STATE.md)
- [Roadmap](docs/ROADMAP.md)

Generated experiment output is written under `results/`, which is intentionally
ignored by git.
