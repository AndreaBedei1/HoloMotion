# Roadmap

## Completed / Implemented

- Step 0: water visibility / fog check.
- Step 1: DVL forward-distance estimation and stopping.
- Step 2A: current disturbance runs without compensation.
- Step 2B: P-only DVL velocity tracking compensation.
- Step 2C: PI DVL velocity tracking with body-frame command abstraction.
- Step 3: PingAltimeter seabed-relative altitude hold.
- Step 3B: OpenWater hole-crossing terrain-following validation.

## Current Validation Status

- Current project snapshot: [CURRENT_STATUS_2026-05-15.md](CURRENT_STATUS_2026-05-15.md).
- Latest experimental results status: [RESULTS_STATUS_2026-05-15.md](RESULTS_STATUS_2026-05-15.md).
- Step 2B and Step 2C are available for 120 s comparisons.
- Step 2C improved robustness at moderate lateral current, but strong
  `current_y=2.0` remains outside the reliable envelope for long 20 m targets.
- Step 3 main batch completed for `current_y=0.0`, `0.5`, and `1.0` with
  target reached in the recorded main run.
- Step 3B has a conservative OpenWater transect that reached target and
  reconstructed a changing seabed profile.

## Open Limitations

- Step 4 is not implemented yet.
- Step 3 altitude control is P-only.
- Step 3B transects need more systematic full-batch analysis.
- Current handling is still simulator-specific.
- No EKF, A-to-B planner, obstacle avoidance, or real hardware backend exists.
- Real BlueROV2 motor order is intentionally not hard-coded.

## Proposed Next Steps

1. Step 2D: BlueROV2 Heavy / T200 actuator-authority sweep.
   Purpose: determine whether `current_y=2.0` failure is caused by conservative
   software limits, mixer allocation, drag/dynamics, or true vehicle authority
   limitations.
2. Finish uniform Step 2B / Step 2C full batch comparison at `--max-duration 120`.
3. Analyze Step 3B full OpenWater hole runs.
4. Optionally improve the Step 3 altitude controller after reviewing P-only results.
5. Step 4: combine Step 2C horizontal PI compensation with Step 3 / Step 3B altitude hold.
6. Add future EKF and A-to-B navigation.
7. Add a future real BlueROV2 ArduSub/MAVLink backend that sends body-axis commands.
