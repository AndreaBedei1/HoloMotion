from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.altitude_hold import (
    DEFAULT_ALTITUDE_TOLERANCE,
    DEFAULT_DESIRED_ALTITUDE,
    DEFAULT_DESIRED_FORWARD_VELOCITY,
    DEFAULT_DESIRED_LATERAL_VELOCITY,
    DEFAULT_FLAT_SEABED_Z,
    DEFAULT_INITIAL_X,
    DEFAULT_KP_ALTITUDE,
    DEFAULT_MAX_DURATION,
    DEFAULT_MAX_INVALID_PING_HOLD,
    DEFAULT_MAX_THRUSTER_COMMAND,
    DEFAULT_MAX_VERTICAL_COMMAND,
    DEFAULT_MIN_SAFE_ALTITUDE,
    run_step_03_altitude_hold_experiment,
)
from experiments.dvl_velocity_compensation import (
    DEFAULT_FORWARD_KP,
    DEFAULT_LATERAL_KP,
    DEFAULT_MAX_COMMAND,
)
from lib.worlds import World


def main() -> None:
    args = parse_args()
    run_step_03_altitude_hold_experiment(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Step 3: move forward with DVL velocity tracking while holding "
            "seabed-relative altitude using a PingAltimeter RangeFinder."
        )
    )
    parser.add_argument("--target-distance", type=float, default=5.0)
    parser.add_argument("--desired-altitude", type=float, default=DEFAULT_DESIRED_ALTITUDE)
    parser.add_argument("--altitude-tolerance", type=float, default=DEFAULT_ALTITUDE_TOLERANCE)
    parser.add_argument("--min-safe-altitude", type=float, default=DEFAULT_MIN_SAFE_ALTITUDE)
    parser.add_argument(
        "--desired-forward-velocity",
        type=float,
        default=DEFAULT_DESIRED_FORWARD_VELOCITY,
    )
    parser.add_argument(
        "--desired-lateral-velocity",
        type=float,
        default=DEFAULT_DESIRED_LATERAL_VELOCITY,
    )
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-y", type=float, default=0.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--repetition", type=int, default=0)
    parser.add_argument("--kp-altitude", type=float, default=DEFAULT_KP_ALTITUDE)
    parser.add_argument("--max-vertical-command", type=float, default=DEFAULT_MAX_VERTICAL_COMMAND)
    parser.add_argument("--kp-forward", type=float, default=DEFAULT_FORWARD_KP)
    parser.add_argument("--kp-lateral", type=float, default=DEFAULT_LATERAL_KP)
    parser.add_argument("--max-forward-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-lateral-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument(
        "--max-thruster-command",
        type=float,
        default=DEFAULT_MAX_THRUSTER_COMMAND,
    )
    parser.add_argument("--max-duration", type=float, default=DEFAULT_MAX_DURATION)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--flat-seabed-z", type=float, default=DEFAULT_FLAT_SEABED_Z)
    parser.add_argument("--initial-x", type=float, default=DEFAULT_INITIAL_X)
    parser.add_argument("--ping-max-range", type=float, default=50.0)
    parser.add_argument(
        "--max-invalid-ping-hold-s",
        type=float,
        default=DEFAULT_MAX_INVALID_PING_HOLD,
    )
    parser.add_argument("--world", choices=World.list_worlds(), default=World.SimpleUnderwater)
    parser.add_argument(
        "--headless",
        action="store_false",
        dest="show_viewport",
        help="Run without the HoloOcean viewport.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "step_03_altitude_hold",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


if __name__ == "__main__":
    main()
