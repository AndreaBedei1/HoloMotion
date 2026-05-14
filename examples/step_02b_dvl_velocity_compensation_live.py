from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.dvl_velocity_compensation import (
    DEFAULT_DESIRED_FORWARD_VELOCITY,
    DEFAULT_DESIRED_LATERAL_VELOCITY,
    DEFAULT_FORWARD_KP,
    DEFAULT_LATERAL_KP,
    DEFAULT_MAX_COMMAND,
    run_dvl_velocity_compensation_experiment,
)
from lib.worlds import World


def main() -> None:
    args = parse_args()
    run_dvl_velocity_compensation_experiment(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Step 2B: forward-distance run with DVL body-frame velocity tracking "
            "under ocean current."
        )
    )
    parser.add_argument("--target-distance", type=float, default=5.0)
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
    parser.add_argument("--current-y", type=float, default=1.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--kp-forward", type=float, default=DEFAULT_FORWARD_KP)
    parser.add_argument("--kp-lateral", type=float, default=DEFAULT_LATERAL_KP)
    parser.add_argument("--max-forward-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-lateral-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--base-vertical-command", type=float, default=0.0)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--speed-warning-threshold", type=float, default=1.0)
    parser.add_argument("--max-dvl-speed-warning-threshold", type=float, default=1.5)
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
        default=PROJECT_ROOT / "results" / "step_02b_dvl_velocity_compensation",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


if __name__ == "__main__":
    main()
