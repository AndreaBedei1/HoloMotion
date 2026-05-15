from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.dvl_pi_velocity_compensation import (
    DEFAULT_DESIRED_FORWARD_VELOCITY,
    DEFAULT_DESIRED_LATERAL_VELOCITY,
    DEFAULT_INTEGRAL_LIMIT_SURGE,
    DEFAULT_INTEGRAL_LIMIT_SWAY,
    DEFAULT_KI_SURGE,
    DEFAULT_KI_SWAY,
    DEFAULT_KP_SURGE,
    DEFAULT_KP_SWAY,
    DEFAULT_MAX_DURATION,
    DEFAULT_MAX_SURGE,
    DEFAULT_MAX_SWAY,
    DEFAULT_MAX_THRUSTER_COMMAND,
    run_dvl_pi_velocity_compensation_experiment,
)
from lib.worlds import World


def main() -> None:
    args = parse_args()
    run_dvl_pi_velocity_compensation_experiment(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Step 2C: forward-distance run with DVL PI body-frame velocity "
            "tracking under ocean current."
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
    parser.add_argument("--kp-surge", type=float, default=DEFAULT_KP_SURGE)
    parser.add_argument("--ki-surge", type=float, default=DEFAULT_KI_SURGE)
    parser.add_argument("--kp-sway", type=float, default=DEFAULT_KP_SWAY)
    parser.add_argument("--ki-sway", type=float, default=DEFAULT_KI_SWAY)
    parser.add_argument("--max-surge", type=float, default=DEFAULT_MAX_SURGE)
    parser.add_argument("--max-sway", type=float, default=DEFAULT_MAX_SWAY)
    parser.add_argument(
        "--integral-limit-surge",
        type=float,
        default=DEFAULT_INTEGRAL_LIMIT_SURGE,
    )
    parser.add_argument(
        "--integral-limit-sway",
        type=float,
        default=DEFAULT_INTEGRAL_LIMIT_SWAY,
    )
    parser.add_argument(
        "--max-thruster-command",
        type=float,
        default=DEFAULT_MAX_THRUSTER_COMMAND,
        help="Simulation mixer limit applied to each HoloOcean thruster command.",
    )
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--max-duration", type=float, default=DEFAULT_MAX_DURATION)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
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
        default=PROJECT_ROOT / "results" / "step_02c_dvl_pi_velocity_compensation",
    )
    parser.set_defaults(
        show_viewport=True,
        speed_warning_threshold=1.0,
        max_dvl_speed_warning_threshold=1.5,
    )
    return parser.parse_args()


if __name__ == "__main__":
    main()
