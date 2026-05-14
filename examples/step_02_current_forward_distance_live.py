from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
EXAMPLES_DIR = PROJECT_ROOT / "examples"
for path in (SRC_DIR, EXAMPLES_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from lib.worlds import World
from experiments.forward_distance import (
    DEFAULT_FORWARD_COMMAND,
    run_forward_distance_experiment,
)


def main() -> None:
    args = parse_args()
    run_forward_distance_experiment(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2: run forward-distance navigation under one ocean current."
    )
    parser.add_argument("--target-distance", type=float, default=5.0)
    parser.add_argument("--forward-command", type=float, default=DEFAULT_FORWARD_COMMAND)
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-y", type=float, default=1.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
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
        default=PROJECT_ROOT / "results" / "step_02_current_forward_distance",
    )
    parser.set_defaults(
        show_viewport=True,
        diagnostic_distance_check=False,
        make_current_plots=True,
        current_api_method="env.set_ocean_currents(agent_name, velocity)",
        scenario_name="Step02_Current_Forward_Distance",
        experiment_label="Step 2 current forward-distance",
        summary_title="Step 2 current forward-distance summary",
    )
    return parser.parse_args()


if __name__ == "__main__":
    main()
