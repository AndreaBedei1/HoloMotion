from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.forward_controller import ForwardThrusterController
from experiment_logging.experiment_logger import ExperimentLogger
from lib.rover import Rover
from lib.scenario_builder import ScenarioConfig
from lib.visual_environment import WaterFogConfig, apply_water_fog
from lib.worlds import World


def main() -> None:
    args = parse_args()
    run_visibility_check(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 0: visual check for HoloOcean water-fog settings."
    )
    parser.add_argument("--fog-density", type=float, default=10.0)
    parser.add_argument("--fog-depth", type=float, default=0.0)
    parser.add_argument("--fog-color-r", type=float, default=0.4)
    parser.add_argument("--fog-color-g", type=float, default=0.6)
    parser.add_argument("--fog-color-b", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
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
        default=PROJECT_ROOT / "results" / "step_00_water_visibility_check",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_visibility_check(args: argparse.Namespace) -> dict:
    try:
        import holoocean
    except ImportError as exc:
        raise RuntimeError(
            "HoloOcean is not installed in this Python environment. "
            "Install HoloOcean 2.2.2 before running the visual check."
        ) from exc

    validate_args(args)

    logger = ExperimentLogger.create_timestamped(args.results_dir)
    fog_config = WaterFogConfig(
        fog_density=args.fog_density,
        fog_depth=args.fog_depth,
        color_r=args.fog_color_r,
        color_g=args.fog_color_g,
        color_b=args.fog_color_b,
    )
    run_config = {
        "world": args.world,
        "ticks_per_sec": args.ticks_per_sec,
        "duration_s": args.duration,
        "show_viewport": bool(args.show_viewport),
        "water_fog": fog_config.to_dict(),
        "checked_holoocean_api": [
            "holoocean.environments.HoloOceanEnvironment.water_fog",
            "holoocean.command.WaterFogCommand",
        ],
        "effect_scope": (
            "The setting is a visual environment check only. This project does "
            "not treat it as affecting DVL-based navigation."
        ),
    }
    logger.write_run_config(run_config)

    rov = Rover.BlueROV2Navigation(
        name="rov0",
        location=[0, 0, -4],
        rotation=[0, 0, 0],
        sensor_hz=args.ticks_per_sec,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step00_Water_Visibility_Check")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(rov)
    )
    stop_command = ForwardThrusterController(thrust=0.0).stop_command()
    step_delay_s = 1.0 / float(args.ticks_per_sec)
    total_steps = max(1, int(args.duration * args.ticks_per_sec))

    print(f"Saving Step 0 water visibility check outputs to: {logger.output_dir}")
    print("Applying HoloOcean water_fog visual setting.")

    with holoocean.make(
        scenario_cfg=scenario.to_dict(),
        show_viewport=bool(args.show_viewport),
        ticks_per_sec=args.ticks_per_sec,
        frames_per_sec=args.ticks_per_sec,
        start_world=True,
    ) as env:
        apply_water_fog(env, fog_config)
        for _ in range(total_steps):
            env.step(stop_command)
            if args.show_viewport:
                time.sleep(step_delay_s)

    summary = {
        "supported": True,
        "output_dir": str(logger.output_dir),
        "water_fog": fog_config.to_dict(),
        "duration_s": float(args.duration),
        "show_viewport": bool(args.show_viewport),
        "effect_scope": "visual only",
    }
    logger.write_summary(summary)
    print(f"Results directory: {logger.output_dir}")
    return summary


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 <= args.fog_density <= 10.0:
        raise ValueError("fog-density must be between 0.0 and 10.0.")
    if not 0.0 <= args.fog_depth <= 10.0:
        raise ValueError("fog-depth must be between 0.0 and 10.0.")
    for value_name in ("fog_color_r", "fog_color_g", "fog_color_b"):
        if not 0.0 <= getattr(args, value_name) <= 1.0:
            raise ValueError(f"{value_name.replace('_', '-')} must be between 0.0 and 1.0.")
    if args.duration <= 0:
        raise ValueError("duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")


if __name__ == "__main__":
    main()
