from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.dvl_velocity_controller import bluerov2_horizontal_command
from experiment_logging.experiment_logger import ExperimentLogger
from experiments.dvl_velocity_compensation import dvl_velocity_components
from experiments.forward_distance import (
    DVL_SENSOR_KEY,
    POSE_SENSOR_KEY,
    pose_components,
    require_sensor,
    run_warmup,
)
from lib.rover import Rover
from lib.scenario_builder import ScenarioConfig
from lib.worlds import World


def main() -> None:
    args = parse_args()
    run_lateral_axis_check(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2B diagnostic for BlueROV2 lateral command and DVL lateral axis sign."
    )
    parser.add_argument("--lateral-command", type=float, default=0.5)
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Duration in seconds for each positive/negative lateral command phase.",
    )
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
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
        default=PROJECT_ROOT / "results" / "step_02b_lateral_axis_check",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_lateral_axis_check(args: argparse.Namespace) -> dict:
    try:
        import holoocean
    except ImportError as exc:
        raise RuntimeError(
            "HoloOcean is not installed in this Python environment. "
            "Install HoloOcean 2.2.2 before running the diagnostic."
        ) from exc

    validate_args(args)

    logger = ExperimentLogger.create_timestamped(args.results_dir)
    command_magnitude = abs(float(args.lateral_command))
    stop_command = np.zeros(8, dtype=float)
    samples: list[dict] = []

    rov = Rover.BlueROV2Navigation(
        name="rov0",
        location=[0, 0, -4],
        rotation=[0, 0, 0],
        sensor_hz=args.ticks_per_sec,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step02B_Lateral_Axis_Check")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(rov)
    )

    run_config = {
        "lateral_command": command_magnitude,
        "duration_per_phase_s": float(args.duration),
        "current_x": 0.0,
        "current_y": 0.0,
        "current_z": 0.0,
        "current_magnitude": 0.0,
        "ticks_per_sec": int(args.ticks_per_sec),
        "world": args.world,
        "dvl_lateral_index": int(args.dvl_lateral_index),
        "dvl_lateral_sign": float(args.dvl_lateral_sign),
        "show_viewport": bool(args.show_viewport),
        "purpose": (
            "Measure DVL lateral velocity response to positive and negative "
            "lateral-only thruster commands. Pose is logged for reporting only."
        ),
    }
    logger.write_run_config(run_config)

    print(f"Saving Step 2B lateral axis check outputs to: {logger.output_dir}")
    print(f"Positive lateral command pattern uses thrusters 4..7: [+, -, +, -].")

    with holoocean.make(
        scenario_cfg=scenario.to_dict(),
        show_viewport=bool(args.show_viewport),
        ticks_per_sec=args.ticks_per_sec,
        frames_per_sec=args.ticks_per_sec,
        start_world=True,
    ) as env:
        state, _ = run_warmup(env, stop_command, args.warmup_ticks, agent_name="rov0")
        require_sensor(state, DVL_SENSOR_KEY)
        require_sensor(state, POSE_SENSOR_KEY)
        start_time = time.perf_counter()
        elapsed_s = 0.0
        for phase_name, lateral_command in (
            ("positive_lateral_command", command_magnitude),
            ("negative_lateral_command", -command_magnitude),
        ):
            command = bluerov2_horizontal_command(
                forward_command=0.0,
                lateral_command=lateral_command,
                max_thruster_command=command_magnitude,
            )
            steps = max(1, int(args.duration * args.ticks_per_sec))
            for _ in range(steps):
                state = env.step(command)
                elapsed_s = time.perf_counter() - start_time
                samples.append(
                    build_sample(
                        state=state,
                        time_s=elapsed_s,
                        phase=phase_name,
                        lateral_command=lateral_command,
                        command=command,
                        args=args,
                    )
                )

        env.step(stop_command)

    positive_samples = [
        sample for sample in samples if sample["phase"] == "positive_lateral_command"
    ]
    negative_samples = [
        sample for sample in samples if sample["phase"] == "negative_lateral_command"
    ]
    positive_mean_raw = mean_metric(positive_samples, "dvl_lateral_velocity_raw")
    negative_mean_raw = mean_metric(negative_samples, "dvl_lateral_velocity_raw")
    positive_mean_used = mean_metric(positive_samples, "dvl_lateral_velocity_used")
    negative_mean_used = mean_metric(negative_samples, "dvl_lateral_velocity_used")
    recommended_sign = recommended_lateral_sign(positive_mean_raw, negative_mean_raw)
    response_direction = (
        "positive"
        if positive_mean_raw > 0.0
        else "negative"
        if positive_mean_raw < 0.0
        else "inconclusive"
    )

    summary = {
        "output_dir": str(logger.output_dir),
        "lateral_command": command_magnitude,
        "duration_per_phase_s": float(args.duration),
        "dvl_lateral_index": int(args.dvl_lateral_index),
        "tested_dvl_lateral_sign": float(args.dvl_lateral_sign),
        "positive_command_mean_raw_lateral_velocity": positive_mean_raw,
        "negative_command_mean_raw_lateral_velocity": negative_mean_raw,
        "positive_command_mean_used_lateral_velocity": positive_mean_used,
        "negative_command_mean_used_lateral_velocity": negative_mean_used,
        "positive_command_response_direction": response_direction,
        "recommended_dvl_lateral_sign": recommended_sign,
        "current_x": 0.0,
        "current_y": 0.0,
        "current_z": 0.0,
        "current_magnitude": 0.0,
        "sample_count": len(samples),
        "pose_reporting_only": True,
    }
    logger.write_trajectory(samples)
    logger.write_summary(summary)

    print("\nStep 2B lateral axis check summary")
    print(f"Positive command mean raw DVL lateral velocity: {positive_mean_raw:.4f} m/s")
    print(f"Negative command mean raw DVL lateral velocity: {negative_mean_raw:.4f} m/s")
    print(f"Positive lateral command response: {response_direction}")
    print(f"Recommended --dvl-lateral-sign: {recommended_sign:+.0f}")
    print(f"Results directory: {logger.output_dir}")
    return summary


def validate_args(args: argparse.Namespace) -> None:
    if args.lateral_command == 0.0:
        raise ValueError("lateral-command must be nonzero.")
    if args.duration <= 0:
        raise ValueError("duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.dvl_lateral_index < 0:
        raise ValueError("dvl-lateral-index must be zero or positive.")


def build_sample(
    state: dict,
    time_s: float,
    phase: str,
    lateral_command: float,
    command: np.ndarray,
    args: argparse.Namespace,
) -> dict:
    dvl_sample = require_sensor(state, DVL_SENSOR_KEY)
    raw_lateral, used_lateral = dvl_velocity_components(
        dvl_sample,
        args.dvl_lateral_index,
        args.dvl_lateral_sign,
    )
    pose = pose_components(require_sensor(state, POSE_SENSOR_KEY))

    sample = {
        "time": float(time_s),
        "phase": phase,
        "lateral_command": float(lateral_command),
        "dvl_lateral_velocity_raw": raw_lateral,
        "dvl_lateral_velocity_used": used_lateral,
        "pose_x": float(pose["position"][0]),
        "pose_y": float(pose["position"][1]),
        "pose_z": float(pose["position"][2]),
    }
    command_values = np.asarray(command, dtype=float).reshape(-1)
    for index in range(8):
        sample[f"cmd_{index}"] = (
            float(command_values[index]) if index < command_values.size else 0.0
        )
    return sample


def mean_metric(samples: list[dict], key: str) -> float:
    if not samples:
        return 0.0
    return float(np.mean([float(sample[key]) for sample in samples]))


def recommended_lateral_sign(positive_mean_raw: float, negative_mean_raw: float) -> float:
    if abs(positive_mean_raw) < 1e-6 and abs(negative_mean_raw) < 1e-6:
        return 1.0
    return 1.0 if positive_mean_raw >= 0.0 else -1.0


if __name__ == "__main__":
    main()
