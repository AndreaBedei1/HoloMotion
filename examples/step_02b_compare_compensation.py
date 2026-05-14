from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np


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
    DEFAULT_MAX_THRUSTER_COMMAND,
    run_dvl_velocity_compensation_experiment,
)
from experiments.forward_distance import DEFAULT_FORWARD_COMMAND, run_forward_distance_experiment
from lib.worlds import World
from visualization.velocity_tracking_plots import (
    plot_mode_metric_comparison,
    plot_velocity_error_summary,
)


CSV_FIELDNAMES = [
    "run_id",
    "mode",
    "target_distance",
    "repetition_index",
    "current_x",
    "current_y",
    "current_z",
    "current_magnitude",
    "forward_command",
    "desired_forward_velocity",
    "desired_lateral_velocity",
    "dvl_estimated_distance",
    "pose_forward_displacement",
    "pose_euclidean_displacement",
    "absolute_error",
    "percentage_error",
    "lateral_drift",
    "final_position_error",
    "duration",
    "sample_count",
    "stop_reason",
    "mean_forward_velocity_error",
    "std_forward_velocity_error",
    "mean_lateral_velocity_error",
    "std_lateral_velocity_error",
    "mean_abs_lateral_velocity_error",
    "max_abs_lateral_velocity_error",
    "mean_command_effort",
    "max_command_effort",
    "mean_dt",
    "std_dt",
    "dt_source",
]


def main() -> None:
    args = parse_args()
    run_comparison(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2B comparison between no compensation and DVL velocity compensation."
    )
    parser.add_argument("--target-distances", type=float, nargs="+", default=[5.0, 10.0, 20.0])
    parser.add_argument("--current-y-values", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--forward-command", type=float, default=DEFAULT_FORWARD_COMMAND)
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
    parser.add_argument("--kp-forward", type=float, default=DEFAULT_FORWARD_KP)
    parser.add_argument("--kp-lateral", type=float, default=DEFAULT_LATERAL_KP)
    parser.add_argument("--max-forward-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-lateral-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument(
        "--max-thruster-command",
        type=float,
        default=DEFAULT_MAX_THRUSTER_COMMAND,
        help="Final absolute limit applied to each Step 2B thruster command after mixing.",
    )
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
        default=PROJECT_ROOT / "results" / "step_02b_compensation_comparison",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_comparison(args: argparse.Namespace) -> None:
    validate_args(args)

    batch_dir = args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving Step 2B compensation comparison outputs to: {batch_dir}")
    print(f"Viewport enabled: {args.show_viewport}")

    rows: list[dict] = []
    total_pairs = len(args.target_distances) * len(args.current_y_values) * args.repetitions
    pair_number = 0

    for target_distance in args.target_distances:
        for current_y in args.current_y_values:
            for repetition_index in range(1, args.repetitions + 1):
                pair_number += 1
                base_run_id = make_run_id(target_distance, current_y, repetition_index)
                print(
                    f"[{pair_number}/{total_pairs}] target={target_distance:g} m, "
                    f"current_y={current_y:g} m/s, repetition={repetition_index}"
                )

                no_comp_args = argparse.Namespace(
                    target_distance=target_distance,
                    forward_command=args.forward_command,
                    current_x=args.current_x,
                    current_y=current_y,
                    current_z=args.current_z,
                    ticks_per_sec=args.ticks_per_sec,
                    max_duration=args.max_duration,
                    warmup_ticks=args.warmup_ticks,
                    dvl_forward_index=args.dvl_forward_index,
                    dvl_forward_sign=args.dvl_forward_sign,
                    speed_warning_threshold=args.speed_warning_threshold,
                    max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
                    diagnostic_distance_check=False,
                    make_current_plots=True,
                    current_api_method="env.set_ocean_currents(agent_name, velocity)",
                    scenario_name="Step02B_Comparison_No_Compensation",
                    experiment_label="Step 2B comparison no-compensation",
                    summary_title="Step 2B comparison no-compensation summary",
                    world=args.world,
                    results_dir=args.results_dir,
                    show_viewport=args.show_viewport,
                )
                no_comp_summary = run_forward_distance_experiment(
                    no_comp_args,
                    output_dir=batch_dir / f"{base_run_id}_no_compensation",
                    print_terminal_summary=False,
                )
                rows.append(
                    summary_to_row(
                        no_comp_summary,
                        run_id=base_run_id,
                        mode="no_compensation",
                        target_distance=target_distance,
                        repetition_index=repetition_index,
                        args=args,
                    )
                )

                comp_args = argparse.Namespace(
                    target_distance=target_distance,
                    desired_forward_velocity=args.desired_forward_velocity,
                    desired_lateral_velocity=args.desired_lateral_velocity,
                    current_x=args.current_x,
                    current_y=current_y,
                    current_z=args.current_z,
                    kp_forward=args.kp_forward,
                    kp_lateral=args.kp_lateral,
                    max_forward_command=args.max_forward_command,
                    max_lateral_command=args.max_lateral_command,
                    max_thruster_command=args.max_thruster_command,
                    base_vertical_command=args.base_vertical_command,
                    ticks_per_sec=args.ticks_per_sec,
                    max_duration=args.max_duration,
                    warmup_ticks=args.warmup_ticks,
                    dvl_forward_index=args.dvl_forward_index,
                    dvl_forward_sign=args.dvl_forward_sign,
                    dvl_lateral_index=args.dvl_lateral_index,
                    dvl_lateral_sign=args.dvl_lateral_sign,
                    speed_warning_threshold=args.speed_warning_threshold,
                    max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
                    world=args.world,
                    results_dir=args.results_dir,
                    show_viewport=args.show_viewport,
                )
                comp_summary = run_dvl_velocity_compensation_experiment(
                    comp_args,
                    output_dir=batch_dir / f"{base_run_id}_dvl_velocity_compensation",
                    print_terminal_summary=False,
                )
                rows.append(
                    summary_to_row(
                        comp_summary,
                        run_id=base_run_id,
                        mode="dvl_velocity_compensation",
                        target_distance=target_distance,
                        repetition_index=repetition_index,
                        args=args,
                    )
                )

    write_all_runs_csv(batch_dir / "all_runs_summary.csv", rows)
    aggregate_summary = build_aggregate_summary(args, batch_dir, rows)
    write_json(batch_dir / "aggregate_summary.json", aggregate_summary)
    plot_mode_metric_comparison(
        rows,
        "lateral_drift",
        "Lateral drift [m]",
        batch_dir / "lateral_drift_comparison.png",
    )
    plot_mode_metric_comparison(
        rows,
        "final_position_error",
        "Final position error [m]",
        batch_dir / "final_position_error_comparison.png",
    )
    plot_mode_metric_comparison(
        rows,
        "duration",
        "Duration [s]",
        batch_dir / "duration_comparison.png",
    )
    plot_velocity_error_summary(rows, batch_dir / "velocity_error_summary.png")

    print("\nStep 2B compensation comparison summary")
    print(f"Total runs: {len(rows)}")
    print(f"Results directory: {batch_dir}")
    for group in aggregate_summary["groups"]:
        print(
            f"target={group['target_distance']:g} m, current_y={group['current_y']:g} m/s: "
            f"lateral drift reduction={group['lateral_drift_reduction_percentage']:.1f}%"
        )


def validate_args(args: argparse.Namespace) -> None:
    if not args.target_distances:
        raise ValueError("At least one target distance is required.")
    if any(distance <= 0 for distance in args.target_distances):
        raise ValueError("All target distances must be positive.")
    if not args.current_y_values:
        raise ValueError("At least one current-y value is required.")
    if args.repetitions <= 0:
        raise ValueError("repetitions must be positive.")
    if args.max_duration <= 0:
        raise ValueError("max-duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.max_forward_command < 0 or args.max_lateral_command < 0:
        raise ValueError("max command values must be non-negative.")
    if args.max_thruster_command < 0:
        raise ValueError("max-thruster-command must be non-negative.")


def make_run_id(target_distance: float, current_y: float, repetition_index: int) -> str:
    return (
        f"target_{format_float(target_distance)}m_"
        f"current_y_{format_float(current_y)}_"
        f"rep_{repetition_index:02d}"
    )


def format_float(value: float) -> str:
    return f"{value:g}".replace("-", "neg").replace(".", "p")


def summary_to_row(
    summary: dict,
    run_id: str,
    mode: str,
    target_distance: float,
    repetition_index: int,
    args: argparse.Namespace,
) -> dict:
    return {
        "run_id": run_id,
        "mode": mode,
        "target_distance": float(target_distance),
        "repetition_index": int(repetition_index),
        "current_x": float(summary["current_x"]),
        "current_y": float(summary["current_y"]),
        "current_z": float(summary["current_z"]),
        "current_magnitude": float(summary["current_magnitude"]),
        "forward_command": float(getattr(args, "forward_command", 0.0)),
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "dvl_estimated_distance": float(summary["dvl_estimated_distance"]),
        "pose_forward_displacement": float(summary["pose_forward_displacement"]),
        "pose_euclidean_displacement": float(summary["pose_euclidean_displacement"]),
        "absolute_error": float(summary["absolute_error"]),
        "percentage_error": float(summary["percentage_error"]),
        "lateral_drift": float(summary["lateral_drift"]),
        "final_position_error": float(summary["final_position_error"]),
        "duration": float(summary["duration"]),
        "sample_count": int(summary["sample_count"]),
        "stop_reason": summary["stop_reason"],
        "mean_forward_velocity_error": summary.get("mean_forward_velocity_error", ""),
        "std_forward_velocity_error": summary.get("std_forward_velocity_error", ""),
        "mean_lateral_velocity_error": summary.get("mean_lateral_velocity_error", ""),
        "std_lateral_velocity_error": summary.get("std_lateral_velocity_error", ""),
        "mean_abs_lateral_velocity_error": summary.get(
            "mean_abs_lateral_velocity_error",
            "",
        ),
        "max_abs_lateral_velocity_error": summary.get(
            "max_abs_lateral_velocity_error",
            "",
        ),
        "mean_command_effort": summary.get("mean_command_effort", ""),
        "max_command_effort": summary.get("max_command_effort", ""),
        "mean_dt": float(summary["mean_dt"]),
        "std_dt": float(summary["std_dt"]),
        "dt_source": summary["dt_source"],
    }


def write_all_runs_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_aggregate_summary(args: argparse.Namespace, batch_dir: Path, rows: list[dict]) -> dict:
    groups = []
    grouped_rows: dict[tuple[float, float], list[dict]] = defaultdict(list)
    for row in rows:
        grouped_rows[(row["target_distance"], row["current_y"])].append(row)

    for key in sorted(grouped_rows):
        target_distance, current_y = key
        group_rows = grouped_rows[key]
        no_comp_rows = [row for row in group_rows if row["mode"] == "no_compensation"]
        comp_rows = [
            row for row in group_rows if row["mode"] == "dvl_velocity_compensation"
        ]
        no_comp_lateral = mean_metric(no_comp_rows, "lateral_drift")
        comp_lateral = mean_metric(comp_rows, "lateral_drift")
        no_comp_final = mean_metric(no_comp_rows, "final_position_error")
        comp_final = mean_metric(comp_rows, "final_position_error")
        no_comp_duration = mean_metric(no_comp_rows, "duration")
        comp_duration = mean_metric(comp_rows, "duration")
        no_comp_error = mean_metric(no_comp_rows, "absolute_error")
        comp_error = mean_metric(comp_rows, "absolute_error")
        groups.append(
            {
                "target_distance": float(target_distance),
                "current_y": float(current_y),
                "number_of_pairs": min(len(no_comp_rows), len(comp_rows)),
                "no_compensation_mean_lateral_drift": no_comp_lateral,
                "compensation_mean_lateral_drift": comp_lateral,
                "lateral_drift_reduction_percentage": reduction_percentage(
                    no_comp_lateral,
                    comp_lateral,
                ),
                "no_compensation_mean_final_position_error": no_comp_final,
                "compensation_mean_final_position_error": comp_final,
                "final_position_error_reduction_percentage": reduction_percentage(
                    no_comp_final,
                    comp_final,
                ),
                "no_compensation_mean_duration": no_comp_duration,
                "compensation_mean_duration": comp_duration,
                "duration_difference_s": comp_duration - no_comp_duration,
                "no_compensation_mean_absolute_error": no_comp_error,
                "compensation_mean_absolute_error": comp_error,
                "absolute_error_difference_m": comp_error - no_comp_error,
                "compensation_mean_abs_lateral_velocity_error": mean_optional_metric(
                    comp_rows,
                    "mean_abs_lateral_velocity_error",
                ),
            }
        )

    return {
        "output_dir": str(batch_dir),
        "target_distances": [float(value) for value in args.target_distances],
        "current_y_values": [float(value) for value in args.current_y_values],
        "repetitions": int(args.repetitions),
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "modes": ["no_compensation", "dvl_velocity_compensation"],
        "groups": groups,
    }


def mean_metric(rows: list[dict], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([float(row[key]) for row in rows]))


def mean_optional_metric(rows: list[dict], key: str) -> float:
    values = [float(row[key]) for row in rows if row.get(key, "") != ""]
    if not values:
        return 0.0
    return float(np.mean(values))


def reduction_percentage(baseline: float, improved: float) -> float:
    if abs(baseline) < 1e-9:
        return 0.0
    return float(100.0 * (baseline - improved) / abs(baseline))


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


if __name__ == "__main__":
    main()
