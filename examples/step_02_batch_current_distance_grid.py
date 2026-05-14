from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
EXAMPLES_DIR = PROJECT_ROOT / "examples"
for path in (SRC_DIR, EXAMPLES_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from lib.current import CurrentConfig, unique_current_cases
from lib.worlds import World
from experiments.forward_distance import DEFAULT_FORWARD_COMMAND, run_forward_distance_experiment
from visualization.current_plots import (
    plot_duration_vs_target_and_current,
    plot_forward_vs_euclidean,
    plot_metric_vs_current,
    plot_metric_vs_target,
)


CSV_FIELDNAMES = [
    "run_id",
    "target_distance",
    "repetition_index",
    "current_x",
    "current_y",
    "current_z",
    "current_magnitude",
    "forward_command",
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
    "mean_dt",
    "std_dt",
    "dt_source",
    "average_pose_speed",
    "max_dvl_forward_velocity",
    "mean_dvl_forward_velocity",
]


def main() -> None:
    args = parse_args()
    run_batch(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2 batch grid: target distances crossed with ocean-current cases."
    )
    parser.add_argument("--target-distances", type=float, nargs="+", default=[5.0, 10.0, 20.0])
    parser.add_argument(
        "--current-y-values",
        type=float,
        nargs="+",
        default=[0.0, 0.25, 0.5, 1.0, 2.0],
    )
    parser.add_argument(
        "--current-x-values",
        type=float,
        nargs="*",
        default=[],
        help="Optional current-x values to combine with every current-y value.",
    )
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument(
        "--include-frontal-currents",
        action="store_true",
        help=(
            "Include frontal/following current cases. These are enabled by default; "
            "use --no-frontal-currents to disable them."
        ),
    )
    parser.add_argument(
        "--no-frontal-currents",
        action="store_false",
        dest="include_frontal_currents",
        help="Disable the default frontal/following current cases.",
    )
    parser.add_argument(
        "--frontal-current-values",
        type=float,
        nargs="+",
        default=[-0.5, 0.5],
    )
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--forward-command", type=float, default=DEFAULT_FORWARD_COMMAND)
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
        default=PROJECT_ROOT / "results" / "step_02_current_distance_grid",
    )
    parser.add_argument(
        "--resume-dir",
        type=Path,
        default=None,
        help="Resume an existing timestamped batch directory and skip completed runs.",
    )
    parser.add_argument(
        "--retry-count",
        type=int,
        default=2,
        help="Number of retries for a failed HoloOcean run before aborting.",
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=5.0,
        help="Seconds to wait before retrying a failed HoloOcean run.",
    )
    parser.add_argument(
        "--run-settle-delay",
        type=float,
        default=1.0,
        help="Seconds to wait between runs so the simulator can release resources.",
    )
    parser.set_defaults(show_viewport=True, include_frontal_currents=True)
    return parser.parse_args()


def run_batch(args: argparse.Namespace) -> None:
    validate_args(args)

    batch_dir = args.resume_dir if args.resume_dir is not None else (
        args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    batch_dir.mkdir(parents=True, exist_ok=True)
    current_cases = build_current_cases(args)
    expected_runs = build_expected_runs(args, current_cases)

    print(f"Saving Step 2 current-distance grid outputs to: {batch_dir}")
    print(f"Viewport enabled: {args.show_viewport}")
    print(f"Current cases: {len(current_cases)}")
    if args.resume_dir is not None:
        print("Resume mode: completed runs with summary.json will be skipped.")

    rows = load_completed_rows(batch_dir, expected_runs)
    completed_run_ids = {row["run_id"] for row in rows}
    if rows:
        print(f"Loaded completed runs: {len(rows)}")
        write_batch_outputs(
            args,
            batch_dir,
            current_cases,
            rows,
            expected_run_count=len(expected_runs),
            write_plots=False,
        )

    total_runs = len(expected_runs)
    run_number = 0

    for expected_run in expected_runs:
        run_number += 1
        run_id = expected_run["run_id"]
        target_distance = expected_run["target_distance"]
        current = expected_run["current"]
        repetition_index = expected_run["repetition_index"]
        run_dir = batch_dir / run_id
        if run_id in completed_run_ids:
            print(f"[{run_number}/{total_runs}] skipping completed run_id={run_id}")
            continue

        print(
            f"[{run_number}/{total_runs}] target={target_distance:g} m, "
            f"current=[{current.x:g}, {current.y:g}, {current.z:g}] m/s, "
            f"repetition={repetition_index}, run_id={run_id}"
        )

        run_args = argparse.Namespace(
            target_distance=target_distance,
            forward_command=args.forward_command,
            current_x=current.x,
            current_y=current.y,
            current_z=current.z,
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
            scenario_name="Step02_Current_Distance_Grid",
            experiment_label="Step 2 current-distance grid",
            summary_title="Step 2 current-distance grid run summary",
            world=args.world,
            results_dir=args.results_dir,
            show_viewport=args.show_viewport,
        )
        summary = run_forward_distance_with_retries(args, run_args, run_dir)
        row = summary_to_row(summary, run_id, target_distance, repetition_index)
        rows.append(row)
        completed_run_ids.add(run_id)
        write_batch_outputs(
            args,
            batch_dir,
            current_cases,
            rows,
            expected_run_count=len(expected_runs),
            write_plots=False,
        )

        if args.run_settle_delay > 0:
            time.sleep(args.run_settle_delay)

    write_batch_outputs(
        args,
        batch_dir,
        current_cases,
        rows,
        expected_run_count=len(expected_runs),
        write_plots=True,
    )

    print("\nStep 2 current-distance grid summary")
    print(f"Total runs: {len(rows)}")
    print(f"Successful runs: {sum(row['stop_reason'] == 'target_reached' for row in rows)}")
    print(f"Timeout runs: {sum(row['stop_reason'] == 'timeout' for row in rows)}")
    print(f"Results directory: {batch_dir}")


def validate_args(args: argparse.Namespace) -> None:
    if not args.target_distances:
        raise ValueError("At least one target distance is required.")
    if any(distance <= 0 for distance in args.target_distances):
        raise ValueError("All target distances must be positive.")
    if not args.current_y_values:
        raise ValueError("At least one current-y value is required.")
    if args.repetitions <= 0:
        raise ValueError("repetitions must be positive.")
    if args.forward_command < 0:
        raise ValueError("forward-command must be zero or positive.")
    if args.max_duration <= 0:
        raise ValueError("max-duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.dvl_forward_index < 0:
        raise ValueError("dvl-forward-index must be zero or positive.")
    if args.speed_warning_threshold <= 0:
        raise ValueError("speed-warning-threshold must be positive.")
    if args.max_dvl_speed_warning_threshold <= 0:
        raise ValueError("max-dvl-speed-warning-threshold must be positive.")
    if args.retry_count < 0:
        raise ValueError("retry-count must be zero or positive.")
    if args.retry_delay < 0:
        raise ValueError("retry-delay must be zero or positive.")
    if args.run_settle_delay < 0:
        raise ValueError("run-settle-delay must be zero or positive.")


def build_current_cases(args: argparse.Namespace) -> list[CurrentConfig]:
    x_values = args.current_x_values if args.current_x_values else [0.0]
    cases = [
        CurrentConfig(x=current_x, y=current_y, z=args.current_z)
        for current_x in x_values
        for current_y in args.current_y_values
    ]
    if args.include_frontal_currents:
        cases.extend(
            CurrentConfig(x=current_x, y=0.0, z=args.current_z)
            for current_x in args.frontal_current_values
        )
    return unique_current_cases(cases)


def build_expected_runs(args: argparse.Namespace, current_cases: list[CurrentConfig]) -> list[dict]:
    expected_runs = []
    for target_distance in args.target_distances:
        for current in current_cases:
            for repetition_index in range(1, args.repetitions + 1):
                run_id = make_run_id(target_distance, current, repetition_index)
                expected_runs.append(
                    {
                        "run_id": run_id,
                        "target_distance": float(target_distance),
                        "current": current,
                        "repetition_index": int(repetition_index),
                    }
                )
    return expected_runs


def load_completed_rows(batch_dir: Path, expected_runs: list[dict]) -> list[dict]:
    rows = []
    for expected_run in expected_runs:
        summary_path = batch_dir / expected_run["run_id"] / "summary.json"
        if not summary_path.exists():
            continue
        with summary_path.open("r", encoding="utf-8") as file:
            summary = json.load(file)
        rows.append(
            summary_to_row(
                summary,
                expected_run["run_id"],
                expected_run["target_distance"],
                expected_run["repetition_index"],
            )
        )
    return rows


def run_forward_distance_with_retries(
    args: argparse.Namespace,
    run_args: argparse.Namespace,
    run_dir: Path,
) -> dict:
    attempts = args.retry_count + 1
    for attempt_index in range(1, attempts + 1):
        try:
            return run_forward_distance_experiment(
                run_args,
                output_dir=run_dir,
                print_terminal_summary=False,
            )
        except Exception as exc:
            if attempt_index >= attempts:
                raise
            print(
                "Warning: HoloOcean run failed during attempt "
                f"{attempt_index}/{attempts}: {exc}"
            )
            if args.retry_delay > 0:
                print(f"Waiting {args.retry_delay:g} s before retrying.")
                time.sleep(args.retry_delay)

    raise RuntimeError("unreachable retry state")


def make_run_id(
    target_distance: float,
    current: CurrentConfig,
    repetition_index: int,
) -> str:
    return (
        f"target_{format_float(target_distance)}m_"
        f"current_x_{format_float(current.x)}_"
        f"y_{format_float(current.y)}_"
        f"z_{format_float(current.z)}_"
        f"rep_{repetition_index:02d}"
    )


def format_float(value: float) -> str:
    return f"{value:g}".replace("-", "neg").replace(".", "p")


def summary_to_row(
    summary: dict,
    run_id: str,
    target_distance: float,
    repetition_index: int,
) -> dict:
    return {
        "run_id": run_id,
        "target_distance": float(target_distance),
        "repetition_index": int(repetition_index),
        "current_x": float(summary["current_x"]),
        "current_y": float(summary["current_y"]),
        "current_z": float(summary["current_z"]),
        "current_magnitude": float(summary["current_magnitude"]),
        "forward_command": float(summary["forward_command"]),
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
        "mean_dt": float(summary["mean_dt"]),
        "std_dt": float(summary["std_dt"]),
        "dt_source": summary["dt_source"],
        "average_pose_speed": float(summary["average_pose_speed"]),
        "max_dvl_forward_velocity": float(summary["max_dvl_forward_velocity"]),
        "mean_dvl_forward_velocity": float(summary["mean_dvl_forward_velocity"]),
    }


def write_all_runs_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def write_batch_outputs(
    args: argparse.Namespace,
    batch_dir: Path,
    current_cases: list[CurrentConfig],
    rows: list[dict],
    expected_run_count: int,
    write_plots: bool,
) -> None:
    write_all_runs_csv(batch_dir / "all_runs_summary.csv", rows)
    aggregate_summary = build_aggregate_summary(
        args,
        batch_dir,
        current_cases,
        rows,
        expected_run_count=expected_run_count,
    )
    write_json(batch_dir / "aggregate_summary.json", aggregate_summary)

    if not write_plots or not rows:
        return

    plot_metric_vs_current(
        rows,
        "lateral_drift",
        "Lateral drift [m]",
        batch_dir / "lateral_drift_vs_current.png",
    )
    plot_metric_vs_current(
        rows,
        "final_position_error",
        "Final position error [m]",
        batch_dir / "final_error_vs_current.png",
    )
    plot_metric_vs_target(
        rows,
        "absolute_error",
        "Absolute DVL/Pose error [m]",
        batch_dir / "distance_error_vs_target.png",
    )
    plot_metric_vs_target(
        rows,
        "lateral_drift",
        "Lateral drift [m]",
        batch_dir / "lateral_drift_vs_target.png",
    )
    plot_duration_vs_target_and_current(rows, batch_dir / "duration_vs_target_and_current.png")
    plot_forward_vs_euclidean(rows, batch_dir / "forward_vs_euclidean_distance.png")


def build_aggregate_summary(
    args: argparse.Namespace,
    batch_dir: Path,
    current_cases: list[CurrentConfig],
    rows: list[dict],
    expected_run_count: int,
) -> dict:
    groups = []
    grouped_rows: dict[tuple[float, float, float, float], list[dict]] = defaultdict(list)
    for row in rows:
        key = (
            row["target_distance"],
            row["current_x"],
            row["current_y"],
            row["current_z"],
        )
        grouped_rows[key].append(row)

    for key in sorted(grouped_rows):
        target_distance, current_x, current_y, current_z = key
        group_rows = grouped_rows[key]
        groups.append(
            {
                "target_distance": float(target_distance),
                "current_x": float(current_x),
                "current_y": float(current_y),
                "current_z": float(current_z),
                "number_of_runs": len(group_rows),
                "successful_runs": sum(
                    row["stop_reason"] == "target_reached" for row in group_rows
                ),
                "timeout_runs": sum(row["stop_reason"] == "timeout" for row in group_rows),
                "mean_absolute_error": mean_metric(group_rows, "absolute_error"),
                "std_absolute_error": std_metric(group_rows, "absolute_error"),
                "mean_percentage_error": mean_metric(group_rows, "percentage_error"),
                "std_percentage_error": std_metric(group_rows, "percentage_error"),
                "mean_lateral_drift": mean_metric(group_rows, "lateral_drift"),
                "std_lateral_drift": std_metric(group_rows, "lateral_drift"),
                "mean_final_position_error": mean_metric(
                    group_rows, "final_position_error"
                ),
                "std_final_position_error": std_metric(group_rows, "final_position_error"),
                "mean_duration": mean_metric(group_rows, "duration"),
                "std_duration": std_metric(group_rows, "duration"),
                "mean_euclidean_displacement": mean_metric(
                    group_rows, "pose_euclidean_displacement"
                ),
                "std_euclidean_displacement": std_metric(
                    group_rows, "pose_euclidean_displacement"
                ),
            }
        )

    return {
        "output_dir": str(batch_dir),
        "target_distances": [float(value) for value in args.target_distances],
        "current_cases": [current.to_dict() for current in current_cases],
        "expected_runs": int(expected_run_count),
        "completed_runs": len(rows),
        "complete": len(rows) == expected_run_count,
        "repetitions": int(args.repetitions),
        "max_duration_s": float(args.max_duration),
        "forward_command": float(args.forward_command),
        "show_viewport": bool(args.show_viewport),
        "dvl_forward_index": int(args.dvl_forward_index),
        "dvl_forward_sign": float(args.dvl_forward_sign),
        "current_application_mode": "every_step",
        "current_api_method": "env.set_ocean_currents(agent_name, velocity)",
        "groups": groups,
    }


def mean_metric(rows: list[dict], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([row[key] for row in rows]))


def std_metric(rows: list[dict], key: str) -> float:
    if len(rows) < 2:
        return 0.0
    return float(np.std([row[key] for row in rows], ddof=0))


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


if __name__ == "__main__":
    main()
