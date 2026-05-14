from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


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


CSV_FIELDNAMES = [
    "run_id",
    "target_distance",
    "repetition_index",
    "forward_command",
    "dvl_estimated_distance",
    "pose_ground_truth_displacement",
    "absolute_error",
    "percentage_error",
    "lateral_drift",
    "duration",
    "sample_count",
    "stop_reason",
    "dvl_forward_index",
    "dvl_forward_sign",
    "dt_source",
    "mean_dt",
    "std_dt",
]


def main() -> None:
    args = parse_args()
    run_batch(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 1.1: batch baseline validation for forward-distance runs."
    )
    parser.add_argument("--target-distances", type=float, nargs="+", default=[2.0, 5.0, 10.0, 20.0])
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--forward-command", type=float, default=DEFAULT_FORWARD_COMMAND)
    parser.add_argument("--speed-warning-threshold", type=float, default=1.0)
    parser.add_argument("--max-dvl-speed-warning-threshold", type=float, default=1.5)
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
        default=PROJECT_ROOT / "results" / "step_01_forward_distance_batch",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_batch(args: argparse.Namespace) -> None:
    validate_args(args)

    batch_dir = args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir.mkdir(parents=True, exist_ok=True)

    print(f"Saving Step 1.1 batch outputs to: {batch_dir}")
    print(f"Viewport enabled: {args.show_viewport}")

    rows: list[dict] = []
    total_runs = len(args.target_distances) * args.repetitions
    run_number = 0

    for target_distance in args.target_distances:
        for repetition_index in range(1, args.repetitions + 1):
            run_number += 1
            run_id = make_run_id(target_distance, repetition_index)
            run_dir = batch_dir / run_id
            print(
                f"[{run_number}/{total_runs}] target={target_distance:g} m, "
                f"repetition={repetition_index}, run_id={run_id}"
            )

            run_args = argparse.Namespace(
                target_distance=target_distance,
                forward_command=args.forward_command,
                ticks_per_sec=args.ticks_per_sec,
                max_duration=args.max_duration,
                warmup_ticks=args.warmup_ticks,
                dvl_forward_index=args.dvl_forward_index,
                dvl_forward_sign=args.dvl_forward_sign,
                speed_warning_threshold=args.speed_warning_threshold,
                max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
                diagnostic_distance_check=False,
                world=args.world,
                results_dir=args.results_dir,
                show_viewport=args.show_viewport,
            )
            summary = run_forward_distance_experiment(
                run_args,
                output_dir=run_dir,
                print_terminal_summary=False,
            )
            rows.append(summary_to_row(summary, run_id, target_distance, repetition_index))

    write_all_runs_csv(batch_dir / "all_runs_summary.csv", rows)
    aggregate_summary = build_aggregate_summary(args, batch_dir, rows)
    write_json(batch_dir / "aggregate_summary.json", aggregate_summary)
    plot_metric_vs_target(
        rows,
        "absolute_error",
        "Absolute distance error [m]",
        batch_dir / "distance_error_vs_target.png",
    )
    plot_metric_vs_target(
        rows,
        "percentage_error",
        "Percentage error [%]",
        batch_dir / "percentage_error_vs_target.png",
    )
    plot_metric_vs_target(
        rows,
        "lateral_drift",
        "Lateral drift [m]",
        batch_dir / "lateral_drift_vs_target.png",
    )
    plot_metric_vs_target(
        rows,
        "duration",
        "Duration [s]",
        batch_dir / "duration_vs_target.png",
    )

    print("\nStep 1.1 batch summary")
    print(f"Total runs: {len(rows)}")
    print(f"Successful runs: {sum(row['stop_reason'] == 'target_reached' for row in rows)}")
    print(f"Timeout runs: {sum(row['stop_reason'] == 'timeout' for row in rows)}")
    print(f"Results directory: {batch_dir}")


def validate_args(args: argparse.Namespace) -> None:
    if not args.target_distances:
        raise ValueError("At least one target distance is required.")
    if any(distance <= 0 for distance in args.target_distances):
        raise ValueError("All target distances must be positive.")
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


def make_run_id(target_distance: float, repetition_index: int) -> str:
    target_text = str(target_distance).replace(".", "p")
    return f"target_{target_text}m_rep_{repetition_index:02d}"


def summary_to_row(
    summary: dict,
    run_id: str,
    target_distance: float,
    repetition_index: int,
) -> dict:
    return {
        "run_id": run_id,
        "target_distance": float(target_distance),
        "repetition_index": repetition_index,
        "forward_command": float(summary["forward_command"]),
        "dvl_estimated_distance": float(summary["dvl_estimated_distance_m"]),
        "pose_ground_truth_displacement": float(summary["pose_ground_truth_displacement_m"]),
        "absolute_error": float(summary["absolute_distance_error_m"]),
        "percentage_error": float(summary["percentage_error"]),
        "lateral_drift": float(summary["lateral_drift_m"]),
        "duration": float(summary["duration_s"]),
        "sample_count": int(summary["num_samples"]),
        "stop_reason": summary["stop_reason"],
        "dvl_forward_index": int(summary["dvl_forward_index"]),
        "dvl_forward_sign": float(summary["dvl_forward_sign"]),
        "dt_source": summary["dt_source"],
        "mean_dt": float(summary["mean_dt"]),
        "std_dt": float(summary["std_dt"]),
    }


def write_all_runs_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_aggregate_summary(args: argparse.Namespace, batch_dir: Path, rows: list[dict]) -> dict:
    targets = {}
    for target_distance in sorted({row["target_distance"] for row in rows}):
        target_rows = [row for row in rows if row["target_distance"] == target_distance]
        targets[str(target_distance)] = {
            "successful_runs": sum(row["stop_reason"] == "target_reached" for row in target_rows),
            "timeout_runs": sum(row["stop_reason"] == "timeout" for row in target_rows),
            "mean_absolute_error": mean_metric(target_rows, "absolute_error"),
            "std_absolute_error": std_metric(target_rows, "absolute_error"),
            "mean_percentage_error": mean_metric(target_rows, "percentage_error"),
            "std_percentage_error": std_metric(target_rows, "percentage_error"),
            "mean_lateral_drift": mean_metric(target_rows, "lateral_drift"),
            "std_lateral_drift": std_metric(target_rows, "lateral_drift"),
            "mean_duration": mean_metric(target_rows, "duration"),
            "std_duration": std_metric(target_rows, "duration"),
        }

    return {
        "output_dir": str(batch_dir),
        "target_distances": [float(value) for value in args.target_distances],
        "repetitions": int(args.repetitions),
        "max_duration_s": float(args.max_duration),
        "forward_command": float(args.forward_command),
        "show_viewport": bool(args.show_viewport),
        "dvl_forward_index": int(args.dvl_forward_index),
        "dvl_forward_sign": float(args.dvl_forward_sign),
        "targets": targets,
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


def plot_metric_vs_target(rows: list[dict], metric_key: str, ylabel: str, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    targets = sorted({row["target_distance"] for row in rows})
    means = []

    for target in targets:
        values = [row[metric_key] for row in rows if row["target_distance"] == target]
        means.append(float(np.mean(values)))
        ax.scatter([target] * len(values), values, alpha=0.7)

    ax.plot(targets, means, color="black", marker="o", linewidth=1.5, label="Mean")
    ax.set_xlabel("Target distance [m]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
