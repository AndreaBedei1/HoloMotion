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
from experiments.forward_distance import DEFAULT_FORWARD_COMMAND, run_forward_distance_experiment
from step_02_strong_current_forward_distance_live import CURRENT_WARNING_THRESHOLD_MPS


CSV_FIELDNAMES = [
    "run_id",
    "current_x",
    "current_y",
    "current_z",
    "current_magnitude",
    "target_distance",
    "repetition_index",
    "forward_command",
    "dvl_estimated_distance",
    "pose_forward_displacement",
    "pose_euclidean_displacement",
    "absolute_error",
    "percentage_error",
    "lateral_drift",
    "duration",
    "sample_count",
    "stop_reason",
    "mean_dt",
    "std_dt",
    "dt_source",
    "max_dvl_forward_velocity",
    "mean_dvl_forward_velocity",
    "average_pose_speed",
]


def main() -> None:
    args = parse_args()
    run_batch(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 2.1: batch forward-distance validation under lateral currents."
    )
    parser.add_argument("--target-distance", type=float, default=5.0)
    parser.add_argument(
        "--current-y-values",
        type=float,
        nargs="+",
        default=[0.0, 0.25, 0.5, 1.0, 2.0, 5.0],
    )
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-z", type=float, default=0.0)
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
        default=PROJECT_ROOT / "results" / "step_02_current_batch",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_batch(args: argparse.Namespace) -> None:
    validate_args(args)

    batch_dir = args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir.mkdir(parents=True, exist_ok=True)

    print(f"Saving Step 2.1 batch outputs to: {batch_dir}")
    print(f"Viewport enabled: {args.show_viewport}")

    rows: list[dict] = []
    total_runs = len(args.current_y_values) * args.repetitions
    run_number = 0

    for current_y in args.current_y_values:
        current_vector = np.array([args.current_x, current_y, args.current_z], dtype=float)
        current_magnitude = float(np.linalg.norm(current_vector))
        if current_magnitude > CURRENT_WARNING_THRESHOLD_MPS:
            print(
                "Warning: this current is intentionally extreme and may be unrealistic "
                "or destabilize the vehicle."
            )

        for repetition_index in range(1, args.repetitions + 1):
            run_number += 1
            run_id = make_run_id(current_y, repetition_index)
            run_dir = batch_dir / run_id
            print(
                f"[{run_number}/{total_runs}] current_y={current_y:g} m/s, "
                f"repetition={repetition_index}, run_id={run_id}"
            )

            run_args = argparse.Namespace(
                target_distance=args.target_distance,
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
                scenario_name="Step02_Current_Batch",
                experiment_label="Step 2.1 current batch",
                summary_title="Step 2.1 current batch run summary",
                world=args.world,
                results_dir=args.results_dir,
                show_viewport=args.show_viewport,
            )
            summary = run_forward_distance_experiment(
                run_args,
                output_dir=run_dir,
                print_terminal_summary=False,
            )
            row = summary_to_row(summary, run_id, args.target_distance, repetition_index)
            rows.append(row)
            if row["average_pose_speed"] > args.speed_warning_threshold:
                print(
                    "Warning: average Pose speed exceeds the configured warning "
                    f"threshold for {run_id}."
                )

    write_all_runs_csv(batch_dir / "all_runs_summary.csv", rows)
    aggregate_summary = build_aggregate_summary(args, batch_dir, rows)
    write_json(batch_dir / "aggregate_summary.json", aggregate_summary)
    plot_metric_vs_current(rows, "lateral_drift", "Lateral drift [m]", batch_dir / "lateral_drift_vs_current.png")
    plot_metric_vs_current(
        rows,
        "pose_euclidean_displacement",
        "Pose Euclidean displacement [m]",
        batch_dir / "euclidean_displacement_vs_current.png",
    )
    plot_metric_vs_current(
        rows,
        "absolute_error",
        "Absolute distance error [m]",
        batch_dir / "distance_error_vs_current.png",
    )
    plot_metric_vs_current(rows, "duration", "Duration [s]", batch_dir / "duration_vs_current.png")
    plot_forward_vs_euclidean(rows, batch_dir / "forward_vs_euclidean_distance.png")

    print("\nStep 2.1 batch summary")
    print(f"Total runs: {len(rows)}")
    print(f"Successful runs: {sum(row['stop_reason'] == 'target_reached' for row in rows)}")
    print(f"Timeout runs: {sum(row['stop_reason'] == 'timeout' for row in rows)}")
    print(f"Results directory: {batch_dir}")


def validate_args(args: argparse.Namespace) -> None:
    if args.target_distance <= 0:
        raise ValueError("target-distance must be positive.")
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


def make_run_id(current_y: float, repetition_index: int) -> str:
    current_text = str(current_y).replace("-", "neg").replace(".", "p")
    return f"current_y_{current_text}_rep_{repetition_index:02d}"


def summary_to_row(
    summary: dict,
    run_id: str,
    target_distance: float,
    repetition_index: int,
) -> dict:
    return {
        "run_id": run_id,
        "current_x": float(summary["current_x"]),
        "current_y": float(summary["current_y"]),
        "current_z": float(summary["current_z"]),
        "current_magnitude": float(summary["current_magnitude"]),
        "target_distance": float(target_distance),
        "repetition_index": repetition_index,
        "forward_command": float(summary["forward_command"]),
        "dvl_estimated_distance": float(summary["dvl_estimated_distance_m"]),
        "pose_forward_displacement": float(summary["pose_forward_displacement_m"]),
        "pose_euclidean_displacement": float(summary["pose_ground_truth_displacement_m"]),
        "absolute_error": float(summary["absolute_distance_error_m"]),
        "percentage_error": float(summary["percentage_error"]),
        "lateral_drift": float(summary["lateral_drift_m"]),
        "duration": float(summary["duration_s"]),
        "sample_count": int(summary["num_samples"]),
        "stop_reason": summary["stop_reason"],
        "mean_dt": float(summary["mean_dt"]),
        "std_dt": float(summary["std_dt"]),
        "dt_source": summary["dt_source"],
        "max_dvl_forward_velocity": float(summary["max_dvl_forward_velocity_mps"]),
        "mean_dvl_forward_velocity": float(summary["mean_dvl_forward_velocity_mps"]),
        "average_pose_speed": float(summary["average_pose_speed_mps"]),
    }


def write_all_runs_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def build_aggregate_summary(args: argparse.Namespace, batch_dir: Path, rows: list[dict]) -> dict:
    currents = {}
    for current_y in sorted({row["current_y"] for row in rows}):
        current_rows = [row for row in rows if row["current_y"] == current_y]
        currents[str(current_y)] = {
            "successful_runs": sum(row["stop_reason"] == "target_reached" for row in current_rows),
            "timeout_runs": sum(row["stop_reason"] == "timeout" for row in current_rows),
            "mean_lateral_drift": mean_metric(current_rows, "lateral_drift"),
            "std_lateral_drift": std_metric(current_rows, "lateral_drift"),
            "mean_absolute_error": mean_metric(current_rows, "absolute_error"),
            "std_absolute_error": std_metric(current_rows, "absolute_error"),
            "mean_percentage_error": mean_metric(current_rows, "percentage_error"),
            "std_percentage_error": std_metric(current_rows, "percentage_error"),
            "mean_duration": mean_metric(current_rows, "duration"),
            "std_duration": std_metric(current_rows, "duration"),
            "mean_euclidean_displacement": mean_metric(current_rows, "pose_euclidean_displacement"),
            "std_euclidean_displacement": std_metric(current_rows, "pose_euclidean_displacement"),
        }

    return {
        "output_dir": str(batch_dir),
        "target_distance_m": float(args.target_distance),
        "current_x": float(args.current_x),
        "current_z": float(args.current_z),
        "current_y_values": [float(value) for value in args.current_y_values],
        "repetitions": int(args.repetitions),
        "max_duration_s": float(args.max_duration),
        "forward_command": float(args.forward_command),
        "show_viewport": bool(args.show_viewport),
        "current_application_mode": "every_step",
        "current_api_method": "env.set_ocean_currents(agent_name, velocity)",
        "currents": currents,
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


def plot_metric_vs_current(rows: list[dict], metric_key: str, ylabel: str, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    current_values = sorted({row["current_y"] for row in rows})
    means = []

    for current_y in current_values:
        values = [row[metric_key] for row in rows if row["current_y"] == current_y]
        means.append(float(np.mean(values)))
        ax.scatter([current_y] * len(values), values, alpha=0.7)

    ax.plot(current_values, means, color="black", marker="o", linewidth=1.5, label="Mean")
    ax.set_xlabel("Lateral current y [m/s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_forward_vs_euclidean(rows: list[dict], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(6, 6))
    current_values = sorted({row["current_y"] for row in rows})
    for current_y in current_values:
        current_rows = [row for row in rows if row["current_y"] == current_y]
        ax.scatter(
            [row["pose_forward_displacement"] for row in current_rows],
            [row["pose_euclidean_displacement"] for row in current_rows],
            label=f"current_y={current_y:g}",
        )

    ax.set_xlabel("Pose forward displacement [m]")
    ax.set_ylabel("Pose Euclidean displacement [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
