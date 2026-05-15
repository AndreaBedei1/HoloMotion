from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.altitude_hold import (
    DEFAULT_MAX_INVALID_PING_HOLD,
    DEFAULT_MAX_THRUSTER_COMMAND,
    DEFAULT_MAX_VERTICAL_COMMAND,
    failed_summary_from_exception,
    run_step_03_altitude_hold_experiment,
    write_json,
)
from experiments.dvl_velocity_compensation import (
    DEFAULT_FORWARD_KP,
    DEFAULT_LATERAL_KP,
    DEFAULT_MAX_COMMAND,
)
from lib.worlds import World


OPENWATER_DOCS_URL = (
    "https://byu-holoocean.github.io/holoocean-docs/v2.2.2/"
    "packages/Ocean/OpenWater/openwater.html"
)

# Coordinates are HoloOcean coordinates copied from the OpenWater documentation
# landmark images. The transects cross each documented depression along +x.
OPENWATER_HOLE_TRANSECTS = [
    {
        "name": "openwater_depression_landmark_12",
        "hole_center_x": 306.6,
        "hole_center_y": -321.7,
        "hole_center_z": -312.4,
        "documentation_source": f"{OPENWATER_DOCS_URL}#open_landmark3",
        "initial_x": 296.6,
        "initial_y": -321.7,
        "target_distance": 20.0,
        "desired_altitude": 3.0,
        "flat_seabed_z": -310.5,
        "initial_z": -307.5,
        "enabled_by_default": True,
    },
    {
        "name": "openwater_depression_landmark_7",
        "hole_center_x": -403.17,
        "hole_center_y": 459.44,
        "hole_center_z": -310.02,
        "documentation_source": f"{OPENWATER_DOCS_URL}#open_landmark2",
        "initial_x": -413.17,
        "initial_y": 459.44,
        "target_distance": 20.0,
        "desired_altitude": 3.0,
        "flat_seabed_z": -305.3,
        "initial_z": -302.3,
        "enabled_by_default": False,
    },
]


SUMMARY_COLUMNS = [
    "transect_name",
    "hole_center_x",
    "hole_center_y",
    "hole_center_z",
    "target_distance",
    "desired_altitude",
    "initial_x",
    "initial_y",
    "initial_z",
    "flat_seabed_z",
    "target_reached",
    "stop_reason",
    "timeout",
    "collision",
    "unsafe_altitude",
    "invalid_ping_failure",
    "runtime_seconds",
    "final_lateral_drift",
    "rmse_altitude_error",
    "max_abs_altitude_error",
    "time_inside_altitude_band_percent",
    "estimated_seabed_z_range",
    "estimated_seabed_z_min",
    "estimated_seabed_z_max",
    "vertical_saturation_percent",
    "output_dir",
]


def main() -> None:
    args = parse_args()
    output = run_openwater_hole_batch(args)
    print_openwater_report(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 3B OpenWater hole-crossing terrain-following validation."
    )
    parser.add_argument("--approach-distance", type=float, default=10.0)
    parser.add_argument("--desired-altitude", type=float, default=3.0)
    parser.add_argument("--desired-forward-velocity", type=float, default=0.15)
    parser.add_argument("--altitude-tolerance", type=float, default=0.30)
    parser.add_argument("--min-safe-altitude", type=float, default=1.0)
    parser.add_argument("--current-y", type=float, default=0.0)
    parser.add_argument("--kp-altitude", type=float, default=0.6)
    parser.add_argument("--max-vertical-command", type=float, default=DEFAULT_MAX_VERTICAL_COMMAND)
    parser.add_argument("--kp-forward", type=float, default=DEFAULT_FORWARD_KP)
    parser.add_argument("--kp-lateral", type=float, default=DEFAULT_LATERAL_KP)
    parser.add_argument("--max-forward-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-lateral-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-thruster-command", type=float, default=DEFAULT_MAX_THRUSTER_COMMAND)
    parser.add_argument("--max-duration", type=float, default=180.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--ping-max-range", type=float, default=100.0)
    parser.add_argument("--max-invalid-ping-hold-s", type=float, default=DEFAULT_MAX_INVALID_PING_HOLD)
    parser.add_argument("--max-transects", type=int, default=None)
    parser.add_argument(
        "--include-risky-transects",
        action="store_true",
        help="Also run documented transects that intersect known landmarks or obstacles.",
    )
    parser.add_argument(
        "--headless",
        action="store_false",
        dest="show_viewport",
        help="Run without the HoloOcean viewport.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "step_03_openwater_holes",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_openwater_hole_batch(args: argparse.Namespace) -> dict:
    validate_args(args)
    batch_dir = args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir.mkdir(parents=True, exist_ok=True)
    transects = build_transects(args)
    if args.max_transects is not None:
        transects = transects[: args.max_transects]

    metadata = {
        "step": "Step 3B",
        "title": "OpenWater hole-crossing terrain-following validation",
        "documentation_source": OPENWATER_DOCS_URL,
        "world": World.OpenWater,
        "controller_policy": "Only PingAltimeter is used for vertical control.",
        "pose_policy": "Pose is used only offline for trajectory and terrain reconstruction metrics.",
        "transects": transects,
    }
    write_json(batch_dir / "metadata.json", metadata)

    rows = []
    for index, transect in enumerate(transects, start=1):
        print(
            f"[{index}/{len(transects)}] {transect['name']} "
            f"center=({transect['hole_center_x']:.2f}, {transect['hole_center_y']:.2f})"
        )
        run_args = build_run_args(args, transect, repetition=index)
        run_dir = batch_dir / transect["name"]
        try:
            summary = run_step_03_altitude_hold_experiment(
                run_args,
                output_dir=run_dir,
                print_terminal_summary=False,
            )
            plot_openwater_hole_run(
                run_dir / "trajectory.csv",
                run_dir / "openwater_hole_profile.png",
            )
        except Exception as exc:
            summary = failed_summary_from_exception(run_args, run_dir, exc)
            run_dir.mkdir(parents=True, exist_ok=True)
            write_json(run_dir / "summary.json", summary)
        row = summary_to_row(summary, transect)
        rows.append(row)
        write_all_runs_summary(batch_dir / "all_runs_summary.csv", rows)
        write_json(batch_dir / "aggregate_summary.json", build_aggregate_summary(rows, batch_dir))

    aggregate = build_aggregate_summary(rows, batch_dir)
    write_all_runs_summary(batch_dir / "all_runs_summary.csv", rows)
    write_json(batch_dir / "aggregate_summary.json", aggregate)
    return {"batch_dir": batch_dir, "rows": rows, "aggregate": aggregate}


def build_transects(args: argparse.Namespace) -> list[dict]:
    transects = []
    for base in OPENWATER_HOLE_TRANSECTS:
        if not base.get("enabled_by_default", True) and not args.include_risky_transects:
            continue
        transect = dict(base)
        transect["initial_x"] = float(base["hole_center_x"] - args.approach_distance)
        transect["initial_y"] = float(base["hole_center_y"])
        transect["target_distance"] = float(2.0 * args.approach_distance)
        transect["desired_altitude"] = float(args.desired_altitude)
        if base.get("initial_z") is not None:
            transect["initial_z"] = float(
                base["initial_z"] + args.desired_altitude - base["desired_altitude"]
            )
        transects.append(transect)
    return transects


def build_run_args(args: argparse.Namespace, transect: dict, repetition: int) -> argparse.Namespace:
    return argparse.Namespace(
        target_distance=float(transect["target_distance"]),
        desired_altitude=float(transect["desired_altitude"]),
        altitude_tolerance=float(args.altitude_tolerance),
        min_safe_altitude=float(args.min_safe_altitude),
        desired_forward_velocity=float(args.desired_forward_velocity),
        desired_lateral_velocity=0.0,
        current_x=0.0,
        current_y=float(args.current_y),
        current_z=0.0,
        repetition=int(repetition),
        kp_altitude=float(args.kp_altitude),
        max_vertical_command=float(args.max_vertical_command),
        kp_forward=float(args.kp_forward),
        kp_lateral=float(args.kp_lateral),
        max_forward_command=float(args.max_forward_command),
        max_lateral_command=float(args.max_lateral_command),
        max_thruster_command=float(args.max_thruster_command),
        max_duration=float(args.max_duration),
        ticks_per_sec=int(args.ticks_per_sec),
        warmup_ticks=int(args.warmup_ticks),
        dvl_forward_index=int(args.dvl_forward_index),
        dvl_forward_sign=float(args.dvl_forward_sign),
        dvl_lateral_index=int(args.dvl_lateral_index),
        dvl_lateral_sign=float(args.dvl_lateral_sign),
        initial_x=float(transect["initial_x"]),
        initial_y=float(transect["initial_y"]),
        initial_z=transect.get("initial_z"),
        flat_seabed_z=float(transect["flat_seabed_z"]),
        ping_max_range=float(args.ping_max_range),
        max_invalid_ping_hold_s=float(args.max_invalid_ping_hold_s),
        world=World.OpenWater,
        show_viewport=bool(args.show_viewport),
        results_dir=args.results_dir,
        hole_name=transect["name"],
        hole_center_x=float(transect["hole_center_x"]),
        hole_center_y=float(transect["hole_center_y"]),
        hole_center_z=float(transect["hole_center_z"]),
        documentation_source=transect["documentation_source"],
    )


def summary_to_row(summary: dict, transect: dict) -> dict:
    row = {
        "transect_name": transect["name"],
        "hole_center_x": float(transect["hole_center_x"]),
        "hole_center_y": float(transect["hole_center_y"]),
        "hole_center_z": float(transect["hole_center_z"]),
        "initial_x": float(transect["initial_x"]),
        "initial_y": float(transect["initial_y"]),
        "initial_z": transect.get("initial_z"),
        "flat_seabed_z": float(transect["flat_seabed_z"]),
    }
    row.update(summary)
    return row


def write_all_runs_summary(path: Path, rows: Sequence[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=SUMMARY_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_aggregate_summary(rows: Sequence[dict], batch_dir: Path) -> dict:
    return {
        "output_dir": str(batch_dir),
        "number_of_runs": len(rows),
        "target_reached_count": count_true(rows, "target_reached"),
        "timeout_count": count_true(rows, "timeout"),
        "collision_count": count_true(rows, "collision"),
        "unsafe_altitude_count": count_true(rows, "unsafe_altitude"),
        "invalid_ping_failure_count": count_true(rows, "invalid_ping_failure"),
        "mean_rmse_altitude_error": mean_metric(rows, "rmse_altitude_error"),
        "max_estimated_seabed_z_range": max_metric(rows, "estimated_seabed_z_range"),
        "transects": [
            {key: row.get(key) for key in SUMMARY_COLUMNS}
            for row in rows
        ],
    }


def plot_openwater_hole_run(trajectory_csv: Path, output_path: Path) -> Path:
    rows = read_csv_rows(trajectory_csv)
    times = values(rows, "sim_time")
    altitude = values(rows, "ping_altitude")
    desired_altitude = values(rows, "desired_altitude")
    altitude_error = values(rows, "altitude_error")
    forward_progress = values(rows, "forward_progress")
    seabed_z = values(rows, "estimated_seabed_z_from_pose_ping")
    pose_z = values(rows, "pose_z")
    vertical_command = values(rows, "vertical_command")

    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=False)
    axes[0].plot(times, altitude, label="Ping altitude")
    axes[0].plot(times, desired_altitude, linestyle="--", label="Desired altitude")
    axes[0].set_ylabel("Altitude [m]")
    axes[0].legend(fontsize="small")

    axes[1].plot(times, altitude_error)
    axes[1].axhline(0.0, linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("Altitude error [m]")

    axes[2].plot(forward_progress, seabed_z, label="Estimated seabed z")
    axes[2].plot(forward_progress, pose_z, label="Pose z")
    axes[2].set_xlabel("Forward progress [m]")
    axes[2].set_ylabel("World z [m]")
    axes[2].legend(fontsize="small")

    axes[3].plot(times, vertical_command)
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Vertical command")

    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def read_csv_rows(path: Path) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def values(rows: Sequence[dict], key: str) -> list[float]:
    parsed = []
    for row in rows:
        value = row.get(key)
        try:
            parsed.append(float(value))
        except (TypeError, ValueError):
            parsed.append(math.nan)
    return parsed


def validate_args(args: argparse.Namespace) -> None:
    if args.approach_distance <= 0.0:
        raise ValueError("approach-distance must be positive.")
    if args.max_transects is not None and args.max_transects <= 0:
        raise ValueError("max-transects must be positive when provided.")


def count_true(rows: Sequence[dict], key: str) -> int:
    return sum(1 for row in rows if truthy(row.get(key)))


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes")
    return bool(value)


def mean_metric(rows: Sequence[dict], key: str) -> float:
    finite = [float(row[key]) for row in rows if is_finite(row.get(key))]
    return float(np.mean(finite)) if finite else math.nan


def max_metric(rows: Sequence[dict], key: str) -> float:
    finite = [float(row[key]) for row in rows if is_finite(row.get(key))]
    return max(finite) if finite else math.nan


def is_finite(value: Any) -> bool:
    try:
        return bool(np.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def print_openwater_report(output: dict) -> None:
    print("\nStep 3B OpenWater hole-crossing summary")
    print(f"Output directory: {output['batch_dir']}")
    for row in output["rows"]:
        print(
            f"{row['transect_name']}: target_reached={row['target_reached']} "
            f"stop_reason={row['stop_reason']} "
            f"rmse_altitude_error={float(row['rmse_altitude_error']):.3f} m "
            f"seabed_z_range={float(row['estimated_seabed_z_range']):.3f} m"
        )


if __name__ == "__main__":
    main()
