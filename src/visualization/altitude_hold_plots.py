from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def plot_representative_altitude_runs(
    trajectory_paths: Sequence[Path],
    output_path: Path | str,
) -> Path:
    return _plot_representative_trajectory_metric(
        trajectory_paths=trajectory_paths,
        value_key="ping_altitude",
        ylabel="Ping altitude [m]",
        output_path=output_path,
        include_desired_altitude=True,
    )


def plot_representative_altitude_error_runs(
    trajectory_paths: Sequence[Path],
    output_path: Path | str,
) -> Path:
    return _plot_representative_trajectory_metric(
        trajectory_paths=trajectory_paths,
        value_key="altitude_error",
        ylabel="Altitude error [m]",
        output_path=output_path,
        include_zero_line=True,
    )


def plot_summary_metric_vs_target(
    summary_rows: Sequence[dict],
    metric_key: str,
    ylabel: str,
    output_path: Path | str,
) -> Path:
    if not summary_rows:
        raise ValueError("Cannot plot Step 3 summary metrics without rows.")

    grouped: dict[float, list[dict]] = defaultdict(list)
    for row in summary_rows:
        grouped[float(row["current_y"])].append(row)

    fig, ax = plt.subplots(figsize=(8, 4.8))
    for current_y in sorted(grouped):
        rows = grouped[current_y]
        targets = sorted({float(row["target_distance"]) for row in rows})
        means = []
        for target in targets:
            values = [
                float(row[metric_key])
                for row in rows
                if float(row["target_distance"]) == target
                and _is_number(row.get(metric_key, ""))
            ]
            means.append(float(np.mean(values)) if values else 0.0)
        ax.plot(targets, means, marker="o", label=f"current_y={current_y:g} m/s")

    ax.set_xlabel("Target distance [m]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_representative_trajectory_metric(
    trajectory_paths: Sequence[Path],
    value_key: str,
    ylabel: str,
    output_path: Path | str,
    include_desired_altitude: bool = False,
    include_zero_line: bool = False,
) -> Path:
    if not trajectory_paths:
        raise ValueError("Cannot plot representative Step 3 runs without trajectories.")

    fig, ax = plt.subplots(figsize=(8, 4.8))
    for path in trajectory_paths:
        rows = _read_csv_rows(path)
        if not rows:
            continue
        times = [float(row["sim_time"]) for row in rows if _is_number(row["sim_time"])]
        values = [float(row[value_key]) for row in rows if _is_number(row[value_key])]
        if not times or len(times) != len(values):
            continue
        first = rows[0]
        label = (
            f"target={float(first['target_distance']):g} m, "
            f"current_y={float(first['current_y']):g} m/s"
        )
        ax.plot(times, values, label=label)
        if include_desired_altitude:
            desired = float(first["desired_altitude"])
            ax.axhline(desired, linestyle="--", linewidth=1.0, alpha=0.5)

    if include_zero_line:
        ax.axhline(0.0, linestyle="--", linewidth=1.0, alpha=0.5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _read_csv_rows(path: Path) -> list[dict]:
    with Path(path).open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _is_number(value) -> bool:
    try:
        return np.isfinite(float(value))
    except (TypeError, ValueError):
        return False
