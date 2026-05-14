from __future__ import annotations

from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def plot_metric_vs_current(
    rows: Sequence[dict],
    metric_key: str,
    ylabel: str,
    output_path: Path | str,
) -> Path:
    if not rows:
        raise ValueError("Cannot plot current metrics without rows.")

    fig, ax = plt.subplots(figsize=(8, 4.8))
    targets = sorted({row["target_distance"] for row in rows})
    for target in targets:
        target_rows = [row for row in rows if row["target_distance"] == target]
        x_values = [_signed_current_axis(row) for row in target_rows]
        y_values = [row[metric_key] for row in target_rows]
        ax.scatter(x_values, y_values, alpha=0.7, label=f"target={target:g} m")

    ax.set_xlabel("Signed dominant current [m/s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_metric_vs_target(
    rows: Sequence[dict],
    metric_key: str,
    ylabel: str,
    output_path: Path | str,
) -> Path:
    if not rows:
        raise ValueError("Cannot plot target metrics without rows.")

    fig, ax = plt.subplots(figsize=(8, 4.8))
    current_cases = _sorted_current_cases(rows)
    for current_case in current_cases:
        case_rows = [row for row in rows if _current_key(row) == current_case]
        targets = sorted({row["target_distance"] for row in case_rows})
        means = [
            float(np.mean([row[metric_key] for row in case_rows if row["target_distance"] == target]))
            for target in targets
        ]
        ax.plot(targets, means, marker="o", linewidth=1.2, label=_current_label(case_rows[0]))

    ax.set_xlabel("Target distance [m]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_duration_vs_target_and_current(rows: Sequence[dict], output_path: Path | str) -> Path:
    return plot_metric_vs_target(rows, "duration", "Duration [s]", output_path)


def plot_forward_vs_euclidean(rows: Sequence[dict], output_path: Path | str) -> Path:
    if not rows:
        raise ValueError("Cannot plot forward/euclidean distance without rows.")

    fig, ax = plt.subplots(figsize=(6, 6))
    current_cases = _sorted_current_cases(rows)
    for current_case in current_cases:
        case_rows = [row for row in rows if _current_key(row) == current_case]
        ax.scatter(
            [row["pose_forward_displacement"] for row in case_rows],
            [row["pose_euclidean_displacement"] for row in case_rows],
            label=_current_label(case_rows[0]),
            alpha=0.7,
        )

    ax.set_xlabel("Pose forward displacement [m]")
    ax.set_ylabel("Pose Euclidean displacement [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _signed_current_axis(row: dict) -> float:
    if abs(row["current_y"]) >= abs(row["current_x"]):
        return float(row["current_y"])
    return float(row["current_x"])


def _current_key(row: dict) -> tuple[float, float, float]:
    return (float(row["current_x"]), float(row["current_y"]), float(row["current_z"]))


def _sorted_current_cases(rows: Sequence[dict]) -> list[tuple[float, float, float]]:
    return sorted({_current_key(row) for row in rows}, key=lambda item: (item[1], item[0], item[2]))


def _current_label(row: dict) -> str:
    return (
        f"current=[{row['current_x']:g}, "
        f"{row['current_y']:g}, {row['current_z']:g}]"
    )
