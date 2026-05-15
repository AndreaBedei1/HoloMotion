from __future__ import annotations

from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def plot_velocity_tracking(samples: Sequence[dict], output_path: Path | str) -> Path:
    if not samples:
        raise ValueError("Cannot plot velocity tracking without trajectory samples.")

    times = [sample["time"] for sample in samples]
    desired_forward = [sample["desired_forward_velocity"] for sample in samples]
    measured_forward = [sample["dvl_forward_velocity_used"] for sample in samples]
    desired_lateral = [sample["desired_lateral_velocity"] for sample in samples]
    measured_lateral = [sample["dvl_lateral_velocity_used"] for sample in samples]

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(times, desired_forward, linestyle="--", label="Desired forward")
    axes[0].plot(times, measured_forward, label="DVL forward")
    axes[0].set_ylabel("Forward velocity [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(times, desired_lateral, linestyle="--", label="Desired lateral")
    axes[1].plot(times, measured_lateral, label="DVL lateral")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Lateral velocity [m/s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_command_history(samples: Sequence[dict], output_path: Path | str) -> Path:
    if not samples:
        raise ValueError("Cannot plot commands without trajectory samples.")

    times = [sample["time"] for sample in samples]
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for index in range(8):
        ax.plot(times, [sample[f"cmd_{index}"] for sample in samples], label=f"cmd_{index}")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Thruster command")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_mode_metric_comparison(
    rows: Sequence[dict],
    metric_key: str,
    ylabel: str,
    output_path: Path | str,
) -> Path:
    if not rows:
        raise ValueError("Cannot plot comparison metrics without rows.")

    fig, ax = plt.subplots(figsize=(8, 4.8))
    preferred_modes = [
        "no_compensation",
        "dvl_velocity_compensation",
        "dvl_pi_velocity_compensation",
    ]
    row_modes = {row["mode"] for row in rows}
    modes = [mode for mode in preferred_modes if mode in row_modes]
    modes.extend(sorted(row_modes - set(modes)))
    targets = sorted({float(row["target_distance"]) for row in rows})
    for target in targets:
        for mode in modes:
            mode_rows = [
                row
                for row in rows
                if float(row["target_distance"]) == target and row["mode"] == mode
            ]
            current_values = sorted({float(row["current_y"]) for row in mode_rows})
            means = [
                float(np.mean([float(row[metric_key]) for row in mode_rows if float(row["current_y"]) == current_y]))
                for current_y in current_values
            ]
            linestyle = {
                "no_compensation": "--",
                "dvl_velocity_compensation": "-",
                "dvl_pi_velocity_compensation": "-.",
            }.get(mode, ":")
            ax.plot(
                current_values,
                means,
                marker="o",
                linestyle=linestyle,
                label=f"{mode}, target={target:g} m",
            )

    ax.set_xlabel("Lateral current y [m/s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_velocity_error_summary(rows: Sequence[dict], output_path: Path | str) -> Path:
    comp_rows = [row for row in rows if row["mode"] == "dvl_velocity_compensation"]
    if not comp_rows:
        raise ValueError("Cannot plot velocity error summary without compensation rows.")

    fig, ax = plt.subplots(figsize=(8, 4.8))
    targets = sorted({float(row["target_distance"]) for row in comp_rows})
    for target in targets:
        target_rows = [row for row in comp_rows if float(row["target_distance"]) == target]
        current_values = sorted({float(row["current_y"]) for row in target_rows})
        lateral_errors = [
            float(
                np.mean(
                    [
                        float(row["mean_abs_lateral_velocity_error"])
                        for row in target_rows
                        if float(row["current_y"]) == current_y
                    ]
                )
            )
            for current_y in current_values
        ]
        ax.plot(current_values, lateral_errors, marker="o", label=f"target={target:g} m")

    ax.set_xlabel("Lateral current y [m/s]")
    ax.set_ylabel("Mean absolute lateral velocity error [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
