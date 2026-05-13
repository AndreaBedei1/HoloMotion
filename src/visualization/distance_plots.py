from __future__ import annotations

from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


def plot_distance_results(
    samples: Sequence[dict],
    target_distance_m: float,
    output_path: Path | str,
) -> Path:
    if not samples:
        raise ValueError("Cannot plot distance results without trajectory samples.")

    times = [sample["time"] for sample in samples]
    estimated = [sample["dvl_distance_estimated"] for sample in samples]
    ground_truth = [sample["pose_forward_displacement"] for sample in samples]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, estimated, label="DVL estimate")
    ax.plot(times, ground_truth, label="Ground truth from Pose")
    ax.axhline(target_distance_m, color="black", linestyle="--", linewidth=1, label="Target")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Forward distance [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_trajectory(samples: Sequence[dict], output_path: Path | str) -> Path:
    if not samples:
        raise ValueError("Cannot plot a trajectory without trajectory samples.")

    x = [sample["pose_x"] for sample in samples]
    y = [sample["pose_y"] for sample in samples]

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(x, y, label="Ground truth trajectory")
    ax.scatter([x[0]], [y[0]], color="green", label="Start", zorder=3)
    ax.scatter([x[-1]], [y[-1]], color="red", label="End", zorder=3)
    ax.set_xlabel("World x [m]")
    ax.set_ylabel("World y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_lateral_drift(samples: Sequence[dict], output_path: Path | str) -> Path:
    if not samples:
        raise ValueError("Cannot plot lateral drift without trajectory samples.")

    times = [sample["time"] for sample in samples]
    drift = [sample["pose_lateral_drift"] for sample in samples]

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, drift, label="Pose lateral drift")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lateral drift [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_speed_results(samples: Sequence[dict], output_path: Path | str) -> Path:
    if not samples:
        raise ValueError("Cannot plot speed results without trajectory samples.")

    times = [sample["time"] for sample in samples]
    dvl_speed = [sample["dvl_forward_velocity_used"] for sample in samples]
    pose_speed = _pose_speed_from_samples(samples)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, dvl_speed, label="DVL forward velocity")
    ax.plot(times, pose_speed, label="Pose Euclidean speed")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    path = Path(output_path)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _pose_speed_from_samples(samples: Sequence[dict]) -> list[float]:
    speeds = [0.0]
    for previous, current in zip(samples, samples[1:]):
        dt = current["time"] - previous["time"]
        if dt <= 0:
            speeds.append(0.0)
            continue
        distance = (
            current["pose_euclidean_displacement"]
            - previous["pose_euclidean_displacement"]
        )
        speeds.append(float(distance / dt))
    return speeds
