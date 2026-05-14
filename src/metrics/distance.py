from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Sequence

import numpy as np


@dataclass
class DistanceMetrics:
    target_distance_m: float
    dvl_estimated_distance_m: float
    pose_forward_displacement_m: float
    pose_ground_truth_displacement_m: float
    longitudinal_error_m: float
    absolute_longitudinal_error_m: float
    signed_error_m: float
    absolute_distance_error_m: float
    percentage_error: float
    lateral_drift_m: float
    final_position_error_m: float
    duration_s: float
    num_samples: int
    stop_reason: str
    target_reached: bool

    def to_dict(self) -> dict:
        return asdict(self)


def compute_distance_metrics(
    samples: Sequence[dict],
    target_distance_m: float,
    stop_reason: str,
) -> DistanceMetrics:
    if not samples:
        raise ValueError("Cannot compute distance metrics without trajectory samples.")

    first = samples[0]
    last = samples[-1]

    start_position = _position_from_sample(first)
    end_position = _position_from_sample(last)
    displacement = end_position - start_position

    initial_forward = _unit_vector_from_sample(first, "pose_forward", np.array([1.0, 0.0, 0.0]))
    initial_right = _unit_vector_from_sample(first, "pose_right", np.array([0.0, 1.0, 0.0]))

    pose_forward_displacement = float(np.dot(displacement, initial_forward))
    lateral_drift = float(np.dot(displacement, initial_right))
    pose_ground_truth_displacement = float(np.linalg.norm(displacement))
    estimated_distance = float(last["dvl_distance_estimated"])
    target_position = start_position + float(target_distance_m) * initial_forward
    final_position_error = float(np.linalg.norm(end_position - target_position))
    longitudinal_error = pose_forward_displacement - float(target_distance_m)

    signed_error = estimated_distance - pose_forward_displacement
    absolute_error = abs(signed_error)
    reference = (
        abs(pose_forward_displacement)
        if abs(pose_forward_displacement) > 1e-9
        else target_distance_m
    )
    percentage_error = 100.0 * absolute_error / reference if reference > 0 else 0.0

    return DistanceMetrics(
        target_distance_m=float(target_distance_m),
        dvl_estimated_distance_m=estimated_distance,
        pose_forward_displacement_m=pose_forward_displacement,
        pose_ground_truth_displacement_m=pose_ground_truth_displacement,
        longitudinal_error_m=float(longitudinal_error),
        absolute_longitudinal_error_m=float(abs(longitudinal_error)),
        signed_error_m=float(signed_error),
        absolute_distance_error_m=float(absolute_error),
        percentage_error=float(percentage_error),
        lateral_drift_m=lateral_drift,
        final_position_error_m=final_position_error,
        duration_s=float(last["time"] - first["time"]),
        num_samples=len(samples),
        stop_reason=stop_reason,
        target_reached=stop_reason == "target_reached",
    )


def _position_from_sample(sample: dict) -> np.ndarray:
    return np.array([sample["pose_x"], sample["pose_y"], sample["pose_z"]], dtype=float)


def _unit_vector_from_sample(sample: dict, prefix: str, fallback: np.ndarray) -> np.ndarray:
    keys = [f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"]
    if not all(key in sample for key in keys):
        return fallback

    vector = np.array([sample[key] for key in keys], dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return fallback
    return vector / norm
