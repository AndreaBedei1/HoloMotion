from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DVLDistanceEstimator:
    """
    Estimate forward distance by integrating the DVL forward velocity.

    HoloOcean's DVL vector starts with velocity_x, velocity_y, and velocity_z.
    The forward axis index and sign are configurable because the correct DVL
    frame convention should be validated against Pose ground truth.
    """

    forward_axis_index: int = 0
    forward_axis_sign: float = 1.0
    clamp_reverse_motion: bool = False

    def __post_init__(self) -> None:
        if self.forward_axis_index < 0:
            raise ValueError("forward_axis_index must be non-negative.")
        if self.forward_axis_sign not in (-1.0, 1.0):
            raise ValueError("forward_axis_sign must be +1 or -1.")

        self.distance_m = 0.0
        self.last_raw_forward_velocity_mps = 0.0
        self.last_used_forward_velocity_mps = 0.0

    def reset(self) -> None:
        self.distance_m = 0.0
        self.last_raw_forward_velocity_mps = 0.0
        self.last_used_forward_velocity_mps = 0.0

    def update(self, dvl_sample, dt_s: float) -> float:
        if dt_s <= 0:
            raise ValueError(f"dt_s must be positive, got {dt_s}.")

        raw_velocity, used_velocity = self.velocity_components(dvl_sample)

        self.last_raw_forward_velocity_mps = raw_velocity
        self.last_used_forward_velocity_mps = used_velocity
        self.distance_m += used_velocity * dt_s
        return self.distance_m

    def velocity_components(self, dvl_sample) -> tuple[float, float]:
        raw_velocity = self._read_forward_velocity(dvl_sample)
        used_velocity = self.forward_axis_sign * raw_velocity
        if self.clamp_reverse_motion:
            used_velocity = max(0.0, used_velocity)
        return raw_velocity, used_velocity

    def _read_forward_velocity(self, dvl_sample) -> float:
        if dvl_sample is None:
            raise ValueError("DVL sample is missing; cannot update distance estimate.")

        values = np.asarray(dvl_sample, dtype=float).reshape(-1)
        if values.size <= self.forward_axis_index:
            raise ValueError(
                "DVL sample does not contain the requested forward velocity "
                f"axis {self.forward_axis_index}. Shape: {np.asarray(dvl_sample).shape}."
            )

        velocity = float(values[self.forward_axis_index])
        if not np.isfinite(velocity):
            raise ValueError(f"DVL forward velocity is not finite: {velocity}.")

        return velocity
