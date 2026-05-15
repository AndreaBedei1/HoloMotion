"""Body-frame setpoints, measurements, and normalized command objects.

These dataclasses are the controller-facing API. They deliberately avoid any
HoloOcean thruster index or real BlueROV2 motor-order assumption.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


BODY_COMMAND_COMPONENTS = ("surge", "sway", "heave", "yaw")


def _validate_finite_float(name: str, value: float) -> float:
    numeric_value = float(value)
    if not np.isfinite(numeric_value):
        raise ValueError(f"{name} must be finite.")
    return numeric_value


@dataclass(frozen=True)
class BodyVelocitySetpoint:
    """Desired body-frame velocity used by velocity controllers."""

    surge_mps: float
    sway_mps: float
    heave_mps: float = 0.0
    yaw_rate_rps: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "surge_mps",
            _validate_finite_float("surge_mps", self.surge_mps),
        )
        object.__setattr__(
            self,
            "sway_mps",
            _validate_finite_float("sway_mps", self.sway_mps),
        )
        object.__setattr__(
            self,
            "heave_mps",
            _validate_finite_float("heave_mps", self.heave_mps),
        )
        object.__setattr__(
            self,
            "yaw_rate_rps",
            _validate_finite_float("yaw_rate_rps", self.yaw_rate_rps),
        )

    def as_array(
        self,
        order: Iterable[str] = ("surge", "sway", "heave", "yaw"),
    ) -> np.ndarray:
        values = {
            "surge": self.surge_mps,
            "sway": self.sway_mps,
            "heave": self.heave_mps,
            "yaw": self.yaw_rate_rps,
        }
        return _values_as_array(values, order)


@dataclass(frozen=True)
class BodyVelocityMeasurement:
    """Measured body-frame velocity from a sensor such as a DVL."""

    surge_mps: float
    sway_mps: float
    heave_mps: float = 0.0
    yaw_rate_rps: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "surge_mps",
            _validate_finite_float("surge_mps", self.surge_mps),
        )
        object.__setattr__(
            self,
            "sway_mps",
            _validate_finite_float("sway_mps", self.sway_mps),
        )
        object.__setattr__(
            self,
            "heave_mps",
            _validate_finite_float("heave_mps", self.heave_mps),
        )
        object.__setattr__(
            self,
            "yaw_rate_rps",
            _validate_finite_float("yaw_rate_rps", self.yaw_rate_rps),
        )

    def as_array(
        self,
        order: Iterable[str] = ("surge", "sway", "heave", "yaw"),
    ) -> np.ndarray:
        values = {
            "surge": self.surge_mps,
            "sway": self.sway_mps,
            "heave": self.heave_mps,
            "yaw": self.yaw_rate_rps,
        }
        return _values_as_array(values, order)


@dataclass(frozen=True)
class BodyCommand:
    """Normalized body-frame command independent from any actuation backend."""

    surge: float
    sway: float
    heave: float = 0.0
    yaw: float = 0.0
    saturated: bool = False
    metadata: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "surge", _validate_finite_float("surge", self.surge))
        object.__setattr__(self, "sway", _validate_finite_float("sway", self.sway))
        object.__setattr__(self, "heave", _validate_finite_float("heave", self.heave))
        object.__setattr__(self, "yaw", _validate_finite_float("yaw", self.yaw))
        if self.metadata is None:
            object.__setattr__(self, "metadata", {})
        elif not isinstance(self.metadata, dict):
            raise TypeError("metadata must be a dictionary.")
        else:
            object.__setattr__(self, "metadata", dict(self.metadata))

    def as_array(
        self,
        order: Iterable[str] = BODY_COMMAND_COMPONENTS,
    ) -> np.ndarray:
        values = {
            "surge": self.surge,
            "sway": self.sway,
            "heave": self.heave,
            "yaw": self.yaw,
        }
        return _values_as_array(values, order)

    def clipped(self, max_abs: float = 1.0) -> "BodyCommand":
        limit = _validate_finite_float("max_abs", max_abs)
        if limit < 0.0:
            raise ValueError("max_abs must be non-negative.")

        original = {
            "surge": self.surge,
            "sway": self.sway,
            "heave": self.heave,
            "yaw": self.yaw,
        }
        clipped_values = {
            key: float(np.clip(value, -limit, limit))
            for key, value in original.items()
        }
        changed = any(
            not np.isclose(original[key], clipped_values[key], rtol=0.0, atol=0.0)
            for key in BODY_COMMAND_COMPONENTS
        )

        metadata = dict(self.metadata)
        if changed:
            metadata["clipped"] = True
            metadata["clip_max_abs"] = limit
            metadata["pre_clip_body_command"] = dict(original)

        return BodyCommand(
            surge=clipped_values["surge"],
            sway=clipped_values["sway"],
            heave=clipped_values["heave"],
            yaw=clipped_values["yaw"],
            saturated=bool(self.saturated or changed),
            metadata=metadata,
        )


def _values_as_array(values: dict[str, float], order: Iterable[str]) -> np.ndarray:
    order_tuple = tuple(order)
    unknown = [name for name in order_tuple if name not in values]
    if unknown:
        allowed = ", ".join(sorted(values))
        raise ValueError(f"Unknown body component(s): {unknown}. Allowed: {allowed}.")
    return np.asarray([values[name] for name in order_tuple], dtype=float)
