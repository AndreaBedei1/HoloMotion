from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


CURRENT_WARNING_THRESHOLD_MPS = 3.0
EXTREME_CURRENT_WARNING = (
    "Warning: this current is intentionally extreme and may be unrealistic "
    "or destabilize the vehicle."
)


@dataclass(frozen=True)
class CurrentConfig:
    """Constant ocean-current vector used as a controlled disturbance."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_iterable(cls, values: Iterable[float]) -> "CurrentConfig":
        vector = list(values)
        if len(vector) != 3:
            raise ValueError("Current vector must contain exactly three values.")
        return cls(float(vector[0]), float(vector[1]), float(vector[2]))

    @property
    def magnitude(self) -> float:
        return float(np.linalg.norm(np.asarray(self.as_list(), dtype=float)))

    @property
    def enabled(self) -> bool:
        return self.magnitude > 0.0

    def as_list(self) -> list[float]:
        return [float(self.x), float(self.y), float(self.z)]

    def to_dict(self) -> dict:
        return {
            "current_x": float(self.x),
            "current_y": float(self.y),
            "current_z": float(self.z),
            "current_magnitude": self.magnitude,
        }

    def warning(self) -> str:
        if self.magnitude > CURRENT_WARNING_THRESHOLD_MPS:
            return EXTREME_CURRENT_WARNING
        return ""


def current_config_from_args(args) -> CurrentConfig:
    """Build a current config from argparse-style attributes."""

    return CurrentConfig(
        x=float(getattr(args, "current_x", 0.0)),
        y=float(getattr(args, "current_y", 0.0)),
        z=float(getattr(args, "current_z", 0.0)),
    )


def apply_current_to_agent(env, agent_name: str, current: CurrentConfig) -> None:
    """Apply a current vector to one HoloOcean agent."""

    env.set_ocean_currents(agent_name, current.as_list())


def unique_current_cases(cases: Iterable[CurrentConfig]) -> list[CurrentConfig]:
    """Return current cases without duplicate vectors, preserving order."""

    unique: list[CurrentConfig] = []
    seen: set[tuple[float, float, float]] = set()
    for current in cases:
        key = (float(current.x), float(current.y), float(current.z))
        if key in seen:
            continue
        seen.add(key)
        unique.append(current)
    return unique
