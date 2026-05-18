from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


KGF_TO_NEWTON = 9.80665
T200_SOURCE_URL = (
    "https://bluerobotics.com/store/thrusters/t100-t200-thrusters/"
    "t200-thruster-r2-rp/"
)


def _finite_float(name: str, value: float) -> float:
    numeric_value = float(value)
    if not isfinite(numeric_value):
        raise ValueError(f"{name} must be finite.")
    return numeric_value


def _optional_non_negative_finite_float(
    name: str,
    value: float | None,
) -> float | None:
    if value is None:
        return None
    numeric_value = _finite_float(name, value)
    if numeric_value < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return numeric_value


@dataclass
class T200PerformancePoint:
    voltage_v: float
    max_forward_thrust_kgf: float
    max_reverse_thrust_kgf: float
    max_current_a: float | None = None
    max_power_w: float | None = None
    source_url: str = T200_SOURCE_URL

    def __post_init__(self) -> None:
        self.voltage_v = _finite_float("voltage_v", self.voltage_v)
        self.max_forward_thrust_kgf = _finite_float(
            "max_forward_thrust_kgf",
            self.max_forward_thrust_kgf,
        )
        self.max_reverse_thrust_kgf = _finite_float(
            "max_reverse_thrust_kgf",
            self.max_reverse_thrust_kgf,
        )
        self.max_current_a = _optional_non_negative_finite_float(
            "max_current_a",
            self.max_current_a,
        )
        self.max_power_w = _optional_non_negative_finite_float(
            "max_power_w",
            self.max_power_w,
        )
        self.validate()

    @property
    def max_forward_thrust_n(self) -> float:
        return self.max_forward_thrust_kgf * KGF_TO_NEWTON

    @property
    def max_reverse_thrust_n(self) -> float:
        return self.max_reverse_thrust_kgf * KGF_TO_NEWTON

    def validate(self) -> None:
        voltage_v = _finite_float("voltage_v", self.voltage_v)
        forward_thrust_kgf = _finite_float(
            "max_forward_thrust_kgf",
            self.max_forward_thrust_kgf,
        )
        reverse_thrust_kgf = _finite_float(
            "max_reverse_thrust_kgf",
            self.max_reverse_thrust_kgf,
        )
        max_current_a = _optional_non_negative_finite_float(
            "max_current_a",
            self.max_current_a,
        )
        max_power_w = _optional_non_negative_finite_float(
            "max_power_w",
            self.max_power_w,
        )
        if voltage_v <= 0.0:
            raise ValueError("voltage_v must be positive.")
        if forward_thrust_kgf <= 0.0:
            raise ValueError("max_forward_thrust_kgf must be positive.")
        if reverse_thrust_kgf <= 0.0:
            raise ValueError("max_reverse_thrust_kgf must be positive.")
        if max_current_a is not None and max_current_a < 0.0:
            raise ValueError("max_current_a must be non-negative.")
        if max_power_w is not None and max_power_w < 0.0:
            raise ValueError("max_power_w must be non-negative.")

    def to_dict(self) -> dict:
        return {
            "voltage_v": self.voltage_v,
            "max_forward_thrust_kgf": self.max_forward_thrust_kgf,
            "max_reverse_thrust_kgf": self.max_reverse_thrust_kgf,
            "max_forward_thrust_n": self.max_forward_thrust_n,
            "max_reverse_thrust_n": self.max_reverse_thrust_n,
            "max_current_a": self.max_current_a,
            "max_power_w": self.max_power_w,
            "source_url": self.source_url,
        }


@dataclass
class T200ThrusterModel:
    performance: T200PerformancePoint

    def __post_init__(self) -> None:
        if not isinstance(self.performance, T200PerformancePoint):
            raise TypeError("performance must be a T200PerformancePoint.")
        self.performance.validate()

    @classmethod
    def from_12v(cls) -> "T200ThrusterModel":
        return cls(
            T200PerformancePoint(
                voltage_v=12.0,
                max_forward_thrust_kgf=3.71,
                max_reverse_thrust_kgf=2.92,
                max_current_a=17.0,
                max_power_w=205.0,
            )
        )

    @classmethod
    def from_nominal_16v(cls) -> "T200ThrusterModel":
        return cls(
            T200PerformancePoint(
                voltage_v=16.0,
                max_forward_thrust_kgf=5.25,
                max_reverse_thrust_kgf=4.10,
                max_current_a=24.0,
                max_power_w=390.0,
            )
        )

    @classmethod
    def from_20v(cls) -> "T200ThrusterModel":
        return cls(
            T200PerformancePoint(
                voltage_v=20.0,
                max_forward_thrust_kgf=6.70,
                max_reverse_thrust_kgf=5.05,
                max_current_a=32.0,
                max_power_w=645.0,
            )
        )

    def command_to_thrust_n(self, command: float) -> float:
        normalized_command = _finite_float("command", command)
        clipped_command = max(-1.0, min(1.0, normalized_command))
        if clipped_command >= 0.0:
            return clipped_command * self.performance.max_forward_thrust_n
        return clipped_command * self.performance.max_reverse_thrust_n

    def thrust_to_command(self, thrust_n: float) -> float:
        requested_thrust_n = _finite_float("thrust_n", thrust_n)
        clipped_thrust_n = self.clip_thrust_n(requested_thrust_n)
        if clipped_thrust_n >= 0.0:
            return clipped_thrust_n / self.performance.max_forward_thrust_n
        return clipped_thrust_n / self.performance.max_reverse_thrust_n

    def clip_thrust_n(self, thrust_n: float) -> float:
        requested_thrust_n = _finite_float("thrust_n", thrust_n)
        return max(
            -self.performance.max_reverse_thrust_n,
            min(self.performance.max_forward_thrust_n, requested_thrust_n),
        )

    def to_dict(self) -> dict:
        return {
            "performance": self.performance.to_dict(),
            "mapping": "piecewise-linear command in [-1, 1] to asymmetric T200 thrust",
        }
