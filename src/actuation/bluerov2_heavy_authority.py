from __future__ import annotations

from dataclasses import dataclass

from actuation.bluerov2_heavy_config import BlueROV2HeavyThrusterLayout
from actuation.t200_thruster_model import T200ThrusterModel


AUTHORITY_NOTES = (
    "The 8-thruster total is a raw sum, not an axis thrust.",
    "Horizontal 4-thruster authority is the relevant first-order quantity for "
    "Step 2C current compensation.",
    "Vertical 4-thruster authority is the relevant first-order quantity for "
    "Step 3 altitude hold.",
    "This model does not calibrate HoloOcean command units to Newtons.",
)


@dataclass
class BlueROV2HeavyAuthority:
    layout: BlueROV2HeavyThrusterLayout
    thruster_model: T200ThrusterModel

    def __post_init__(self) -> None:
        if not isinstance(self.layout, BlueROV2HeavyThrusterLayout):
            raise TypeError("layout must be a BlueROV2HeavyThrusterLayout.")
        if not isinstance(self.thruster_model, T200ThrusterModel):
            raise TypeError("thruster_model must be a T200ThrusterModel.")
        self.layout.validate()

    @property
    def per_thruster_forward_thrust_n(self) -> float:
        return self.thruster_model.performance.max_forward_thrust_n

    @property
    def per_thruster_reverse_thrust_n(self) -> float:
        return self.thruster_model.performance.max_reverse_thrust_n

    @property
    def total_8_thruster_forward_sum_n(self) -> float:
        return self.layout.total_thrusters * self.per_thruster_forward_thrust_n

    @property
    def total_8_thruster_reverse_sum_n(self) -> float:
        return self.layout.total_thrusters * self.per_thruster_reverse_thrust_n

    @property
    def horizontal_4_thruster_forward_sum_n(self) -> float:
        return self.layout.horizontal_thruster_count() * self.per_thruster_forward_thrust_n

    @property
    def horizontal_4_thruster_reverse_sum_n(self) -> float:
        return self.layout.horizontal_thruster_count() * self.per_thruster_reverse_thrust_n

    @property
    def vertical_4_thruster_forward_sum_n(self) -> float:
        return self.layout.vertical_thruster_count() * self.per_thruster_forward_thrust_n

    @property
    def vertical_4_thruster_reverse_sum_n(self) -> float:
        return self.layout.vertical_thruster_count() * self.per_thruster_reverse_thrust_n

    @property
    def forward_reverse_asymmetry_ratio(self) -> float:
        return self.per_thruster_forward_thrust_n / self.per_thruster_reverse_thrust_n

    def to_dict(self) -> dict:
        return {
            "layout": self.layout.to_dict(),
            "thruster_model": self.thruster_model.to_dict(),
            "per_thruster_forward_thrust_n": self.per_thruster_forward_thrust_n,
            "per_thruster_reverse_thrust_n": self.per_thruster_reverse_thrust_n,
            "total_8_thruster_forward_sum_n": self.total_8_thruster_forward_sum_n,
            "total_8_thruster_reverse_sum_n": self.total_8_thruster_reverse_sum_n,
            "horizontal_4_thruster_forward_sum_n": self.horizontal_4_thruster_forward_sum_n,
            "horizontal_4_thruster_reverse_sum_n": self.horizontal_4_thruster_reverse_sum_n,
            "vertical_4_thruster_forward_sum_n": self.vertical_4_thruster_forward_sum_n,
            "vertical_4_thruster_reverse_sum_n": self.vertical_4_thruster_reverse_sum_n,
            "forward_reverse_asymmetry_ratio": self.forward_reverse_asymmetry_ratio,
            "notes": list(AUTHORITY_NOTES),
        }
