from __future__ import annotations

from dataclasses import dataclass


BLUE_ROV2_SOURCE_URL = "https://bluerobotics.com/store/rov/bluerov2/"
ROVSUB_USER_REFERENCE_URL = "https://rovsub.it/index.php/project/stacys-photo-set-2/"


DEFAULT_LAYOUT_NOTES = (
    "BlueROV2 Heavy-style 8-thruster authority model. "
    "This is not a verified real motor-order map. "
    "The ROVSub URL was user-provided; official Blue Robotics documentation "
    "is used for accessible factual vehicle references."
)


@dataclass
class BlueROV2HeavyThrusterLayout:
    """First-order BlueROV2 Heavy-style thruster group counts.

    The counts describe authority groups only. They do not encode real motor
    order, allocation signs, thruster angles, or HoloOcean command indexes.
    """

    total_thrusters: int = 8
    horizontal_thrusters: int = 4
    vertical_thrusters: int = 4
    vehicle_source_url: str = BLUE_ROV2_SOURCE_URL
    user_reference_url: str = ROVSUB_USER_REFERENCE_URL
    notes: str = DEFAULT_LAYOUT_NOTES

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        if self.total_thrusters <= 0:
            raise ValueError("total_thrusters must be positive.")
        if self.horizontal_thrusters < 0:
            raise ValueError("horizontal_thrusters must be non-negative.")
        if self.vertical_thrusters < 0:
            raise ValueError("vertical_thrusters must be non-negative.")
        if self.horizontal_thrusters + self.vertical_thrusters != self.total_thrusters:
            raise ValueError(
                "horizontal_thrusters + vertical_thrusters must equal total_thrusters."
            )

    def horizontal_thruster_count(self) -> int:
        return self.horizontal_thrusters

    def vertical_thruster_count(self) -> int:
        return self.vertical_thrusters

    def to_dict(self) -> dict:
        return {
            "total_thrusters": self.total_thrusters,
            "horizontal_thrusters": self.horizontal_thrusters,
            "vertical_thrusters": self.vertical_thrusters,
            "vehicle_source_url": self.vehicle_source_url,
            "user_reference_url": self.user_reference_url,
            "notes": self.notes,
        }
