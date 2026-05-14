from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def bluerov2_horizontal_command(
    forward_command: float,
    lateral_command: float,
    base_vertical_command: float = 0.0,
    max_thruster_command: float | None = None,
) -> np.ndarray:
    """Build a BlueROV2 control-scheme-0 command from body-frame motions.

    Thrusters 0..3 are vertical thrusters. Thrusters 4..7 are horizontal.
    Positive forward command drives horizontal thrusters 4..7 equally.
    Positive lateral command uses [+, -, +, -] on thrusters 4..7. This sign
    convention is matched to the default DVL lateral axis used by Step 2B.
    """

    command = np.zeros(8, dtype=float)
    command[0:4] = float(base_vertical_command)
    command[4] = float(forward_command + lateral_command)
    command[5] = float(forward_command - lateral_command)
    command[6] = float(forward_command + lateral_command)
    command[7] = float(forward_command - lateral_command)
    if max_thruster_command is not None:
        command = np.clip(
            command,
            -float(max_thruster_command),
            float(max_thruster_command),
        )
    return command


@dataclass
class DVLVelocityTrackingController:
    """Proportional DVL body-frame velocity tracking controller."""

    kp_forward: float = 20.0
    kp_lateral: float = 12.0
    max_forward_command: float = 2.0
    max_lateral_command: float = 2.0
    max_thruster_command: float = 2.0
    base_vertical_command: float = 0.0

    def __post_init__(self) -> None:
        if self.kp_forward < 0:
            raise ValueError("kp_forward must be non-negative.")
        if self.kp_lateral < 0:
            raise ValueError("kp_lateral must be non-negative.")
        if self.max_forward_command < 0:
            raise ValueError("max_forward_command must be non-negative.")
        if self.max_lateral_command < 0:
            raise ValueError("max_lateral_command must be non-negative.")
        if self.max_thruster_command < 0:
            raise ValueError("max_thruster_command must be non-negative.")
        self.final_clipping_applied = False
        self.last_final_clipping_applied = False

    def command(
        self,
        desired_forward_velocity: float,
        desired_lateral_velocity: float,
        measured_forward_velocity: float,
        measured_lateral_velocity: float,
    ) -> np.ndarray:
        forward_error = float(desired_forward_velocity - measured_forward_velocity)
        lateral_error = float(desired_lateral_velocity - measured_lateral_velocity)
        forward_command = np.clip(
            self.kp_forward * forward_error,
            -self.max_forward_command,
            self.max_forward_command,
        )
        lateral_command = np.clip(
            self.kp_lateral * lateral_error,
            -self.max_lateral_command,
            self.max_lateral_command,
        )
        mixed_command = bluerov2_horizontal_command(
            forward_command=float(forward_command),
            lateral_command=float(lateral_command),
            base_vertical_command=self.base_vertical_command,
        )
        clipped_command = np.clip(
            mixed_command,
            -self.max_thruster_command,
            self.max_thruster_command,
        )
        self.last_final_clipping_applied = bool(
            not np.allclose(mixed_command, clipped_command)
        )
        self.final_clipping_applied = (
            self.final_clipping_applied or self.last_final_clipping_applied
        )
        return clipped_command

    def stop_command(self) -> np.ndarray:
        return np.zeros(8, dtype=float)
