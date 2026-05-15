from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from control.body_commands import BodyCommand


def _finite_float(name: str, value: float) -> float:
    numeric_value = float(value)
    if not np.isfinite(numeric_value):
        raise ValueError(f"{name} must be finite.")
    return numeric_value


@dataclass
class HoloOceanBlueROV2Mixer:
    """Convert body-frame commands to HoloOcean BlueROV2 thruster vectors.

    This is a HoloOcean control_scheme=0 mixer. It is not a verified real
    BlueROV2 motor-order mapping. For real hardware, use an ArduSub/MAVLink
    backend that sends body-axis commands to ArduSub's configured frame mixer.
    """

    max_thruster_command: float = 2.0
    base_vertical_command: float = 0.0

    def __post_init__(self) -> None:
        self.max_thruster_command = _finite_float(
            "max_thruster_command",
            self.max_thruster_command,
        )
        if self.max_thruster_command < 0.0:
            raise ValueError("max_thruster_command must be non-negative.")
        self.base_vertical_command = _finite_float(
            "base_vertical_command",
            self.base_vertical_command,
        )

    def mix(self, command: BodyCommand) -> np.ndarray:
        if not isinstance(command, BodyCommand):
            raise TypeError("command must be a BodyCommand.")

        normalized = command.clipped(max_abs=1.0)
        scale = self.max_thruster_command
        surge = normalized.surge * scale
        sway = normalized.sway * scale
        heave = normalized.heave * scale
        yaw = normalized.yaw * scale

        # HoloOcean control_scheme=0 convention used by the existing Step 2B
        # baseline: vertical thrusters are 0..3 and horizontal thrusters are
        # 4..7. This is simulation-specific and must not be used as a real
        # hardware motor order.
        thrusters = build_holoocean_bluerov2_horizontal_command(
            forward_command=surge,
            lateral_command=sway,
            base_vertical_command=self.base_vertical_command + heave,
            max_thruster_command=None,
        )

        if yaw != 0.0:
            thrusters[4] += yaw
            thrusters[5] -= yaw
            thrusters[6] -= yaw
            thrusters[7] += yaw

        return np.clip(thrusters, -scale, scale)


def build_holoocean_bluerov2_horizontal_command(
    forward_command: float,
    lateral_command: float,
    base_vertical_command: float = 0.0,
    max_thruster_command: float | None = None,
) -> np.ndarray:
    """Build the legacy HoloOcean control_scheme=0 horizontal command vector."""

    command = np.zeros(8, dtype=float)
    command[0:4] = float(base_vertical_command)
    command[4] = float(forward_command + lateral_command)
    command[5] = float(forward_command - lateral_command)
    command[6] = float(forward_command + lateral_command)
    command[7] = float(forward_command - lateral_command)
    if max_thruster_command is not None:
        limit = _finite_float("max_thruster_command", max_thruster_command)
        if limit < 0.0:
            raise ValueError("max_thruster_command must be non-negative.")
        command = np.clip(command, -limit, limit)
    return command
