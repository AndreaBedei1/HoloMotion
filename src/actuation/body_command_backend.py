from __future__ import annotations

from abc import ABC, abstractmethod

from control.body_commands import BodyCommand


class BodyCommandBackend(ABC):
    """Interface for backends that consume normalized body-frame commands."""

    @abstractmethod
    def send_body_command(self, command: BodyCommand) -> None:
        """Send a body-frame command to an actuation backend."""


class ArduSubMavlinkBodyCommandBackend(BodyCommandBackend):
    """Placeholder for a future real BlueROV2 ArduSub/MAVLink backend.

    Real BlueROV2 and BlueROV2 Heavy vehicles should normally send body-axis
    controls to ArduSub and let ArduSub's configured frame mixer translate
    those commands to motor outputs. This project intentionally does not send
    hard-coded direct motor commands to real hardware.
    """

    def send_body_command(self, command: BodyCommand) -> None:
        if not isinstance(command, BodyCommand):
            raise TypeError("command must be a BodyCommand.")
        raise NotImplementedError(
            "ArduSub/MAVLink communication is not implemented. "
            "Use ArduSub's configured frame mixer for real hardware."
        )
