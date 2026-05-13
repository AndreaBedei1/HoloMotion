from dataclasses import dataclass

import numpy as np


@dataclass
class ForwardThrusterController:
    """
    Constant forward-thrust controller for BlueROV2 control scheme 0.

    HoloOcean BlueROV2 thruster commands use an 8-value vector. The last four
    entries drive the horizontal thrusters, following the same convention used
    by the existing keyboard controller.
    """

    thrust: float = 12.0
    max_thrust: float = 20.0

    def command(self) -> np.ndarray:
        cmd = np.zeros(8, dtype=float)
        cmd[4:8] = self._clamped_thrust()
        return cmd

    def stop_command(self) -> np.ndarray:
        return np.zeros(8, dtype=float)

    def _clamped_thrust(self) -> float:
        return float(np.clip(self.thrust, -self.max_thrust, self.max_thrust))
