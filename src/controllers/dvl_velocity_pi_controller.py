from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from control.body_commands import (
    BodyCommand,
    BodyVelocityMeasurement,
    BodyVelocitySetpoint,
)


def _finite_float(name: str, value: float) -> float:
    numeric_value = float(value)
    if not np.isfinite(numeric_value):
        raise ValueError(f"{name} must be finite.")
    return numeric_value


def _non_negative_finite_float(name: str, value: float) -> float:
    numeric_value = _finite_float(name, value)
    if numeric_value < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    return numeric_value


def _normalized_limit(name: str, value: float) -> float:
    numeric_value = _non_negative_finite_float(name, value)
    if numeric_value > 1.0:
        raise ValueError(f"{name} must be less than or equal to 1.0.")
    return numeric_value


@dataclass
class DVLVelocityPIController:
    """PI controller for DVL-measured body-frame velocity tracking.

    The controller consumes desired and measured body-frame velocities and
    returns a normalized `BodyCommand`. It does not know about HoloOcean
    thruster ordering, real motor ordering, Pose, or ground-truth Velocity.
    """

    kp_surge: float = 2.0
    ki_surge: float = 0.2
    kp_sway: float = 3.0
    ki_sway: float = 0.5
    max_surge: float = 1.0
    max_sway: float = 1.0
    integral_limit_surge: float = 1.0
    integral_limit_sway: float = 1.0

    def __post_init__(self) -> None:
        self.kp_surge = _non_negative_finite_float("kp_surge", self.kp_surge)
        self.ki_surge = _non_negative_finite_float("ki_surge", self.ki_surge)
        self.kp_sway = _non_negative_finite_float("kp_sway", self.kp_sway)
        self.ki_sway = _non_negative_finite_float("ki_sway", self.ki_sway)
        self.max_surge = _normalized_limit("max_surge", self.max_surge)
        self.max_sway = _normalized_limit("max_sway", self.max_sway)
        self.integral_limit_surge = _non_negative_finite_float(
            "integral_limit_surge",
            self.integral_limit_surge,
        )
        self.integral_limit_sway = _non_negative_finite_float(
            "integral_limit_sway",
            self.integral_limit_sway,
        )
        self.reset()

    def reset(self) -> None:
        self.surge_integral = 0.0
        self.sway_integral = 0.0

    def command(
        self,
        setpoint: BodyVelocitySetpoint,
        measurement: BodyVelocityMeasurement,
        dt_s: float,
    ) -> BodyCommand:
        if not isinstance(setpoint, BodyVelocitySetpoint):
            raise TypeError("setpoint must be a BodyVelocitySetpoint.")
        if not isinstance(measurement, BodyVelocityMeasurement):
            raise TypeError("measurement must be a BodyVelocityMeasurement.")
        dt = _finite_float("dt_s", dt_s)
        if dt <= 0.0:
            raise ValueError("dt_s must be positive.")

        surge_error = setpoint.surge_mps - measurement.surge_mps
        sway_error = setpoint.sway_mps - measurement.sway_mps

        surge_output = self._axis_command(
            error=surge_error,
            dt_s=dt,
            kp=self.kp_surge,
            ki=self.ki_surge,
            previous_integral=self.surge_integral,
            integral_limit=self.integral_limit_surge,
            output_limit=self.max_surge,
        )
        sway_output = self._axis_command(
            error=sway_error,
            dt_s=dt,
            kp=self.kp_sway,
            ki=self.ki_sway,
            previous_integral=self.sway_integral,
            integral_limit=self.integral_limit_sway,
            output_limit=self.max_sway,
        )

        self.surge_integral = surge_output["integral"]
        self.sway_integral = sway_output["integral"]

        metadata = {
            "surge_error": float(surge_error),
            "sway_error": float(sway_error),
            "surge_integral": float(self.surge_integral),
            "sway_integral": float(self.sway_integral),
            "raw_surge_command": float(surge_output["raw_command"]),
            "raw_sway_command": float(sway_output["raw_command"]),
            "surge_saturated": bool(surge_output["saturated"]),
            "sway_saturated": bool(sway_output["saturated"]),
        }

        return BodyCommand(
            surge=float(surge_output["command"]),
            sway=float(sway_output["command"]),
            heave=0.0,
            yaw=0.0,
            saturated=bool(surge_output["saturated"] or sway_output["saturated"]),
            metadata=metadata,
        )

    @staticmethod
    def _axis_command(
        error: float,
        dt_s: float,
        kp: float,
        ki: float,
        previous_integral: float,
        integral_limit: float,
        output_limit: float,
    ) -> dict:
        integral_delta = error * dt_s
        candidate_integral = float(
            np.clip(
                previous_integral + integral_delta,
                -integral_limit,
                integral_limit,
            )
        )
        candidate_raw = kp * error + ki * candidate_integral
        candidate_saturated = abs(candidate_raw) > output_limit

        # Simple anti-windup: when the candidate output is already saturated,
        # do not let the integral keep growing in the direction that would
        # push the command deeper into saturation.
        pushes_deeper_positive = candidate_raw > output_limit and integral_delta > 0.0
        pushes_deeper_negative = candidate_raw < -output_limit and integral_delta < 0.0
        if candidate_saturated and (pushes_deeper_positive or pushes_deeper_negative):
            integral = previous_integral
        else:
            integral = candidate_integral

        raw_command = float(kp * error + ki * integral)
        command = float(np.clip(raw_command, -output_limit, output_limit))
        saturated = bool(abs(raw_command) > output_limit)
        return {
            "integral": float(integral),
            "raw_command": raw_command,
            "command": command,
            "saturated": saturated,
        }
