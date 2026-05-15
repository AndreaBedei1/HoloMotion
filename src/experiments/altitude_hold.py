"""Step 3 DVL horizontal tracking with PingAltimeter altitude hold.

The vertical control loop uses only PingAltimeter / RangeFinder altitude.
Pose is logged for evaluation and offline seabed reconstruction only.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.dvl_velocity_controller import DVLVelocityTrackingController
from estimators.dvl_distance import DVLDistanceEstimator
from experiment_logging.experiment_logger import ExperimentLogger
from experiments.dvl_velocity_compensation import dvl_velocity_components
from experiments.forward_distance import (
    DEPTH_SENSOR_KEY,
    DVL_SENSOR_KEY,
    POSE_SENSOR_KEY,
    VELOCITY_SENSOR_KEY,
    apply_ocean_current,
    extract_state_time_s,
    optional_scalar,
    optional_vector,
    pose_components,
    require_sensor,
    run_warmup,
)
from lib.agents import AgentConfig
from lib.current import current_config_from_args
from lib.scenario_builder import ScenarioConfig
from lib.sensors import Sensor
from lib.worlds import World
from visualization.altitude_hold_plots import (
    plot_representative_altitude_error_runs,
    plot_representative_altitude_runs,
    plot_summary_metric_vs_target,
)


PING_SENSOR_KEY = "PingAltimeter"
COLLISION_SENSOR_KEY = "CollisionSensor"
CONTROLLER_NAME = "dvl_p_altitude_hold"
DEFAULT_DESIRED_FORWARD_VELOCITY = 0.3
DEFAULT_DESIRED_LATERAL_VELOCITY = 0.0
DEFAULT_DESIRED_ALTITUDE = 1.5
DEFAULT_ALTITUDE_TOLERANCE = 0.20
DEFAULT_MIN_SAFE_ALTITUDE = 0.50
DEFAULT_KP_ALTITUDE = 0.6
DEFAULT_MAX_VERTICAL_COMMAND = 0.8
DEFAULT_MAX_THRUSTER_COMMAND = 2.0
DEFAULT_MAX_DURATION = 120.0
DEFAULT_FLAT_SEABED_Z = -35.344
DEFAULT_INITIAL_X = -20.0
DEFAULT_INITIAL_Y = 0.0
DEFAULT_MAX_INVALID_PING_HOLD = 1.0


FRAME_FIELDNAMES = [
    "timestamp",
    "step_index",
    "sim_time",
    "controller_name",
    "target_distance",
    "desired_altitude",
    "current_y",
    "repetition",
    "pose_x",
    "pose_y",
    "pose_z",
    "pose_forward_x",
    "pose_forward_y",
    "pose_forward_z",
    "pose_right_x",
    "pose_right_y",
    "pose_right_z",
    "dvl_vx",
    "dvl_vy",
    "dvl_vz",
    "ping_altitude_raw",
    "ping_altitude",
    "estimated_seabed_z_from_pose_ping",
    "altitude_error",
    "abs_altitude_error",
    "desired_forward_velocity",
    "desired_lateral_velocity",
    "vertical_command",
    "horizontal_command_x",
    "horizontal_command_y",
    "forward_progress",
    "lateral_drift",
    "target_reached",
    "collision",
    "invalid_ping",
    "unsafe_altitude",
]


SUMMARY_FIELDNAMES = [
    "controller_name",
    "target_distance",
    "desired_altitude",
    "current_y",
    "repetition",
    "target_reached",
    "timeout",
    "collision",
    "unsafe_altitude",
    "invalid_ping_failure",
    "runtime_seconds",
    "final_forward_progress",
    "final_forward_error",
    "final_lateral_drift",
    "mean_altitude_error",
    "mean_abs_altitude_error",
    "rmse_altitude_error",
    "max_abs_altitude_error",
    "final_altitude_error",
    "min_altitude",
    "max_altitude",
    "estimated_seabed_z_min",
    "estimated_seabed_z_max",
    "estimated_seabed_z_range",
    "estimated_seabed_z_std",
    "time_inside_altitude_band_percent",
    "time_below_safe_altitude_percent",
    "vertical_command_mean_abs",
    "vertical_command_max_abs",
    "vertical_saturation_percent",
    "notes",
    "stop_reason",
    "output_dir",
]


AGGREGATE_FIELDNAMES = [
    "target_distance",
    "desired_altitude",
    "current_y",
    "controller_name",
    "number_of_runs",
    "target_reached_rate",
    "timeout_rate",
    "collision_rate",
    "unsafe_altitude_rate",
    "invalid_ping_failure_rate",
    "final_lateral_drift_mean",
    "final_lateral_drift_std",
    "mean_abs_altitude_error_mean",
    "mean_abs_altitude_error_std",
    "rmse_altitude_error_mean",
    "rmse_altitude_error_std",
    "max_abs_altitude_error_mean",
    "max_abs_altitude_error_std",
    "time_inside_altitude_band_percent_mean",
    "time_inside_altitude_band_percent_std",
    "runtime_seconds_mean",
    "runtime_seconds_std",
    "vertical_saturation_percent_mean",
    "vertical_saturation_percent_std",
]


@dataclass(frozen=True)
class PingAltitudeReading:
    raw_value: Any
    altitude_m: float | None
    valid: bool
    reason: str = ""


def run_step_03_altitude_hold_experiment(
    args: argparse.Namespace,
    output_dir: Path | None = None,
    print_terminal_summary: bool = True,
) -> dict:
    try:
        import holoocean
    except ImportError as exc:
        raise RuntimeError(
            "HoloOcean is not installed in this Python environment. "
            "Install HoloOcean before running Step 3."
        ) from exc

    validate_run_args(args)

    fallback_dt_s = 1.0 / float(args.ticks_per_sec)
    logger = (
        ExperimentLogger(output_dir)
        if output_dir is not None
        else ExperimentLogger.create_timestamped(args.results_dir)
    )
    show_viewport = bool(getattr(args, "show_viewport", True))
    repetition = int(getattr(args, "repetition", 0))
    current_config = current_config_from_args(args)
    current_vector = current_config.as_list()
    current_enabled = current_config.enabled
    current_application_calls = 0

    horizontal_controller = DVLVelocityTrackingController(
        kp_forward=args.kp_forward,
        kp_lateral=args.kp_lateral,
        max_forward_command=args.max_forward_command,
        max_lateral_command=args.max_lateral_command,
        max_thruster_command=args.max_thruster_command,
        base_vertical_command=0.0,
    )
    estimator = DVLDistanceEstimator(
        forward_axis_index=args.dvl_forward_index,
        forward_axis_sign=args.dvl_forward_sign,
    )

    initial_z_policy = (
        "explicit"
        if getattr(args, "initial_z", None) is not None
        else "flat_seabed_plus_desired_altitude"
    )
    initial_z = (
        float(args.initial_z)
        if getattr(args, "initial_z", None) is not None
        else float(args.flat_seabed_z + args.desired_altitude)
    )
    agent = build_step_03_agent(
        sensor_hz=args.ticks_per_sec,
        initial_x=args.initial_x,
        initial_y=getattr(args, "initial_y", DEFAULT_INITIAL_Y),
        initial_z=initial_z,
        ping_max_range=args.ping_max_range,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step03_Altitude_Hold")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(agent)
    )

    run_config = {
        "controller_name": CONTROLLER_NAME,
        "target_distance_m": float(args.target_distance),
        "desired_forward_velocity_mps": float(args.desired_forward_velocity),
        "desired_lateral_velocity_mps": float(args.desired_lateral_velocity),
        "desired_altitude_m": float(args.desired_altitude),
        "altitude_tolerance_m": float(args.altitude_tolerance),
        "min_safe_altitude_m": float(args.min_safe_altitude),
        "kp_altitude": float(args.kp_altitude),
        "max_vertical_command": float(args.max_vertical_command),
        "max_thruster_command": float(args.max_thruster_command),
        "max_duration_s": float(args.max_duration),
        "ticks_per_sec": int(args.ticks_per_sec),
        "warmup_ticks": int(args.warmup_ticks),
        "flat_seabed_z": float(args.flat_seabed_z),
        "initial_x": float(args.initial_x),
        "initial_y": float(getattr(args, "initial_y", DEFAULT_INITIAL_Y)),
        "initial_z": initial_z,
        "initial_z_policy": initial_z_policy,
        "world": args.world,
        "current_x": float(current_vector[0]),
        "current_y": float(current_vector[1]),
        "current_z": float(current_vector[2]),
        "current_magnitude": current_config.magnitude,
        "ping_sensor_key": PING_SENSOR_KEY,
        "ping_sensor_policy": (
            "The vertical control loop uses only the PingAltimeter RangeFinder "
            "measurement. Pose.z is logged for evaluation only."
        ),
        "vertical_command_sign_policy": (
            "Positive vertical command on HoloOcean BlueROV2 vertical thrusters "
            "moves the vehicle upward and increases bottom-relative altitude."
        ),
    }
    for optional_key in (
        "hole_name",
        "hole_center_x",
        "hole_center_y",
        "hole_center_z",
        "documentation_source",
    ):
        if hasattr(args, optional_key):
            run_config[optional_key] = getattr(args, optional_key)
    logger.write_run_config(run_config)

    print(f"Saving Step 3 altitude hold outputs to: {logger.output_dir}")
    print(
        "Target distance and altitude: "
        f"distance={args.target_distance:.3f} m, "
        f"altitude={args.desired_altitude:.3f} m"
    )
    print(
        "Controller: DVL horizontal velocity tracking plus PingAltimeter P altitude hold."
    )

    samples: list[dict] = []
    dt_samples: list[float] = []
    notes: list[str] = []
    stop_reason = "timeout"
    last_valid_altitude: float | None = None
    invalid_ping_age_s = 0.0
    invalid_ping_failure = False
    collision_detected = False
    unsafe_altitude_detected = False
    step_index = 0
    wall_clock_start = time.perf_counter()

    with holoocean.make(
        scenario_cfg=scenario.to_dict(),
        show_viewport=show_viewport,
        ticks_per_sec=args.ticks_per_sec,
        frames_per_sec=args.ticks_per_sec,
        start_world=True,
    ) as env:
        stop_command = np.zeros(8, dtype=float)
        state, calls = run_warmup(
            env,
            stop_command,
            args.warmup_ticks,
            agent_name="rov0",
            current_vector=current_vector if current_enabled else None,
        )
        current_application_calls += calls

        require_sensor(state, DVL_SENSOR_KEY)
        require_sensor(state, POSE_SENSOR_KEY)
        if PING_SENSOR_KEY not in state:
            available = ", ".join(sorted(str(key) for key in state.keys()))
            raise RuntimeError(
                f"PingAltimeter sensor key '{PING_SENSOR_KEY}' was not found. "
                f"Available keys: {available}"
            )

        state_time = extract_state_time_s(state)
        use_state_time = state_time is not None
        last_time_reference = state_time if use_state_time else time.perf_counter()
        elapsed_s = 0.0
        dvl_sample = require_sensor(state, DVL_SENSOR_KEY)

        reading = parse_ping_altitude(state.get(PING_SENSOR_KEY), args.ping_max_range)
        if reading.valid and reading.altitude_m is not None:
            last_valid_altitude = reading.altitude_m
        else:
            invalid_ping_failure = True
            stop_reason = "invalid_ping_failure"
            notes.append(f"Initial PingAltimeter reading invalid: {reading.reason}")

        samples.append(
            build_step_03_sample(
                state=state,
                dvl_sample=dvl_sample,
                time_s=elapsed_s,
                step_index=step_index,
                args=args,
                repetition=repetition,
                ping_reading=reading,
                control_altitude=last_valid_altitude,
                vertical_command=0.0,
                horizontal_command_x=0.0,
                horizontal_command_y=0.0,
                forward_progress=0.0,
                lateral_drift=0.0,
                target_reached=False,
                collision=False,
                invalid_ping=not reading.valid,
                unsafe_altitude=False,
            )
        )

        if not invalid_ping_failure:
            while elapsed_s < args.max_duration:
                measured_forward = estimator.velocity_components(dvl_sample)[1]
                measured_lateral = dvl_velocity_components(
                    dvl_sample,
                    args.dvl_lateral_index,
                    args.dvl_lateral_sign,
                )[1]
                horizontal_command = horizontal_controller.command(
                    desired_forward_velocity=args.desired_forward_velocity,
                    desired_lateral_velocity=args.desired_lateral_velocity,
                    measured_forward_velocity=measured_forward,
                    measured_lateral_velocity=measured_lateral,
                )
                horizontal_command_x, horizontal_command_y = horizontal_components_from_command(
                    horizontal_command
                )

                reading = parse_ping_altitude(
                    state.get(PING_SENSOR_KEY),
                    args.ping_max_range,
                )
                invalid_ping = not reading.valid
                if reading.valid and reading.altitude_m is not None:
                    control_altitude = reading.altitude_m
                    last_valid_altitude = reading.altitude_m
                    invalid_ping_age_s = 0.0
                elif last_valid_altitude is not None:
                    control_altitude = last_valid_altitude
                    if invalid_ping_age_s > args.max_invalid_ping_hold_s:
                        invalid_ping_failure = True
                        stop_reason = "invalid_ping_failure"
                        notes.append(
                            "PingAltimeter reading was invalid longer than the hold limit."
                        )
                        break
                else:
                    invalid_ping_failure = True
                    stop_reason = "invalid_ping_failure"
                    notes.append("No valid PingAltimeter reading was available for control.")
                    break

                unsafe_altitude = bool(control_altitude < args.min_safe_altitude)
                unsafe_altitude_detected = unsafe_altitude_detected or unsafe_altitude
                if unsafe_altitude:
                    stop_reason = "unsafe_altitude"
                    notes.append(
                        "Measured altitude fell below min_safe_altitude; terminating run."
                    )
                    break

                altitude_error = args.desired_altitude - control_altitude
                vertical_command = clipped_vertical_command(
                    altitude_error=altitude_error,
                    kp_altitude=args.kp_altitude,
                    max_vertical_command=args.max_vertical_command,
                )
                command = np.asarray(horizontal_command, dtype=float).copy()
                command[0:4] = vertical_command

                if current_enabled:
                    apply_ocean_current(env, "rov0", current_vector)
                    current_application_calls += 1
                state = env.step(command)
                step_index += 1

                current_reference = (
                    extract_state_time_s(state)
                    if use_state_time
                    else time.perf_counter()
                )
                if current_reference is None:
                    dt_s = fallback_dt_s
                else:
                    dt_s = current_reference - last_time_reference
                    if not np.isfinite(dt_s) or dt_s <= 0.0:
                        dt_s = fallback_dt_s
                    last_time_reference = current_reference
                elapsed_s += dt_s
                dt_samples.append(dt_s)
                if invalid_ping:
                    invalid_ping_age_s += dt_s

                dvl_sample = require_sensor(state, DVL_SENSOR_KEY)
                estimator.update(dvl_sample, dt_s)
                forward_progress, lateral_drift = pose_progress_from_samples(
                    samples[0],
                    state,
                )
                collision = collision_detected_from_state(state)
                collision_detected = collision_detected or collision
                target_reached = bool(estimator.distance_m >= args.target_distance)

                samples.append(
                    build_step_03_sample(
                        state=state,
                        dvl_sample=dvl_sample,
                        time_s=elapsed_s,
                        step_index=step_index,
                        args=args,
                        repetition=repetition,
                        ping_reading=reading,
                        control_altitude=control_altitude,
                        vertical_command=vertical_command,
                        horizontal_command_x=horizontal_command_x,
                        horizontal_command_y=horizontal_command_y,
                        forward_progress=forward_progress,
                        lateral_drift=lateral_drift,
                        target_reached=target_reached,
                        collision=collision,
                        invalid_ping=invalid_ping,
                        unsafe_altitude=unsafe_altitude,
                    )
                )

                if collision:
                    stop_reason = "collision"
                    notes.append("Collision sensor reported a collision.")
                    break
                if target_reached:
                    stop_reason = "target_reached"
                    break

        if current_enabled:
            apply_ocean_current(env, "rov0", current_vector)
            current_application_calls += 1
        env.step(stop_command)

    wall_clock_duration_s = time.perf_counter() - wall_clock_start
    if stop_reason == "timeout":
        notes.append("Target was not reached before max_duration.")

    summary = build_step_03_summary(
        samples=samples,
        args=args,
        repetition=repetition,
        stop_reason=stop_reason,
        wall_clock_duration_s=wall_clock_duration_s,
        invalid_ping_failure=invalid_ping_failure,
        collision_detected=collision_detected,
        unsafe_altitude_detected=unsafe_altitude_detected,
        notes=notes,
        output_dir=logger.output_dir,
    )
    summary["current_application_calls"] = current_application_calls
    summary["mean_dt"] = mean_value(dt_samples)
    summary["std_dt"] = std_value(dt_samples)

    logger.write_trajectory(samples)
    logger.write_summary(summary)

    if print_terminal_summary:
        print_step_03_summary(summary)

    return summary


def build_step_03_agent(
    sensor_hz: int,
    initial_x: float,
    initial_y: float,
    initial_z: float,
    ping_max_range: float,
    include_ground_truth: bool,
) -> AgentConfig:
    sensors = [
        Sensor.DVL(
            socket="DVLSocket",
            Hz=sensor_hz,
            Elevation=22.5,
            VelSigma=0.02,
            ReturnRange=True,
            MaxRange=200.0,
        ),
        Sensor.RangeFinder(
            sensor_name=PING_SENSOR_KEY,
            socket="COM",
            Hz=sensor_hz,
            rotation=[0.0, 90.0, 0.0],
            LaserMaxDistance=ping_max_range,
            LaserCount=1,
            LaserAngle=0.0,
            LaserDebug=False,
        ),
        Sensor.Collision(socket="CollisionSocket", Hz=sensor_hz),
        Sensor.IMU(socket="IMUSocket", Hz=sensor_hz),
        Sensor.Depth(socket="DepthSocket", Hz=sensor_hz),
    ]
    if include_ground_truth:
        sensors.extend(
            [
                Sensor.Pose(socket="COM", Hz=sensor_hz),
                Sensor.Velocity(socket="COM", Hz=sensor_hz),
            ]
        )
    return AgentConfig(
        agent_name="rov0",
        agent_type="BlueROV2",
        control_scheme=0,
        location=[initial_x, initial_y, initial_z],
        rotation=[0, 0, 0],
        sensors=[sensor.to_dict() for sensor in sensors],
    )


def parse_ping_altitude(raw_value: Any, max_valid_range: float) -> PingAltitudeReading:
    if raw_value is None:
        return PingAltitudeReading(raw_value=raw_value, altitude_m=None, valid=False, reason="missing")

    candidate = None
    if isinstance(raw_value, dict):
        for key in ("range", "Range", "altitude", "Altitude", "distance", "Distance"):
            if key in raw_value:
                candidate = raw_value[key]
                break
    else:
        candidate = raw_value

    try:
        values = np.asarray(candidate, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return PingAltitudeReading(
            raw_value=raw_value,
            altitude_m=None,
            valid=False,
            reason="not_numeric",
        )

    if values.size == 0:
        return PingAltitudeReading(raw_value=raw_value, altitude_m=None, valid=False, reason="empty")

    altitude = float(values[0])
    if not np.isfinite(altitude):
        return PingAltitudeReading(raw_value=raw_value, altitude_m=None, valid=False, reason="not_finite")
    if altitude < 0.0:
        return PingAltitudeReading(raw_value=raw_value, altitude_m=altitude, valid=False, reason="negative")
    if altitude > max_valid_range:
        return PingAltitudeReading(raw_value=raw_value, altitude_m=altitude, valid=False, reason="too_large")
    return PingAltitudeReading(raw_value=raw_value, altitude_m=altitude, valid=True)


def clipped_vertical_command(
    altitude_error: float,
    kp_altitude: float,
    max_vertical_command: float,
) -> float:
    raw_command = float(kp_altitude * altitude_error)
    return float(np.clip(raw_command, -max_vertical_command, max_vertical_command))


def horizontal_components_from_command(command: np.ndarray) -> tuple[float, float]:
    values = np.asarray(command, dtype=float).reshape(-1)
    if values.size < 8:
        return 0.0, 0.0
    forward = 0.25 * float(values[4] + values[5] + values[6] + values[7])
    lateral = 0.25 * float(values[4] - values[5] + values[6] - values[7])
    return forward, lateral


def collision_detected_from_state(state: dict) -> bool:
    if COLLISION_SENSOR_KEY not in state:
        return False
    value = state.get(COLLISION_SENSOR_KEY)
    try:
        values = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return bool(value)
    if values.size == 0:
        return False
    return bool(np.any(np.abs(values) > 1e-9))


def build_step_03_sample(
    state: dict,
    dvl_sample,
    time_s: float,
    step_index: int,
    args: argparse.Namespace,
    repetition: int,
    ping_reading: PingAltitudeReading,
    control_altitude: float | None,
    vertical_command: float,
    horizontal_command_x: float,
    horizontal_command_y: float,
    forward_progress: float,
    lateral_drift: float,
    target_reached: bool,
    collision: bool,
    invalid_ping: bool,
    unsafe_altitude: bool,
) -> dict:
    pose = pose_components(require_sensor(state, POSE_SENSOR_KEY))
    dvl_values = np.asarray(dvl_sample, dtype=float).reshape(-1)
    altitude = (
        float(control_altitude)
        if control_altitude is not None and np.isfinite(control_altitude)
        else math.nan
    )
    altitude_error = (
        float(args.desired_altitude - altitude)
        if np.isfinite(altitude)
        else math.nan
    )
    estimated_seabed_z = math.nan
    if np.isfinite(altitude):
        # Verified on the flat Step 3 SimpleUnderwater smoke runs:
        # pose_z - ping_altitude is approximately flat_seabed_z.
        estimated_seabed_z = float(pose["position"][2]) - altitude

    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "step_index": int(step_index),
        "sim_time": float(time_s),
        "controller_name": CONTROLLER_NAME,
        "target_distance": float(args.target_distance),
        "desired_altitude": float(args.desired_altitude),
        "current_y": float(args.current_y),
        "repetition": int(repetition),
        "pose_x": float(pose["position"][0]),
        "pose_y": float(pose["position"][1]),
        "pose_z": float(pose["position"][2]),
        "pose_forward_x": float(pose["forward"][0]),
        "pose_forward_y": float(pose["forward"][1]),
        "pose_forward_z": float(pose["forward"][2]),
        "pose_right_x": float(pose["right"][0]),
        "pose_right_y": float(pose["right"][1]),
        "pose_right_z": float(pose["right"][2]),
        "dvl_vx": float(dvl_values[0]) if dvl_values.size > 0 else math.nan,
        "dvl_vy": float(dvl_values[1]) if dvl_values.size > 1 else math.nan,
        "dvl_vz": float(dvl_values[2]) if dvl_values.size > 2 else math.nan,
        "ping_altitude_raw": raw_ping_to_csv_value(ping_reading.raw_value),
        "ping_altitude": altitude,
        "estimated_seabed_z_from_pose_ping": estimated_seabed_z,
        "altitude_error": altitude_error,
        "abs_altitude_error": abs(altitude_error) if np.isfinite(altitude_error) else math.nan,
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "vertical_command": float(vertical_command),
        "horizontal_command_x": float(horizontal_command_x),
        "horizontal_command_y": float(horizontal_command_y),
        "forward_progress": float(forward_progress),
        "lateral_drift": float(lateral_drift),
        "target_reached": bool(target_reached),
        "collision": bool(collision),
        "invalid_ping": bool(invalid_ping),
        "unsafe_altitude": bool(unsafe_altitude),
    }


def raw_ping_to_csv_value(raw_value: Any) -> str | float:
    try:
        values = np.asarray(raw_value, dtype=float).reshape(-1)
        if values.size == 1:
            return float(values[0])
    except (TypeError, ValueError):
        pass
    try:
        return json.dumps(raw_value)
    except TypeError:
        return str(raw_value)


def pose_progress_from_samples(first_sample: dict, state: dict) -> tuple[float, float]:
    pose = pose_components(require_sensor(state, POSE_SENSOR_KEY))
    current_position = np.asarray(pose["position"], dtype=float)
    start_position = np.array(
        [
            float(first_sample["pose_x"]),
            float(first_sample["pose_y"]),
            float(first_sample["pose_z"]),
        ],
        dtype=float,
    )
    start_forward = np.array(
        [
            float(first_sample.get("pose_forward_x", 1.0)),
            float(first_sample.get("pose_forward_y", 0.0)),
            float(first_sample.get("pose_forward_z", 0.0)),
        ],
        dtype=float,
    )
    start_right = np.array(
        [
            float(first_sample.get("pose_right_x", 0.0)),
            float(first_sample.get("pose_right_y", 1.0)),
            float(first_sample.get("pose_right_z", 0.0)),
        ],
        dtype=float,
    )
    displacement = current_position - start_position
    return float(np.dot(displacement, start_forward)), float(np.dot(displacement, start_right))


def build_step_03_summary(
    samples: Sequence[dict],
    args: argparse.Namespace,
    repetition: int,
    stop_reason: str,
    wall_clock_duration_s: float,
    invalid_ping_failure: bool,
    collision_detected: bool,
    unsafe_altitude_detected: bool,
    notes: Sequence[str],
    output_dir: Path,
) -> dict:
    if not samples:
        raise ValueError("Cannot summarize Step 3 run without samples.")

    last = samples[-1]
    altitude_errors = finite_values(sample["altitude_error"] for sample in samples)
    abs_altitude_errors = [abs(value) for value in altitude_errors]
    altitudes = finite_values(sample["ping_altitude"] for sample in samples)
    estimated_seabed_z_values = finite_values(
        sample["estimated_seabed_z_from_pose_ping"] for sample in samples
    )
    vertical_commands = finite_values(sample["vertical_command"] for sample in samples[1:])
    inside_band = [
        abs(float(sample["altitude_error"])) <= args.altitude_tolerance
        for sample in samples
        if is_finite_number(sample["altitude_error"])
    ]
    below_safe = [
        float(sample["ping_altitude"]) < args.min_safe_altitude
        for sample in samples
        if is_finite_number(sample["ping_altitude"])
    ]
    saturated = [
        abs(float(sample["vertical_command"])) >= args.max_vertical_command - 1e-9
        for sample in samples[1:]
        if is_finite_number(sample["vertical_command"])
    ]

    final_forward_progress = float(last["forward_progress"])
    target_reached = stop_reason == "target_reached"
    timeout = stop_reason == "timeout"

    return {
        "controller_name": CONTROLLER_NAME,
        "target_distance": float(args.target_distance),
        "desired_altitude": float(args.desired_altitude),
        "current_y": float(args.current_y),
        "repetition": int(repetition),
        "target_reached": bool(target_reached),
        "timeout": bool(timeout),
        "collision": bool(collision_detected),
        "unsafe_altitude": bool(unsafe_altitude_detected),
        "invalid_ping_failure": bool(invalid_ping_failure),
        "runtime_seconds": float(last["sim_time"]),
        "elapsed_wall_clock_time_s": float(wall_clock_duration_s),
        "final_forward_progress": final_forward_progress,
        "final_forward_error": float(args.target_distance - final_forward_progress),
        "final_lateral_drift": float(last["lateral_drift"]),
        "mean_altitude_error": mean_value(altitude_errors),
        "mean_abs_altitude_error": mean_value(abs_altitude_errors),
        "rmse_altitude_error": rmse_value(altitude_errors),
        "max_abs_altitude_error": max(abs_altitude_errors, default=0.0),
        "final_altitude_error": (
            float(last["altitude_error"])
            if is_finite_number(last["altitude_error"])
            else math.nan
        ),
        "min_altitude": min(altitudes, default=math.nan),
        "max_altitude": max(altitudes, default=math.nan),
        "estimated_seabed_z_min": min(estimated_seabed_z_values, default=math.nan),
        "estimated_seabed_z_max": max(estimated_seabed_z_values, default=math.nan),
        "estimated_seabed_z_range": (
            max(estimated_seabed_z_values) - min(estimated_seabed_z_values)
            if estimated_seabed_z_values
            else math.nan
        ),
        "estimated_seabed_z_std": std_value(estimated_seabed_z_values),
        "time_inside_altitude_band_percent": percent_true(inside_band),
        "time_below_safe_altitude_percent": percent_true(below_safe),
        "vertical_command_mean_abs": mean_value([abs(value) for value in vertical_commands]),
        "vertical_command_max_abs": max([abs(value) for value in vertical_commands], default=0.0),
        "vertical_saturation_percent": percent_true(saturated),
        "notes": "; ".join(notes),
        "stop_reason": stop_reason,
        "output_dir": str(output_dir),
        "sample_count": len(samples),
        "altitude_tolerance": float(args.altitude_tolerance),
        "min_safe_altitude": float(args.min_safe_altitude),
        "kp_altitude": float(args.kp_altitude),
        "max_vertical_command": float(args.max_vertical_command),
    }


def validate_run_args(args: argparse.Namespace) -> None:
    if args.target_distance <= 0.0:
        raise ValueError("target-distance must be positive.")
    if args.desired_altitude <= 0.0:
        raise ValueError("desired-altitude must be positive.")
    if args.altitude_tolerance <= 0.0:
        raise ValueError("altitude-tolerance must be positive.")
    if args.min_safe_altitude <= 0.0:
        raise ValueError("min-safe-altitude must be positive.")
    if args.desired_altitude <= args.min_safe_altitude:
        raise ValueError("desired-altitude must be greater than min-safe-altitude.")
    if args.kp_altitude < 0.0:
        raise ValueError("kp-altitude must be non-negative.")
    if args.max_vertical_command <= 0.0:
        raise ValueError("max-vertical-command must be positive.")
    if args.max_duration <= 0.0:
        raise ValueError("max-duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.max_invalid_ping_hold_s < 0.0:
        raise ValueError("max-invalid-ping-hold-s must be zero or positive.")
    if args.ping_max_range <= args.desired_altitude:
        raise ValueError("ping-max-range must be greater than desired-altitude.")
    if not np.isfinite(float(args.initial_x)):
        raise ValueError("initial-x must be finite.")
    if not np.isfinite(float(getattr(args, "initial_y", DEFAULT_INITIAL_Y))):
        raise ValueError("initial-y must be finite.")
    if getattr(args, "initial_z", None) is not None and not np.isfinite(float(args.initial_z)):
        raise ValueError("initial-z must be finite when provided.")


def run_step_03_batch(args: argparse.Namespace) -> dict:
    validate_batch_args(args)
    batch_dir = args.resume_dir if args.resume_dir is not None else (
        args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    batch_dir.mkdir(parents=True, exist_ok=True)
    metadata = build_batch_metadata(args, batch_dir)
    write_json(batch_dir / "metadata.json", metadata)
    write_yaml_mapping(batch_dir / "metadata.yaml", metadata)
    logs_path = batch_dir / "logs.txt"

    expected_runs = build_expected_step_03_runs(args)
    rows = load_existing_summary_rows(batch_dir)
    completed_run_ids = {row["run_id"] for row in rows if row.get("run_id")}

    print(f"Saving Step 3 altitude-hold batch outputs to: {batch_dir}")
    print(f"Expected runs: {len(expected_runs)}")
    print(f"Batch type: {args.batch_type}")

    for index, run_spec in enumerate(expected_runs, start=1):
        run_id = run_spec["run_id"]
        if run_id in completed_run_ids:
            print(f"[{index}/{len(expected_runs)}] skipping completed {run_id}")
            continue

        print(
            f"[{index}/{len(expected_runs)}] target={run_spec['target_distance']:g} m, "
            f"altitude={run_spec['desired_altitude']:g} m, "
            f"current_y={run_spec['current_y']:g} m/s, "
            f"rep={run_spec['repetition']}"
        )
        append_log(
            logs_path,
            f"START {run_id} {datetime.now(timezone.utc).isoformat()}",
        )
        run_args = build_run_args_from_batch(args, run_spec)
        run_dir = batch_dir / run_id
        try:
            summary = run_step_03_altitude_hold_experiment(
                run_args,
                output_dir=run_dir,
                print_terminal_summary=False,
            )
        except Exception as exc:
            summary = failed_summary_from_exception(run_args, run_dir, exc)
            run_dir.mkdir(parents=True, exist_ok=True)
            write_json(run_dir / "summary.json", summary)
            append_log(logs_path, f"FAIL {run_id}: {exc}")
        else:
            append_log(
                logs_path,
                f"END {run_id} stop_reason={summary['stop_reason']}",
            )

        row = dict(summary)
        row["run_id"] = run_id
        rows.append(row)
        write_summary_csv(batch_dir / "summary.csv", rows)
        aggregate_rows = build_step_03_aggregate(rows)
        write_aggregate_csv(batch_dir / "aggregate_by_condition.csv", aggregate_rows)

        if args.run_settle_delay > 0.0:
            time.sleep(args.run_settle_delay)

    write_summary_csv(batch_dir / "summary.csv", rows)
    aggregate_rows = build_step_03_aggregate(rows)
    write_aggregate_csv(batch_dir / "aggregate_by_condition.csv", aggregate_rows)
    write_plots_for_batch(batch_dir, rows)

    print_step_03_aggregate_table(aggregate_rows)
    return {
        "batch_dir": str(batch_dir),
        "expected_runs": len(expected_runs),
        "completed_runs": len(rows),
        "aggregate_rows": aggregate_rows,
        "summary_rows": rows,
    }


def build_expected_step_03_runs(args: argparse.Namespace) -> list[dict]:
    runs = []
    for target_distance in args.target_distances:
        for desired_altitude in args.desired_altitudes:
            for current_y in args.current_y_values:
                for repetition in range(1, args.repetitions + 1):
                    runs.append(
                        {
                            "run_id": make_step_03_run_id(
                                target_distance,
                                desired_altitude,
                                current_y,
                                repetition,
                            ),
                            "target_distance": float(target_distance),
                            "desired_altitude": float(desired_altitude),
                            "current_y": float(current_y),
                            "repetition": int(repetition),
                        }
                    )
    return runs


def build_run_args_from_batch(args: argparse.Namespace, run_spec: dict) -> argparse.Namespace:
    return argparse.Namespace(
        target_distance=run_spec["target_distance"],
        desired_altitude=run_spec["desired_altitude"],
        altitude_tolerance=args.altitude_tolerance,
        min_safe_altitude=args.min_safe_altitude,
        desired_forward_velocity=args.desired_forward_velocity,
        desired_lateral_velocity=args.desired_lateral_velocity,
        current_x=args.current_x,
        current_y=run_spec["current_y"],
        current_z=args.current_z,
        repetition=run_spec["repetition"],
        kp_altitude=args.kp_altitude,
        max_vertical_command=args.max_vertical_command,
        kp_forward=args.kp_forward,
        kp_lateral=args.kp_lateral,
        max_forward_command=args.max_forward_command,
        max_lateral_command=args.max_lateral_command,
        max_thruster_command=args.max_thruster_command,
        max_duration=args.max_duration,
        ticks_per_sec=args.ticks_per_sec,
        warmup_ticks=args.warmup_ticks,
        dvl_forward_index=args.dvl_forward_index,
        dvl_forward_sign=args.dvl_forward_sign,
        dvl_lateral_index=args.dvl_lateral_index,
        dvl_lateral_sign=args.dvl_lateral_sign,
        initial_x=args.initial_x,
        initial_y=getattr(args, "initial_y", DEFAULT_INITIAL_Y),
        initial_z=getattr(args, "initial_z", None),
        flat_seabed_z=args.flat_seabed_z,
        ping_max_range=args.ping_max_range,
        max_invalid_ping_hold_s=args.max_invalid_ping_hold_s,
        world=args.world,
        show_viewport=args.show_viewport,
        results_dir=args.results_dir,
    )


def build_step_03_aggregate(rows: Sequence[dict]) -> list[dict]:
    grouped: dict[tuple[float, float, float, str], list[dict]] = defaultdict(list)
    for row in rows:
        grouped[
            (
                float(row["target_distance"]),
                float(row["desired_altitude"]),
                float(row["current_y"]),
                str(row["controller_name"]),
            )
        ].append(row)

    aggregate_rows = []
    for key in sorted(grouped):
        target_distance, desired_altitude, current_y, controller_name = key
        group_rows = grouped[key]
        aggregate_rows.append(
            {
                "target_distance": target_distance,
                "desired_altitude": desired_altitude,
                "current_y": current_y,
                "controller_name": controller_name,
                "number_of_runs": len(group_rows),
                "target_reached_rate": rate(group_rows, "target_reached"),
                "timeout_rate": rate(group_rows, "timeout"),
                "collision_rate": rate(group_rows, "collision"),
                "unsafe_altitude_rate": rate(group_rows, "unsafe_altitude"),
                "invalid_ping_failure_rate": rate(group_rows, "invalid_ping_failure"),
                "final_lateral_drift_mean": mean_row_metric(group_rows, "final_lateral_drift"),
                "final_lateral_drift_std": std_row_metric(group_rows, "final_lateral_drift"),
                "mean_abs_altitude_error_mean": mean_row_metric(group_rows, "mean_abs_altitude_error"),
                "mean_abs_altitude_error_std": std_row_metric(group_rows, "mean_abs_altitude_error"),
                "rmse_altitude_error_mean": mean_row_metric(group_rows, "rmse_altitude_error"),
                "rmse_altitude_error_std": std_row_metric(group_rows, "rmse_altitude_error"),
                "max_abs_altitude_error_mean": mean_row_metric(group_rows, "max_abs_altitude_error"),
                "max_abs_altitude_error_std": std_row_metric(group_rows, "max_abs_altitude_error"),
                "time_inside_altitude_band_percent_mean": mean_row_metric(
                    group_rows,
                    "time_inside_altitude_band_percent",
                ),
                "time_inside_altitude_band_percent_std": std_row_metric(
                    group_rows,
                    "time_inside_altitude_band_percent",
                ),
                "runtime_seconds_mean": mean_row_metric(group_rows, "runtime_seconds"),
                "runtime_seconds_std": std_row_metric(group_rows, "runtime_seconds"),
                "vertical_saturation_percent_mean": mean_row_metric(
                    group_rows,
                    "vertical_saturation_percent",
                ),
                "vertical_saturation_percent_std": std_row_metric(
                    group_rows,
                    "vertical_saturation_percent",
                ),
            }
        )
    return aggregate_rows


def write_plots_for_batch(batch_dir: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    try:
        representative_paths = representative_trajectory_paths(batch_dir, rows)
        if representative_paths:
            plot_representative_altitude_runs(
                representative_paths,
                batch_dir / "altitude_representative_runs.png",
            )
            plot_representative_altitude_error_runs(
                representative_paths,
                batch_dir / "altitude_error_representative_runs.png",
            )
        plot_summary_metric_vs_target(
            rows,
            "final_lateral_drift",
            "Final lateral drift [m]",
            batch_dir / "final_lateral_drift_vs_target.png",
        )
        plot_summary_metric_vs_target(
            rows,
            "rmse_altitude_error",
            "RMSE altitude error [m]",
            batch_dir / "rmse_altitude_error_vs_target.png",
        )
        plot_summary_metric_vs_target(
            rows,
            "time_inside_altitude_band_percent",
            "Time inside altitude band [%]",
            batch_dir / "time_inside_altitude_band_vs_target.png",
        )
    except Exception as exc:
        append_log(batch_dir / "logs.txt", f"Plot generation failed: {exc}")


def representative_trajectory_paths(batch_dir: Path, rows: Sequence[dict]) -> list[Path]:
    selected = []
    seen_current = set()
    sorted_rows = sorted(
        rows,
        key=lambda row: (
            -float(row["target_distance"]),
            float(row["current_y"]),
            int(row["repetition"]),
        ),
    )
    for row in sorted_rows:
        current_y = float(row["current_y"])
        if current_y in seen_current:
            continue
        run_id = row.get("run_id") or Path(str(row["output_dir"])).name
        path = batch_dir / run_id / "trajectory.csv"
        if path.exists():
            selected.append(path)
            seen_current.add(current_y)
    return selected


def load_existing_summary_rows(batch_dir: Path) -> list[dict]:
    path = batch_dir / "summary.csv"
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def write_summary_csv(path: Path, rows: Sequence[dict]) -> None:
    fieldnames = ["run_id"] + SUMMARY_FIELDNAMES
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_aggregate_csv(path: Path, rows: Sequence[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=AGGREGATE_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def failed_summary_from_exception(
    args: argparse.Namespace,
    output_dir: Path,
    exc: Exception,
) -> dict:
    return {
        "controller_name": CONTROLLER_NAME,
        "target_distance": float(args.target_distance),
        "desired_altitude": float(args.desired_altitude),
        "current_y": float(args.current_y),
        "repetition": int(args.repetition),
        "target_reached": False,
        "timeout": False,
        "collision": False,
        "unsafe_altitude": False,
        "invalid_ping_failure": False,
        "runtime_seconds": 0.0,
        "final_forward_progress": 0.0,
        "final_forward_error": float(args.target_distance),
        "final_lateral_drift": 0.0,
        "mean_altitude_error": 0.0,
        "mean_abs_altitude_error": 0.0,
        "rmse_altitude_error": 0.0,
        "max_abs_altitude_error": 0.0,
        "final_altitude_error": 0.0,
        "min_altitude": 0.0,
        "max_altitude": 0.0,
        "estimated_seabed_z_min": 0.0,
        "estimated_seabed_z_max": 0.0,
        "estimated_seabed_z_range": 0.0,
        "estimated_seabed_z_std": 0.0,
        "time_inside_altitude_band_percent": 0.0,
        "time_below_safe_altitude_percent": 0.0,
        "vertical_command_mean_abs": 0.0,
        "vertical_command_max_abs": 0.0,
        "vertical_saturation_percent": 0.0,
        "notes": f"Run failed with exception: {exc}",
        "stop_reason": "exception",
        "output_dir": str(output_dir),
    }


def validate_batch_args(args: argparse.Namespace) -> None:
    if not args.target_distances:
        raise ValueError("At least one target distance is required.")
    if not args.desired_altitudes:
        raise ValueError("At least one desired altitude is required.")
    if not args.current_y_values:
        raise ValueError("At least one current-y value is required.")
    if args.repetitions <= 0:
        raise ValueError("repetitions must be positive.")
    for target_distance in args.target_distances:
        if target_distance <= 0.0:
            raise ValueError("All target distances must be positive.")
    for desired_altitude in args.desired_altitudes:
        if desired_altitude <= args.min_safe_altitude:
            raise ValueError("All desired altitudes must exceed min-safe-altitude.")


def build_batch_metadata(args: argparse.Namespace, batch_dir: Path) -> dict:
    return {
        "step": "Step 3",
        "title": "Forward motion while maintaining seabed-relative altitude",
        "scientific_goal": (
            "Advance by the requested distance while keeping bottom-relative "
            "altitude close to desired_altitude."
        ),
        "output_dir": str(batch_dir),
        "batch_type": args.batch_type,
        "controller_name": CONTROLLER_NAME,
        "target_distances": [float(value) for value in args.target_distances],
        "desired_altitudes": [float(value) for value in args.desired_altitudes],
        "current_y_values": [float(value) for value in args.current_y_values],
        "repetitions": int(args.repetitions),
        "desired_forward_velocity_mps": float(args.desired_forward_velocity),
        "desired_lateral_velocity_mps": float(args.desired_lateral_velocity),
        "altitude_tolerance_m": float(args.altitude_tolerance),
        "min_safe_altitude_m": float(args.min_safe_altitude),
        "kp_altitude": float(args.kp_altitude),
        "max_vertical_command": float(args.max_vertical_command),
        "kp_forward": float(args.kp_forward),
        "kp_lateral": float(args.kp_lateral),
        "max_forward_command": float(args.max_forward_command),
        "max_lateral_command": float(args.max_lateral_command),
        "max_thruster_command": float(args.max_thruster_command),
        "max_duration_s": float(args.max_duration),
        "ticks_per_sec": int(args.ticks_per_sec),
        "warmup_ticks": int(args.warmup_ticks),
        "dvl_forward_index": int(args.dvl_forward_index),
        "dvl_forward_sign": float(args.dvl_forward_sign),
        "dvl_lateral_index": int(args.dvl_lateral_index),
        "dvl_lateral_sign": float(args.dvl_lateral_sign),
        "initial_x": float(args.initial_x),
        "initial_y": float(getattr(args, "initial_y", DEFAULT_INITIAL_Y)),
        "initial_z": (
            float(args.initial_z)
            if getattr(args, "initial_z", None) is not None
            else None
        ),
        "flat_seabed_z": float(args.flat_seabed_z),
        "world": args.world,
        "pose_policy": "Pose is evaluation ground truth only and is never used for altitude control.",
        "vertical_control_sensor": "PingAltimeter RangeFinderSensor",
        "validated_envelope_note": (
            "The main batch excludes current_y=2.0 m/s based on Step 2 results. "
            "Use the stress-test batch for currents outside the main envelope."
        ),
    }


def make_step_03_run_id(
    target_distance: float,
    desired_altitude: float,
    current_y: float,
    repetition: int,
) -> str:
    return (
        f"target_{format_float(target_distance)}m_"
        f"alt_{format_float(desired_altitude)}m_"
        f"current_y_{format_float(current_y)}_"
        f"rep_{repetition:02d}"
    )


def format_float(value: float) -> str:
    return f"{value:g}".replace("-", "neg").replace(".", "p")


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


def write_yaml_mapping(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        for key, value in data.items():
            file.write(f"{key}: {json.dumps(value)}\n")


def append_log(path: Path, line: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(line)
        file.write("\n")


def finite_values(values: Any) -> list[float]:
    result = []
    for value in values:
        if is_finite_number(value):
            result.append(float(value))
    return result


def is_finite_number(value: Any) -> bool:
    try:
        return bool(np.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def mean_value(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return float(np.mean([float(value) for value in values]))


def std_value(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    return float(np.std([float(value) for value in values], ddof=0))


def rmse_value(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(array * array)))


def percent_true(values: Sequence[bool]) -> float:
    if not values:
        return 0.0
    return float(100.0 * np.mean([1.0 if value else 0.0 for value in values]))


def rate(rows: Sequence[dict], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([1.0 if truthy(row.get(key, False)) else 0.0 for row in rows]))


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes")
    return bool(value)


def mean_row_metric(rows: Sequence[dict], key: str) -> float:
    return mean_value([float(row[key]) for row in rows if is_finite_number(row.get(key))])


def std_row_metric(rows: Sequence[dict], key: str) -> float:
    return std_value([float(row[key]) for row in rows if is_finite_number(row.get(key))])


def print_step_03_summary(summary: dict) -> None:
    print("\nStep 3 altitude-hold summary")
    print(f"Target distance: {summary['target_distance']:.3f} m")
    print(f"Desired altitude: {summary['desired_altitude']:.3f} m")
    print(f"Current y: {summary['current_y']:.3f} m/s")
    print(f"Target reached: {summary['target_reached']}")
    print(f"Stop reason: {summary['stop_reason']}")
    print(f"Runtime: {summary['runtime_seconds']:.2f} s")
    print(f"Final forward progress: {summary['final_forward_progress']:.3f} m")
    print(f"Final lateral drift: {summary['final_lateral_drift']:.3f} m")
    print(f"RMSE altitude error: {summary['rmse_altitude_error']:.3f} m")
    print(f"Max abs altitude error: {summary['max_abs_altitude_error']:.3f} m")
    print(
        "Time inside altitude band: "
        f"{summary['time_inside_altitude_band_percent']:.1f}%"
    )
    print(f"Results directory: {summary['output_dir']}")


def print_step_03_aggregate_table(aggregate_rows: Sequence[dict]) -> None:
    print("\nStep 3 aggregate by condition")
    print(
        "target  alt  current_y  reached  timeout  RMSE_alt  max_abs_alt  "
        "inside_band  drift"
    )
    for row in aggregate_rows:
        print(
            f"{row['target_distance']:>6g} "
            f"{row['desired_altitude']:>4g} "
            f"{row['current_y']:>9g} "
            f"{row['target_reached_rate']:>7.2f} "
            f"{row['timeout_rate']:>7.2f} "
            f"{row['rmse_altitude_error_mean']:>8.3f} "
            f"{row['max_abs_altitude_error_mean']:>11.3f} "
            f"{row['time_inside_altitude_band_percent_mean']:>11.1f} "
            f"{row['final_lateral_drift_mean']:>7.3f}"
        )
