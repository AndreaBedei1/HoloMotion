from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.dvl_velocity_controller import DVLVelocityTrackingController
from estimators.dvl_distance import DVLDistanceEstimator
from experiment_logging.experiment_logger import ExperimentLogger
from experiments.forward_distance import (
    DEPTH_SENSOR_KEY,
    DVL_SENSOR_KEY,
    POSE_SENSOR_KEY,
    VELOCITY_SENSOR_KEY,
    add_pose_evaluation_fields,
    add_summary_aliases,
    apply_ocean_current,
    build_distance_validation,
    build_dvl_pose_sanity_check,
    extract_state_time_s,
    mean_value,
    optional_scalar,
    optional_vector,
    pose_components,
    print_warnings,
    require_sensor,
    run_warmup,
    std_value,
)
from lib.current import current_config_from_args
from lib.rover import Rover
from lib.scenario_builder import ScenarioConfig
from metrics.distance import compute_distance_metrics
from visualization.distance_plots import (
    plot_distance_results,
    plot_lateral_drift,
    plot_trajectory,
)
from visualization.velocity_tracking_plots import (
    plot_command_history,
    plot_velocity_tracking,
)


DEFAULT_DESIRED_FORWARD_VELOCITY = 0.3
DEFAULT_DESIRED_LATERAL_VELOCITY = 0.0
DEFAULT_FORWARD_KP = 20.0
DEFAULT_LATERAL_KP = 12.0
DEFAULT_VELOCITY_KP = DEFAULT_FORWARD_KP
DEFAULT_MAX_COMMAND = 2.0


def run_dvl_velocity_compensation_experiment(
    args: argparse.Namespace,
    output_dir: Path | None = None,
    print_terminal_summary: bool = True,
) -> dict:
    try:
        import holoocean
    except ImportError as exc:
        raise RuntimeError(
            "HoloOcean is not installed in this Python environment. "
            "Install HoloOcean 2.2.2 before running the live example."
        ) from exc

    validate_args(args)

    fallback_dt_s = 1.0 / float(args.ticks_per_sec)
    logger = (
        ExperimentLogger(output_dir)
        if output_dir is not None
        else ExperimentLogger.create_timestamped(args.results_dir)
    )
    show_viewport = bool(getattr(args, "show_viewport", True))
    current_config = current_config_from_args(args)
    current_vector = current_config.as_list()
    current_enabled = current_config.enabled
    current_application_mode = "every_step" if current_enabled else "none"
    current_application_calls = 0

    controller = DVLVelocityTrackingController(
        kp_forward=args.kp_forward,
        kp_lateral=args.kp_lateral,
        max_forward_command=args.max_forward_command,
        max_lateral_command=args.max_lateral_command,
        base_vertical_command=args.base_vertical_command,
    )
    estimator = DVLDistanceEstimator(
        forward_axis_index=args.dvl_forward_index,
        forward_axis_sign=args.dvl_forward_sign,
    )

    rov = Rover.BlueROV2Navigation(
        name="rov0",
        location=[0, 0, -4],
        rotation=[0, 0, 0],
        sensor_hz=args.ticks_per_sec,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step02B_DVL_Velocity_Compensation")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(rov)
    )

    run_config = {
        "target_distance_m": float(args.target_distance),
        "desired_forward_velocity_mps": float(args.desired_forward_velocity),
        "desired_lateral_velocity_mps": float(args.desired_lateral_velocity),
        "kp_forward": float(args.kp_forward),
        "kp_lateral": float(args.kp_lateral),
        "max_forward_command": float(args.max_forward_command),
        "max_lateral_command": float(args.max_lateral_command),
        "base_vertical_command": float(args.base_vertical_command),
        "ticks_per_sec": int(args.ticks_per_sec),
        "max_duration_s": float(args.max_duration),
        "warmup_ticks": int(args.warmup_ticks),
        "world": args.world,
        "dvl_forward_index": int(args.dvl_forward_index),
        "dvl_forward_sign": float(args.dvl_forward_sign),
        "dvl_lateral_index": int(args.dvl_lateral_index),
        "dvl_lateral_sign": float(args.dvl_lateral_sign),
        "show_viewport": show_viewport,
        "current_x": float(current_vector[0]),
        "current_y": float(current_vector[1]),
        "current_z": float(current_vector[2]),
        "current_magnitude": current_config.magnitude,
        "current_application_mode": current_application_mode,
        "controller_type": "DVLVelocityTrackingController",
        "controller_policy": (
            "The controller tracks desired body-frame DVL velocities. It does "
            "not use the configured HoloOcean current vector."
        ),
        "sensor_policy": {
            "DVL": "used for forward distance estimation, stopping, and velocity tracking",
            "Pose": "logged during the run and used after the run for evaluation only",
            "Velocity": "logged during the run and used after the run for evaluation only",
            "IMU": "available for logging/future work, not required by this controller",
            "Depth": "available for future depth control, not used by this controller",
        },
    }
    logger.write_run_config(run_config)

    samples: list[dict] = []
    dt_samples: list[float] = []
    dt_fallback_count = 0
    stop_reason = "timeout"

    print(f"Saving Step 2B DVL velocity compensation outputs to: {logger.output_dir}")
    print(
        "Desired body velocity: "
        f"forward={args.desired_forward_velocity:.3f} m/s, "
        f"lateral={args.desired_lateral_velocity:.3f} m/s"
    )
    print(
        "Using DVL forward velocity "
        f"index {args.dvl_forward_index} with sign {args.dvl_forward_sign:+.0f}; "
        "DVL lateral velocity "
        f"index {args.dvl_lateral_index} with sign {args.dvl_lateral_sign:+.0f}."
    )

    with holoocean.make(
        scenario_cfg=scenario.to_dict(),
        show_viewport=show_viewport,
        ticks_per_sec=args.ticks_per_sec,
        frames_per_sec=args.ticks_per_sec,
        start_world=True,
    ) as env:
        stop_command = controller.stop_command()
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

        state_time = extract_state_time_s(state)
        use_state_time = state_time is not None
        time_source = "simulator_state_time" if use_state_time else "wall_clock"
        last_time_reference = state_time if use_state_time else time.perf_counter()
        elapsed_s = 0.0
        wall_clock_start = time.perf_counter()
        dvl_sample = require_sensor(state, DVL_SENSOR_KEY)

        samples.append(
            build_velocity_tracking_sample(
                state=state,
                estimator=estimator,
                time_s=elapsed_s,
                command=stop_command,
                dvl_sample=dvl_sample,
                args=args,
                current_vector=current_vector if current_enabled else None,
            )
        )

        while elapsed_s < args.max_duration:
            measured_forward = estimator.velocity_components(dvl_sample)[1]
            measured_lateral = dvl_velocity_components(
                dvl_sample,
                args.dvl_lateral_index,
                args.dvl_lateral_sign,
            )[1]
            command = controller.command(
                desired_forward_velocity=args.desired_forward_velocity,
                desired_lateral_velocity=args.desired_lateral_velocity,
                measured_forward_velocity=measured_forward,
                measured_lateral_velocity=measured_lateral,
            )

            if current_enabled:
                apply_ocean_current(env, "rov0", current_vector)
                current_application_calls += 1
            state = env.step(command)

            current_reference = (
                extract_state_time_s(state) if use_state_time else time.perf_counter()
            )
            if current_reference is None:
                dt_s = fallback_dt_s
                dt_fallback_count += 1
            else:
                dt_s = current_reference - last_time_reference
                if not np.isfinite(dt_s) or dt_s <= 0.0:
                    dt_s = fallback_dt_s
                    dt_fallback_count += 1
                last_time_reference = current_reference

            elapsed_s += dt_s
            dt_samples.append(dt_s)
            dvl_sample = require_sensor(state, DVL_SENSOR_KEY)
            estimator.update(dvl_sample, dt_s)
            samples.append(
                build_velocity_tracking_sample(
                    state=state,
                    estimator=estimator,
                    time_s=elapsed_s,
                    command=command,
                    dvl_sample=dvl_sample,
                    args=args,
                    current_vector=current_vector if current_enabled else None,
                )
            )

            if estimator.distance_m >= args.target_distance:
                stop_reason = "target_reached"
                break

        wall_clock_duration_s = time.perf_counter() - wall_clock_start
        if current_enabled:
            apply_ocean_current(env, "rov0", current_vector)
            current_application_calls += 1
        env.step(stop_command)

    add_pose_evaluation_fields(samples)
    metrics = compute_distance_metrics(samples, args.target_distance, stop_reason)
    distance_validation = build_distance_validation(
        samples=samples,
        metrics=metrics.to_dict(),
        wall_clock_duration_s=wall_clock_duration_s,
        speed_warning_threshold=args.speed_warning_threshold,
        max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
    )

    summary = metrics.to_dict()
    summary.update(
        {
            "current_x": float(current_vector[0]),
            "current_y": float(current_vector[1]),
            "current_z": float(current_vector[2]),
            "current_magnitude": current_config.magnitude,
            "current_application_mode": current_application_mode,
            "current_application_calls": current_application_calls,
            "desired_forward_velocity": float(args.desired_forward_velocity),
            "desired_lateral_velocity": float(args.desired_lateral_velocity),
            "kp_forward": float(args.kp_forward),
            "kp_lateral": float(args.kp_lateral),
            "max_forward_command": float(args.max_forward_command),
            "max_lateral_command": float(args.max_lateral_command),
            "dvl_forward_index": int(args.dvl_forward_index),
            "dvl_forward_sign": float(args.dvl_forward_sign),
            "dvl_lateral_index": int(args.dvl_lateral_index),
            "dvl_lateral_sign": float(args.dvl_lateral_sign),
            "elapsed_wall_clock_time_s": wall_clock_duration_s,
            "time_source": time_source,
            "dt_source": time_source,
            "mean_dt": mean_value(dt_samples),
            "std_dt": std_value(dt_samples),
            "dt_fallback_count": dt_fallback_count,
            "initial_pose": distance_validation["initial_pose"],
            "final_pose": distance_validation["final_pose"],
            "average_dvl_speed_mps": distance_validation["average_dvl_speed_mps"],
            "average_pose_speed": distance_validation["average_pose_speed_mps"],
            "average_pose_speed_mps": distance_validation["average_pose_speed_mps"],
            "mean_dvl_forward_velocity_mps": distance_validation[
                "mean_dvl_forward_velocity_mps"
            ],
            "max_dvl_forward_velocity_mps": distance_validation[
                "max_dvl_forward_velocity_mps"
            ],
            "controller_type": "DVLVelocityTrackingController",
            "output_dir": str(logger.output_dir),
            "distance_validation_warnings": distance_validation["warnings"],
            "dvl_pose_sanity_check": build_dvl_pose_sanity_check(metrics.to_dict()),
        }
    )
    summary.update(build_velocity_tracking_metrics(samples))
    add_summary_aliases(summary)

    logger.write_trajectory(samples)
    logger.write_summary(summary)
    plot_distance_results(samples, args.target_distance, logger.output_dir / "distance_plot.png")
    plot_trajectory(samples, logger.output_dir / "trajectory_plot.png")
    plot_lateral_drift(samples, logger.output_dir / "lateral_drift_plot.png")
    plot_velocity_tracking(samples, logger.output_dir / "velocity_tracking_plot.png")
    plot_command_history(samples, logger.output_dir / "command_plot.png")

    if print_terminal_summary:
        print_summary(summary)
    else:
        print_warnings(summary)

    return summary


def validate_args(args: argparse.Namespace) -> None:
    if args.target_distance <= 0:
        raise ValueError("target-distance must be positive.")
    if args.max_duration <= 0:
        raise ValueError("max-duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.dvl_forward_index < 0:
        raise ValueError("dvl-forward-index must be zero or positive.")
    if args.dvl_lateral_index < 0:
        raise ValueError("dvl-lateral-index must be zero or positive.")
    if args.dvl_forward_sign not in (-1.0, 1.0):
        raise ValueError("dvl-forward-sign must be +1 or -1.")
    if args.dvl_lateral_sign not in (-1.0, 1.0):
        raise ValueError("dvl-lateral-sign must be +1 or -1.")
    if args.kp_forward < 0 or args.kp_lateral < 0:
        raise ValueError("controller gains must be non-negative.")
    if args.max_forward_command < 0 or args.max_lateral_command < 0:
        raise ValueError("max command values must be non-negative.")
    if args.speed_warning_threshold <= 0:
        raise ValueError("speed-warning-threshold must be positive.")
    if args.max_dvl_speed_warning_threshold <= 0:
        raise ValueError("max-dvl-speed-warning-threshold must be positive.")


def build_velocity_tracking_sample(
    state: dict,
    estimator: DVLDistanceEstimator,
    time_s: float,
    command: np.ndarray,
    dvl_sample,
    args: argparse.Namespace,
    current_vector: list[float] | None = None,
) -> dict:
    pose = pose_components(require_sensor(state, POSE_SENSOR_KEY))
    velocity = optional_vector(state.get(VELOCITY_SENSOR_KEY), 3)
    depth = optional_scalar(state.get(DEPTH_SENSOR_KEY))
    raw_forward, used_forward = estimator.velocity_components(dvl_sample)
    raw_lateral, used_lateral = dvl_velocity_components(
        dvl_sample,
        args.dvl_lateral_index,
        args.dvl_lateral_sign,
    )
    forward_error = float(args.desired_forward_velocity - used_forward)
    lateral_error = float(args.desired_lateral_velocity - used_lateral)
    current = current_vector or [0.0, 0.0, 0.0]
    current_norm = float(np.linalg.norm(np.asarray(current, dtype=float)))

    sample = {
        "time": float(time_s),
        "current_x": float(current[0]),
        "current_y": float(current[1]),
        "current_z": float(current[2]),
        "current_magnitude": current_norm,
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "dvl_forward_velocity_raw": raw_forward,
        "dvl_forward_velocity_used": used_forward,
        "dvl_lateral_velocity_raw": raw_lateral,
        "dvl_lateral_velocity_used": used_lateral,
        "forward_velocity_error": forward_error,
        "lateral_velocity_error": lateral_error,
        "dvl_distance_estimated": float(estimator.distance_m),
        "pose_x": float(pose["position"][0]),
        "pose_y": float(pose["position"][1]),
        "pose_z": float(pose["position"][2]),
        "pose_forward_x": float(pose["forward"][0]),
        "pose_forward_y": float(pose["forward"][1]),
        "pose_forward_z": float(pose["forward"][2]),
        "pose_right_x": float(pose["right"][0]),
        "pose_right_y": float(pose["right"][1]),
        "pose_right_z": float(pose["right"][2]),
        "pose_forward_displacement": 0.0,
        "pose_lateral_drift": 0.0,
        "pose_ground_truth_displacement": 0.0,
        "pose_euclidean_displacement": 0.0,
        "gt_velocity_x": velocity[0] if velocity is not None else "",
        "gt_velocity_y": velocity[1] if velocity is not None else "",
        "gt_velocity_z": velocity[2] if velocity is not None else "",
        "depth_m": depth if depth is not None else "",
    }

    command_values = np.asarray(command, dtype=float).reshape(-1)
    for index in range(8):
        sample[f"cmd_{index}"] = (
            float(command_values[index]) if index < command_values.size else 0.0
        )
    return sample


def dvl_velocity_components(
    dvl_sample,
    axis_index: int,
    axis_sign: float,
) -> tuple[float, float]:
    values = np.asarray(dvl_sample, dtype=float).reshape(-1)
    if values.size <= axis_index:
        raise ValueError(
            f"DVL sample does not contain requested velocity axis {axis_index}. "
            f"Shape: {np.asarray(dvl_sample).shape}."
        )
    raw_velocity = float(values[axis_index])
    if not np.isfinite(raw_velocity):
        raise ValueError(f"DVL velocity is not finite: {raw_velocity}.")
    return raw_velocity, float(axis_sign * raw_velocity)


def build_velocity_tracking_metrics(samples: list[dict]) -> dict:
    forward_errors = [float(sample["forward_velocity_error"]) for sample in samples]
    lateral_errors = [float(sample["lateral_velocity_error"]) for sample in samples]
    command_efforts = [
        float(
            np.linalg.norm(
                np.asarray([sample[f"cmd_{index}"] for index in range(8)], dtype=float)
            )
        )
        for sample in samples
    ]

    return {
        "mean_forward_velocity_error": mean_value(forward_errors),
        "std_forward_velocity_error": std_value(forward_errors),
        "mean_lateral_velocity_error": mean_value(lateral_errors),
        "std_lateral_velocity_error": std_value(lateral_errors),
        "mean_abs_lateral_velocity_error": mean_value([abs(value) for value in lateral_errors]),
        "max_abs_lateral_velocity_error": max(
            [abs(value) for value in lateral_errors],
            default=0.0,
        ),
        "mean_command_effort": mean_value(command_efforts),
        "max_command_effort": max(command_efforts, default=0.0),
    }


def print_summary(summary: dict) -> None:
    print("\nStep 2B DVL velocity compensation summary")
    print(f"Target distance: {summary['target_distance']:.3f} m")
    print(
        "Desired body velocity: "
        f"forward={summary['desired_forward_velocity']:.3f} m/s, "
        f"lateral={summary['desired_lateral_velocity']:.3f} m/s"
    )
    print(
        "Current vector: "
        f"[{summary['current_x']:.3f}, {summary['current_y']:.3f}, "
        f"{summary['current_z']:.3f}] m/s"
    )
    print(f"DVL estimated distance: {summary['dvl_estimated_distance']:.3f} m")
    print(f"Pose forward displacement: {summary['pose_forward_displacement']:.3f} m")
    print(f"Pose Euclidean displacement: {summary['pose_euclidean_displacement']:.3f} m")
    print(f"Lateral drift: {summary['lateral_drift']:.3f} m")
    print(f"Final position error: {summary['final_position_error']:.3f} m")
    print(f"Absolute DVL/Pose error: {summary['absolute_error']:.3f} m")
    print(f"Duration: {summary['duration']:.2f} s")
    print(f"Mean lateral velocity error: {summary['mean_lateral_velocity_error']:.3f} m/s")
    print(f"Mean abs lateral velocity error: {summary['mean_abs_lateral_velocity_error']:.3f} m/s")
    print(f"Stop reason: {summary['stop_reason']}")
    print(f"Results directory: {summary['output_dir']}")
    print_warnings(summary)
