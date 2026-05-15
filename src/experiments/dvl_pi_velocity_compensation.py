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

from actuation.holoocean_bluerov2_mixer import HoloOceanBlueROV2Mixer
from control.body_commands import (
    BodyCommand,
    BodyVelocityMeasurement,
    BodyVelocitySetpoint,
)
from controllers.dvl_velocity_pi_controller import DVLVelocityPIController
from estimators.dvl_distance import DVLDistanceEstimator
from experiment_logging.experiment_logger import ExperimentLogger
from experiments.dvl_velocity_compensation import dvl_velocity_components
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


DEFAULT_DESIRED_FORWARD_VELOCITY = 0.5
DEFAULT_DESIRED_LATERAL_VELOCITY = 0.0
DEFAULT_KP_SURGE = 2.0
DEFAULT_KI_SURGE = 0.2
DEFAULT_KP_SWAY = 3.0
DEFAULT_KI_SWAY = 0.5
DEFAULT_MAX_SURGE = 1.0
DEFAULT_MAX_SWAY = 1.0
DEFAULT_INTEGRAL_LIMIT_SURGE = 1.0
DEFAULT_INTEGRAL_LIMIT_SWAY = 1.0
DEFAULT_MAX_THRUSTER_COMMAND = 2.0
DEFAULT_MAX_DURATION = 120.0


def run_dvl_pi_velocity_compensation_experiment(
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

    controller = DVLVelocityPIController(
        kp_surge=args.kp_surge,
        ki_surge=args.ki_surge,
        kp_sway=args.kp_sway,
        ki_sway=args.ki_sway,
        max_surge=args.max_surge,
        max_sway=args.max_sway,
        integral_limit_surge=args.integral_limit_surge,
        integral_limit_sway=args.integral_limit_sway,
    )
    mixer = HoloOceanBlueROV2Mixer(
        max_thruster_command=args.max_thruster_command,
        base_vertical_command=0.0,
    )
    estimator = DVLDistanceEstimator(
        forward_axis_index=args.dvl_forward_index,
        forward_axis_sign=args.dvl_forward_sign,
    )
    setpoint = BodyVelocitySetpoint(
        surge_mps=args.desired_forward_velocity,
        sway_mps=args.desired_lateral_velocity,
    )

    rov = Rover.BlueROV2Navigation(
        name="rov0",
        location=[0, 0, -4],
        rotation=[0, 0, 0],
        sensor_hz=args.ticks_per_sec,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step02C_DVL_PI_Velocity_Compensation")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(rov)
    )

    run_config = {
        "target_distance_m": float(args.target_distance),
        "desired_forward_velocity_mps": float(args.desired_forward_velocity),
        "desired_lateral_velocity_mps": float(args.desired_lateral_velocity),
        "kp_surge": float(args.kp_surge),
        "ki_surge": float(args.ki_surge),
        "kp_sway": float(args.kp_sway),
        "ki_sway": float(args.ki_sway),
        "max_surge": float(args.max_surge),
        "max_sway": float(args.max_sway),
        "integral_limit_surge": float(args.integral_limit_surge),
        "integral_limit_sway": float(args.integral_limit_sway),
        "max_thruster_command": float(args.max_thruster_command),
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
        "controller_type": "DVLVelocityPIController",
        "command_abstraction": "body_frame_normalized_command",
        "actuation_backend": "HoloOceanBlueROV2Mixer",
        "real_hardware_policy": (
            "controller outputs body-frame commands; real BlueROV2 should use "
            "an ArduSub/MAVLink backend rather than hard-coded motor order."
        ),
        "sensor_policy": {
            "DVL": "used for forward distance estimation, stopping, and PI velocity tracking",
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

    print(f"Saving Step 2C DVL PI velocity compensation outputs to: {logger.output_dir}")
    print(
        "Desired body velocity: "
        f"surge={args.desired_forward_velocity:.3f} m/s, "
        f"sway={args.desired_lateral_velocity:.3f} m/s"
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
        zero_body_command = BodyCommand(surge=0.0, sway=0.0)
        stop_command = mixer.mix(zero_body_command)
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
        control_dt_s = fallback_dt_s
        wall_clock_start = time.perf_counter()
        dvl_sample = require_sensor(state, DVL_SENSOR_KEY)

        samples.append(
            build_pi_velocity_tracking_sample(
                state=state,
                estimator=estimator,
                time_s=elapsed_s,
                body_command=zero_body_command,
                thruster_command=stop_command,
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
            measurement = BodyVelocityMeasurement(
                surge_mps=measured_forward,
                sway_mps=measured_lateral,
            )
            body_command = controller.command(
                setpoint=setpoint,
                measurement=measurement,
                dt_s=control_dt_s,
            )
            thruster_command = mixer.mix(body_command)

            if current_enabled:
                apply_ocean_current(env, "rov0", current_vector)
                current_application_calls += 1
            state = env.step(thruster_command)

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
            control_dt_s = dt_s
            dt_samples.append(dt_s)
            dvl_sample = require_sensor(state, DVL_SENSOR_KEY)
            estimator.update(dvl_sample, dt_s)
            samples.append(
                build_pi_velocity_tracking_sample(
                    state=state,
                    estimator=estimator,
                    time_s=elapsed_s,
                    body_command=body_command,
                    thruster_command=thruster_command,
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
            "target_reached": bool(stop_reason == "target_reached"),
            "desired_forward_velocity": float(args.desired_forward_velocity),
            "desired_lateral_velocity": float(args.desired_lateral_velocity),
            "kp_surge": float(args.kp_surge),
            "ki_surge": float(args.ki_surge),
            "kp_sway": float(args.kp_sway),
            "ki_sway": float(args.ki_sway),
            "max_surge": float(args.max_surge),
            "max_sway": float(args.max_sway),
            "integral_limit_surge": float(args.integral_limit_surge),
            "integral_limit_sway": float(args.integral_limit_sway),
            "max_thruster_command": float(args.max_thruster_command),
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
            "controller_type": "DVLVelocityPIController",
            "command_abstraction": "body_frame_normalized_command",
            "actuation_backend": "HoloOceanBlueROV2Mixer",
            "output_dir": str(logger.output_dir),
            "distance_validation_warnings": distance_validation["warnings"],
            "dvl_pose_sanity_check": build_dvl_pose_sanity_check(metrics.to_dict()),
        }
    )
    summary.update(build_pi_velocity_tracking_metrics(samples))
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
    if args.kp_surge < 0 or args.kp_sway < 0:
        raise ValueError("proportional gains must be non-negative.")
    if args.ki_surge < 0 or args.ki_sway < 0:
        raise ValueError("integral gains must be non-negative.")
    if not 0.0 <= args.max_surge <= 1.0:
        raise ValueError("max-surge must be in [0, 1].")
    if not 0.0 <= args.max_sway <= 1.0:
        raise ValueError("max-sway must be in [0, 1].")
    if args.integral_limit_surge < 0 or args.integral_limit_sway < 0:
        raise ValueError("integral limits must be non-negative.")
    if args.max_thruster_command < 0:
        raise ValueError("max-thruster-command must be non-negative.")
    if args.speed_warning_threshold <= 0:
        raise ValueError("speed-warning-threshold must be positive.")
    if args.max_dvl_speed_warning_threshold <= 0:
        raise ValueError("max-dvl-speed-warning-threshold must be positive.")


def build_pi_velocity_tracking_sample(
    state: dict,
    estimator: DVLDistanceEstimator,
    time_s: float,
    body_command: BodyCommand,
    thruster_command: np.ndarray,
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
    surge_error = float(args.desired_forward_velocity - used_forward)
    sway_error = float(args.desired_lateral_velocity - used_lateral)
    current = current_vector or [0.0, 0.0, 0.0]
    current_norm = float(np.linalg.norm(np.asarray(current, dtype=float)))
    metadata = body_command.metadata

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
        "forward_velocity_error": surge_error,
        "lateral_velocity_error": sway_error,
        "surge_velocity_error": surge_error,
        "sway_velocity_error": sway_error,
        "body_command_surge": float(body_command.surge),
        "body_command_sway": float(body_command.sway),
        "body_command_heave": float(body_command.heave),
        "body_command_yaw": float(body_command.yaw),
        "body_command_saturated": bool(body_command.saturated),
        "surge_saturated": bool(metadata.get("surge_saturated", False)),
        "sway_saturated": bool(metadata.get("sway_saturated", False)),
        "surge_integral": float(metadata.get("surge_integral", 0.0)),
        "sway_integral": float(metadata.get("sway_integral", 0.0)),
        "raw_surge_command": float(metadata.get("raw_surge_command", body_command.surge)),
        "raw_sway_command": float(metadata.get("raw_sway_command", body_command.sway)),
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

    command_values = np.asarray(thruster_command, dtype=float).reshape(-1)
    for index in range(8):
        sample[f"cmd_{index}"] = (
            float(command_values[index]) if index < command_values.size else 0.0
        )
    return sample


def build_pi_velocity_tracking_metrics(samples: list[dict]) -> dict:
    active_samples = samples[1:] if len(samples) > 1 else samples
    surge_errors = [
        float(sample["surge_velocity_error"]) for sample in active_samples
    ]
    sway_errors = [
        float(sample["sway_velocity_error"]) for sample in active_samples
    ]
    command_efforts = [
        float(
            np.linalg.norm(
                np.asarray(
                    [
                        sample["body_command_surge"],
                        sample["body_command_sway"],
                        sample["body_command_heave"],
                        sample["body_command_yaw"],
                    ],
                    dtype=float,
                )
            )
        )
        for sample in active_samples
    ]
    saturated = [bool(sample["body_command_saturated"]) for sample in active_samples]
    surge_saturated = [bool(sample["surge_saturated"]) for sample in active_samples]
    sway_saturated = [bool(sample["sway_saturated"]) for sample in active_samples]

    return {
        "velocity_tracking_metric_sample_count": len(active_samples),
        "mean_surge_velocity_error": mean_value(surge_errors),
        "mean_abs_surge_velocity_error": mean_value([abs(value) for value in surge_errors]),
        "mean_sway_velocity_error": mean_value(sway_errors),
        "mean_abs_sway_velocity_error": mean_value([abs(value) for value in sway_errors]),
        "mean_forward_velocity_error": mean_value(surge_errors),
        "std_forward_velocity_error": std_value(surge_errors),
        "mean_lateral_velocity_error": mean_value(sway_errors),
        "std_lateral_velocity_error": std_value(sway_errors),
        "mean_abs_lateral_velocity_error": mean_value([abs(value) for value in sway_errors]),
        "max_abs_lateral_velocity_error": max(
            [abs(value) for value in sway_errors],
            default=0.0,
        ),
        "mean_command_effort": mean_value(command_efforts),
        "max_command_effort": max(command_efforts, default=0.0),
        "saturation_fraction": mean_value([float(value) for value in saturated]),
        "surge_saturation_fraction": mean_value(
            [float(value) for value in surge_saturated]
        ),
        "sway_saturation_fraction": mean_value(
            [float(value) for value in sway_saturated]
        ),
    }


def print_summary(summary: dict) -> None:
    print("\nStep 2C DVL PI velocity compensation summary")
    print(f"Target distance: {summary['target_distance']:.3f} m")
    print(
        "Desired body velocity: "
        f"surge={summary['desired_forward_velocity']:.3f} m/s, "
        f"sway={summary['desired_lateral_velocity']:.3f} m/s"
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
    print(f"Duration: {summary['duration']:.2f} s")
    print(f"Mean sway velocity error: {summary['mean_sway_velocity_error']:.3f} m/s")
    print(f"Mean abs sway velocity error: {summary['mean_abs_sway_velocity_error']:.3f} m/s")
    print(f"Saturation fraction: {summary['saturation_fraction']:.3f}")
    print(f"Stop reason: {summary['stop_reason']}")
    print(f"Results directory: {summary['output_dir']}")
    print_warnings(summary)
