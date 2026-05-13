from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.forward_controller import ForwardThrusterController
from estimators.dvl_distance import DVLDistanceEstimator
from experiment_logging.experiment_logger import ExperimentLogger
from lib.rover import Rover
from lib.scenario_builder import ScenarioConfig
from lib.worlds import World
from metrics.distance import compute_distance_metrics
from visualization.distance_plots import (
    plot_distance_results,
    plot_lateral_drift,
    plot_speed_results,
    plot_trajectory,
)


DVL_SENSOR_KEY = "DVLSensor"
POSE_SENSOR_KEY = "PoseSensor"
VELOCITY_SENSOR_KEY = "VelocitySensor"
DEPTH_SENSOR_KEY = "DepthSensor"
DEFAULT_FORWARD_COMMAND = 2.0
OLD_FORWARD_COMMAND_DEFAULT = 12.0
DIAGNOSTIC_FORWARD_COMMAND = 0.1
DISTANCE_WARNING_RATIO = 0.10


def main() -> None:
    args = parse_args()
    apply_cli_defaults(args)
    run_forward_distance_experiment(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 1: move the BlueROV2 forward using DVL distance estimation."
    )
    parser.add_argument("--target-distance", type=float, default=5.0)
    parser.add_argument(
        "--forward-command",
        type=float,
        default=None,
        help=(
            "Forward command sent to horizontal BlueROV2 thrusters. "
            f"Default is {DEFAULT_FORWARD_COMMAND}, or {DIAGNOSTIC_FORWARD_COMMAND} "
            "when --diagnostic-distance-check is enabled."
        ),
    )
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--speed-warning-threshold", type=float, default=1.0)
    parser.add_argument("--max-dvl-speed-warning-threshold", type=float, default=1.5)
    parser.add_argument(
        "--diagnostic-distance-check",
        action="store_true",
        help="Print a detailed low-speed DVL/Pose distance comparison.",
    )
    parser.add_argument(
        "--headless",
        action="store_false",
        dest="show_viewport",
        help="Run without the HoloOcean viewport.",
    )
    parser.add_argument("--world", choices=World.list_worlds(), default=World.SimpleUnderwater)
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "step_01_forward_distance",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def apply_cli_defaults(args: argparse.Namespace) -> None:
    if args.forward_command is None:
        args.forward_command = (
            DIAGNOSTIC_FORWARD_COMMAND
            if args.diagnostic_distance_check
            else DEFAULT_FORWARD_COMMAND
        )


def run_forward_distance_experiment(
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
    forward_command = float(args.forward_command)
    current_vector = current_vector_from_args(args)
    current_enabled = current_vector is not None
    current_application_mode = "every_step" if current_enabled else "none"
    current_application_calls = 0
    experiment_label = getattr(args, "experiment_label", "Step 1")

    rov = Rover.BlueROV2Navigation(
        name="rov0",
        location=[0, 0, -4],
        rotation=[0, 0, 0],
        sensor_hz=args.ticks_per_sec,
        include_ground_truth=True,
    )
    scenario = (
        ScenarioConfig("Step01_Forward_Distance")
        .set_world(args.world)
        .set_main_agent("rov0")
        .add_agent(rov)
    )

    run_config = {
        "target_distance_m": args.target_distance,
        "forward_command": forward_command,
        "old_forward_command_default": OLD_FORWARD_COMMAND_DEFAULT,
        "new_forward_command_default": DEFAULT_FORWARD_COMMAND,
        "ticks_per_sec": args.ticks_per_sec,
        "max_duration_s": args.max_duration,
        "warmup_ticks": args.warmup_ticks,
        "world": args.world,
        "dvl_forward_index": args.dvl_forward_index,
        "dvl_forward_sign": args.dvl_forward_sign,
        "speed_warning_threshold_mps": args.speed_warning_threshold,
        "max_dvl_speed_warning_threshold_mps": args.max_dvl_speed_warning_threshold,
        "diagnostic_distance_check": bool(args.diagnostic_distance_check),
        "show_viewport": show_viewport,
        "current_x": float(current_vector[0]) if current_enabled else 0.0,
        "current_y": float(current_vector[1]) if current_enabled else 0.0,
        "current_z": float(current_vector[2]) if current_enabled else 0.0,
        "current_api_method": getattr(args, "current_api_method", ""),
        "current_application_mode": current_application_mode,
        "dt_policy": (
            "Use simulator state time if present; otherwise use measured wall-clock "
            "loop dt. Fall back to 1 / ticks_per_sec only if dt is invalid."
        ),
        "sensor_policy": {
            "DVL": "used for forward distance estimation",
            "IMU": "available for future estimation, not used in Step 1",
            "Depth": "available for future depth control, not used in Step 1",
            "Pose": "logged during the run and used after the run for evaluation only",
            "Velocity": "logged during the run and used after the run for evaluation only",
        },
    }
    logger.write_run_config(run_config)

    controller = ForwardThrusterController(thrust=forward_command)
    estimator = DVLDistanceEstimator(
        forward_axis_index=args.dvl_forward_index,
        forward_axis_sign=args.dvl_forward_sign,
    )
    samples: list[dict] = []
    stop_reason = "timeout"
    dt_fallback_count = 0
    dt_samples: list[float] = []

    print(f"Saving {experiment_label} outputs to: {logger.output_dir}")
    print(f"Forward command: {forward_command:.3f}")
    print(
        "Using DVL forward velocity "
        f"index {args.dvl_forward_index} with sign {args.dvl_forward_sign:+.0f}."
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
            current_vector=current_vector,
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
            build_sample(
                state,
                estimator,
                elapsed_s,
                stop_command,
                dvl_sample,
                current_vector,
            )
        )

        while elapsed_s < args.max_duration:
            command = controller.command()
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
                build_sample(state, estimator, elapsed_s, command, dvl_sample, current_vector)
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
    sanity_check = build_dvl_pose_sanity_check(metrics.to_dict())
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
            "forward_command": forward_command,
            "forward_command_vector": controller.command().tolist(),
            "old_forward_command_default": OLD_FORWARD_COMMAND_DEFAULT,
            "new_forward_command_default": DEFAULT_FORWARD_COMMAND,
            "dvl_forward_index": args.dvl_forward_index,
            "dvl_forward_sign": args.dvl_forward_sign,
            "warmup_ticks": args.warmup_ticks,
            "elapsed_wall_clock_time_s": wall_clock_duration_s,
            "time_source": time_source,
            "dt_source": time_source,
            "mean_dt": mean_value(dt_samples),
            "std_dt": std_value(dt_samples),
            "dt_fallback_count": dt_fallback_count,
            "speed_warning_threshold_mps": args.speed_warning_threshold,
            "max_dvl_speed_warning_threshold_mps": args.max_dvl_speed_warning_threshold,
            "mean_dvl_forward_velocity_mps": distance_validation[
                "mean_dvl_forward_velocity_mps"
            ],
            "max_dvl_forward_velocity_mps": distance_validation[
                "max_dvl_forward_velocity_mps"
            ],
            "average_dvl_speed_mps": distance_validation["average_dvl_speed_mps"],
            "average_pose_speed_mps": distance_validation["average_pose_speed_mps"],
            "mean_speed_mps": distance_validation["average_pose_speed_mps"],
            "initial_pose": distance_validation["initial_pose"],
            "final_pose": distance_validation["final_pose"],
            "distance_validation_warnings": distance_validation["warnings"],
            "diagnostic_distance_check": bool(args.diagnostic_distance_check),
            "experiment_label": experiment_label,
            "current_x": float(current_vector[0]) if current_enabled else 0.0,
            "current_y": float(current_vector[1]) if current_enabled else 0.0,
            "current_z": float(current_vector[2]) if current_enabled else 0.0,
            "current_magnitude": current_magnitude(current_vector),
            "current_api_method": getattr(args, "current_api_method", ""),
            "current_application_mode": current_application_mode,
            "current_application_calls": current_application_calls,
            "final_lateral_offset": metrics.lateral_drift_m,
            "summary_title": getattr(args, "summary_title", "Step 1 final summary"),
            "output_dir": str(logger.output_dir),
            "dvl_pose_sanity_check": sanity_check,
        }
    )

    logger.write_trajectory(samples)
    logger.write_summary(summary)
    plot_distance_results(samples, args.target_distance, logger.output_dir / "distance_plot.png")
    plot_trajectory(samples, logger.output_dir / "trajectory_plot.png")
    if bool(getattr(args, "make_current_plots", False)):
        plot_lateral_drift(samples, logger.output_dir / "lateral_drift_plot.png")
        plot_speed_results(samples, logger.output_dir / "speed_plot.png")

    if print_terminal_summary:
        print_summary(summary)

    return summary


def validate_args(args: argparse.Namespace) -> None:
    if args.target_distance <= 0:
        raise ValueError("target-distance must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.max_duration <= 0:
        raise ValueError("max-duration must be positive.")
    if args.forward_command < 0:
        raise ValueError("forward-command must be zero or positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.dvl_forward_index < 0:
        raise ValueError("dvl-forward-index must be zero or positive.")
    if args.dvl_forward_sign not in (-1.0, 1.0):
        raise ValueError("dvl-forward-sign must be +1 or -1.")
    if args.speed_warning_threshold <= 0:
        raise ValueError("speed-warning-threshold must be positive.")
    if args.max_dvl_speed_warning_threshold <= 0:
        raise ValueError("max-dvl-speed-warning-threshold must be positive.")


def run_warmup(
    env,
    command: np.ndarray,
    warmup_ticks: int,
    agent_name: str = "rov0",
    current_vector: list[float] | None = None,
) -> tuple[dict, int]:
    state = None
    current_application_calls = 0
    for _ in range(warmup_ticks):
        if current_vector is not None:
            apply_ocean_current(env, agent_name, current_vector)
            current_application_calls += 1
        state = env.step(command)

    if state is None:
        if current_vector is not None:
            apply_ocean_current(env, agent_name, current_vector)
            current_application_calls += 1
        state = env.step(command)

    return state, current_application_calls


def apply_ocean_current(env, agent_name: str, current_vector: list[float]) -> None:
    env.set_ocean_currents(agent_name, current_vector)


def build_sample(
    state: dict,
    estimator: DVLDistanceEstimator,
    time_s: float,
    command: np.ndarray,
    dvl_sample,
    current_vector: list[float] | None = None,
) -> dict:
    pose = pose_components(require_sensor(state, POSE_SENSOR_KEY))
    velocity = optional_vector(state.get(VELOCITY_SENSOR_KEY), 3)
    depth = optional_scalar(state.get(DEPTH_SENSOR_KEY))
    raw_dvl_velocity, used_dvl_velocity = estimator.velocity_components(dvl_sample)
    current = current_vector or [0.0, 0.0, 0.0]

    sample = {
        "time": float(time_s),
        "current_x": float(current[0]),
        "current_y": float(current[1]),
        "current_z": float(current[2]),
        "dvl_forward_velocity_raw": raw_dvl_velocity,
        "dvl_forward_velocity_used": used_dvl_velocity,
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
        sample[f"cmd_{index}"] = float(command_values[index]) if index < command_values.size else 0.0

    return sample


def add_pose_evaluation_fields(samples: list[dict]) -> None:
    if not samples:
        return

    start_position = position_from_sample(samples[0])
    start_forward = unit_vector_from_sample(samples[0], "pose_forward")
    start_right = unit_vector_from_sample(samples[0], "pose_right")

    for sample in samples:
        displacement = position_from_sample(sample) - start_position
        sample["pose_forward_displacement"] = float(np.dot(displacement, start_forward))
        sample["pose_lateral_drift"] = float(np.dot(displacement, start_right))
        sample["pose_ground_truth_displacement"] = float(np.linalg.norm(displacement))
        sample["pose_euclidean_displacement"] = sample["pose_ground_truth_displacement"]


def build_dvl_pose_sanity_check(metrics: dict) -> dict:
    dvl_distance = float(metrics["dvl_estimated_distance_m"])
    pose_forward = float(metrics["pose_forward_displacement_m"])
    lateral_drift = float(metrics["lateral_drift_m"])

    min_check_distance = 0.05
    warning = ""
    if abs(dvl_distance) < min_check_distance or abs(pose_forward) < min_check_distance:
        warning = (
            "DVL/Pose sign check is inconclusive because the traveled distance is too small."
        )
    elif dvl_distance * pose_forward < 0:
        warning = (
            "DVL integrated distance and Pose forward displacement have opposite signs. "
            "The DVL forward axis/sign may need to be inverted; try --dvl-forward-sign -1 "
            "or a different --dvl-forward-index."
        )

    lateral_ratio = abs(lateral_drift) / max(abs(pose_forward), min_check_distance)
    return {
        "dvl_estimated_distance_m": dvl_distance,
        "pose_forward_displacement_m": pose_forward,
        "lateral_drift_m": lateral_drift,
        "lateral_drift_ratio": float(lateral_ratio),
        "opposite_signs": bool(dvl_distance * pose_forward < 0),
        "warning": warning,
    }


def build_distance_validation(
    samples: list[dict],
    metrics: dict,
    wall_clock_duration_s: float,
    speed_warning_threshold: float,
    max_dvl_speed_warning_threshold: float,
) -> dict:
    first = samples[0]
    last = samples[-1]
    duration = max(float(metrics["duration_s"]), 1e-9)
    dvl_distance = float(metrics["dvl_estimated_distance_m"])
    pose_forward = float(metrics["pose_forward_displacement_m"])
    pose_euclidean = float(metrics["pose_ground_truth_displacement_m"])
    dvl_velocities = [float(sample["dvl_forward_velocity_used"]) for sample in samples]
    max_dvl_velocity = max(abs(value) for value in dvl_velocities) if dvl_velocities else 0.0
    mean_dvl_velocity = float(np.mean(dvl_velocities)) if dvl_velocities else 0.0
    average_dvl_speed = dvl_distance / duration
    average_pose_speed = pose_euclidean / duration

    warnings = []
    if average_pose_speed > speed_warning_threshold:
        warnings.append(
            "Average Pose speed exceeds the configured threshold. "
            "The forward command or environmental disturbance may be too high "
            "for slow validation."
        )
    if max_dvl_velocity > max_dvl_speed_warning_threshold:
        warnings.append(
            "Maximum DVL forward velocity exceeds the configured threshold. "
            "The forward command may be too high for slow validation."
        )

    reference_distance = max(abs(pose_forward), 1e-9)
    dvl_pose_difference_ratio = abs(dvl_distance - pose_forward) / reference_distance
    if dvl_pose_difference_ratio > DISTANCE_WARNING_RATIO:
        warnings.append(
            "DVL estimated distance and Pose forward displacement differ by more "
            "than 10 percent."
        )

    pose_direction_difference_ratio = abs(pose_euclidean - abs(pose_forward)) / max(
        pose_euclidean,
        1e-9,
    )
    if pose_direction_difference_ratio > DISTANCE_WARNING_RATIO:
        warnings.append(
            "Pose Euclidean displacement differs from Pose forward displacement "
            "by more than 10 percent, indicating lateral or vertical motion."
        )

    return {
        "initial_pose": pose_dict_from_sample(first),
        "final_pose": pose_dict_from_sample(last),
        "elapsed_wall_clock_time_s": float(wall_clock_duration_s),
        "average_dvl_speed_mps": float(average_dvl_speed),
        "average_pose_speed_mps": float(average_pose_speed),
        "max_dvl_forward_velocity_mps": float(max_dvl_velocity),
        "mean_dvl_forward_velocity_mps": float(mean_dvl_velocity),
        "dvl_pose_difference_ratio": float(dvl_pose_difference_ratio),
        "pose_direction_difference_ratio": float(pose_direction_difference_ratio),
        "warnings": warnings,
    }


def pose_dict_from_sample(sample: dict) -> dict:
    return {
        "x": float(sample["pose_x"]),
        "y": float(sample["pose_y"]),
        "z": float(sample["pose_z"]),
    }


def current_vector_from_args(args: argparse.Namespace) -> list[float] | None:
    if not hasattr(args, "current_x"):
        return None
    return [
        float(args.current_x),
        float(args.current_y),
        float(args.current_z),
    ]


def current_magnitude(current_vector: list[float] | None) -> float:
    if current_vector is None:
        return 0.0
    return float(np.linalg.norm(np.asarray(current_vector, dtype=float)))


def require_sensor(state: dict, sensor_key: str):
    if sensor_key not in state:
        available = ", ".join(sorted(str(key) for key in state.keys()))
        raise RuntimeError(
            f"Required sensor '{sensor_key}' was not found in HoloOcean state. "
            f"Available keys: {available}"
        )
    return state[sensor_key]


def extract_state_time_s(state: dict) -> float | None:
    for key in ("time", "Time", "timestamp", "Timestamp", "sim_time", "SimTime"):
        if key in state:
            value = optional_scalar(state[key])
            if value is not None and np.isfinite(value):
                return value
    return None


def pose_components(pose_matrix) -> dict:
    matrix = np.asarray(pose_matrix, dtype=float)
    if matrix.shape[0] < 3 or matrix.shape[1] < 4:
        raise ValueError(f"Pose sensor returned an unexpected shape: {matrix.shape}.")

    rotation = matrix[:3, :3]
    return {
        "position": matrix[:3, 3].astype(float),
        "forward": unit_vector(rotation[:, 0], "pose forward axis"),
        "right": unit_vector(rotation[:, 1], "pose right axis"),
    }


def position_from_sample(sample: dict) -> np.ndarray:
    return np.array([sample["pose_x"], sample["pose_y"], sample["pose_z"]], dtype=float)


def unit_vector_from_sample(sample: dict, prefix: str) -> np.ndarray:
    return unit_vector(
        np.array(
            [
                sample[f"{prefix}_x"],
                sample[f"{prefix}_y"],
                sample[f"{prefix}_z"],
            ],
            dtype=float,
        ),
        prefix,
    )


def unit_vector(vector, name: str) -> np.ndarray:
    values = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(values))
    if norm < 1e-9:
        raise ValueError(f"Cannot normalize {name}; norm is too small.")
    return values / norm


def optional_vector(value, expected_size: int):
    if value is None:
        return None
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size < expected_size:
        return None
    return [float(values[i]) for i in range(expected_size)]


def optional_scalar(value):
    if value is None:
        return None
    values = np.asarray(value, dtype=float).reshape(-1)
    if values.size == 0:
        return None
    return float(values[0])


def mean_value(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(np.mean(values))


def std_value(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    return float(np.std(values, ddof=0))


def print_summary(summary: dict) -> None:
    print(f"\n{summary.get('summary_title', 'Step 1 final summary')}")
    print(f"Target distance: {summary['target_distance_m']:.3f} m")
    print(f"Forward command: {summary['forward_command']:.3f}")
    print(f"Command vector: {summary['forward_command_vector']}")
    if summary.get("current_magnitude", 0.0) > 0.0:
        print(
            "Current vector: "
            f"[{summary['current_x']:.3f}, {summary['current_y']:.3f}, {summary['current_z']:.3f}] m/s"
        )
        print(f"Current magnitude: {summary['current_magnitude']:.3f} m/s")
        print(f"Current application mode: {summary['current_application_mode']}")
    print(f"DVL estimated distance: {summary['dvl_estimated_distance_m']:.3f} m")
    print(
        "Pose Euclidean displacement: "
        f"{summary['pose_ground_truth_displacement_m']:.3f} m"
    )
    print(f"Pose forward displacement: {summary['pose_forward_displacement_m']:.3f} m")
    print(f"Initial Pose: {summary['initial_pose']}")
    print(f"Final Pose: {summary['final_pose']}")
    print(f"Absolute distance error: {summary['absolute_distance_error_m']:.3f} m")
    print(f"Percentage error: {summary['percentage_error']:.2f}%")
    print(f"Lateral drift: {summary['lateral_drift_m']:.3f} m")
    print(f"Duration: {summary['duration_s']:.2f} s")
    print(f"Elapsed wall-clock time: {summary['elapsed_wall_clock_time_s']:.2f} s")
    print(f"Average DVL speed: {summary['average_dvl_speed_mps']:.3f} m/s")
    print(f"Average Pose speed: {summary['average_pose_speed_mps']:.3f} m/s")
    print(f"Average speed: {summary['mean_speed_mps']:.3f} m/s")
    print(f"Mean DVL forward velocity: {summary['mean_dvl_forward_velocity_mps']:.3f} m/s")
    print(f"Max DVL forward velocity: {summary['max_dvl_forward_velocity_mps']:.3f} m/s")
    print(f"Number of samples: {summary['num_samples']}")
    print(f"Stop reason: {summary['stop_reason']}")
    print(f"Time source: {summary['time_source']}")
    print(f"Results directory: {summary['output_dir']}")

    warning = summary["dvl_pose_sanity_check"].get("warning")
    if warning:
        print(f"\nWARNING: {warning}")
    else:
        print("\nDVL/Pose sanity check: DVL and Pose forward displacement signs agree.")

    for validation_warning in summary["distance_validation_warnings"]:
        print(f"WARNING: {validation_warning}")

    if summary.get("diagnostic_distance_check", False):
        print("\nDiagnostic distance check")
        print(f"DVL distance: {summary['dvl_estimated_distance_m']:.3f} m")
        print(f"Pose forward displacement: {summary['pose_forward_displacement_m']:.3f} m")
        print(f"Pose Euclidean displacement: {summary['pose_ground_truth_displacement_m']:.3f} m")
        print(f"Duration: {summary['duration_s']:.2f} s")
        print(f"Average Pose speed: {summary['average_pose_speed_mps']:.3f} m/s")
        print(f"Final coordinates: {summary['final_pose']}")


if __name__ == "__main__":
    main()
