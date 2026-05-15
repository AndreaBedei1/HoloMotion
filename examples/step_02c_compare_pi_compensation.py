from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.dvl_pi_velocity_compensation import (
    DEFAULT_DESIRED_FORWARD_VELOCITY,
    DEFAULT_DESIRED_LATERAL_VELOCITY,
    DEFAULT_INTEGRAL_LIMIT_SURGE,
    DEFAULT_INTEGRAL_LIMIT_SWAY,
    DEFAULT_KI_SURGE,
    DEFAULT_KI_SWAY,
    DEFAULT_KP_SURGE,
    DEFAULT_KP_SWAY,
    DEFAULT_MAX_DURATION,
    DEFAULT_MAX_SURGE,
    DEFAULT_MAX_SWAY,
    DEFAULT_MAX_THRUSTER_COMMAND,
    run_dvl_pi_velocity_compensation_experiment,
)
from experiments.dvl_velocity_compensation import (
    DEFAULT_FORWARD_KP,
    DEFAULT_LATERAL_KP,
    DEFAULT_MAX_COMMAND,
    run_dvl_velocity_compensation_experiment,
)
from experiments.forward_distance import DEFAULT_FORWARD_COMMAND, run_forward_distance_experiment
from lib.worlds import World
from visualization.velocity_tracking_plots import plot_mode_metric_comparison


MODES = [
    "no_compensation",
    "dvl_velocity_compensation",
    "dvl_pi_velocity_compensation",
]

CSV_FIELDNAMES = [
    "run_id",
    "mode",
    "target_distance",
    "repetition_index",
    "current_x",
    "current_y",
    "current_z",
    "current_magnitude",
    "max_duration_s",
    "forward_command",
    "desired_forward_velocity",
    "desired_lateral_velocity",
    "dvl_estimated_distance",
    "pose_forward_displacement",
    "pose_euclidean_displacement",
    "absolute_error",
    "percentage_error",
    "lateral_drift",
    "final_position_error",
    "duration",
    "sample_count",
    "stop_reason",
    "target_reached",
    "mean_forward_velocity_error",
    "std_forward_velocity_error",
    "mean_lateral_velocity_error",
    "std_lateral_velocity_error",
    "mean_abs_lateral_velocity_error",
    "max_abs_lateral_velocity_error",
    "mean_surge_velocity_error",
    "mean_abs_surge_velocity_error",
    "mean_sway_velocity_error",
    "mean_abs_sway_velocity_error",
    "mean_command_effort",
    "max_command_effort",
    "saturation_fraction",
    "surge_saturation_fraction",
    "sway_saturation_fraction",
    "mean_dt",
    "std_dt",
    "dt_source",
]


def main() -> None:
    args = parse_args()
    run_comparison(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Step 2C comparison between no compensation, Step 2B P-only DVL "
            "velocity compensation, and Step 2C PI DVL velocity compensation."
        )
    )
    parser.add_argument("--target-distances", type=float, nargs="+", default=[5.0, 10.0, 20.0])
    parser.add_argument("--current-y-values", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    parser.add_argument("--current-x", type=float, default=0.0)
    parser.add_argument("--current-z", type=float, default=0.0)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--forward-command", type=float, default=DEFAULT_FORWARD_COMMAND)
    parser.add_argument(
        "--desired-forward-velocity",
        type=float,
        default=DEFAULT_DESIRED_FORWARD_VELOCITY,
    )
    parser.add_argument(
        "--desired-lateral-velocity",
        type=float,
        default=DEFAULT_DESIRED_LATERAL_VELOCITY,
    )
    parser.add_argument("--kp-forward", type=float, default=DEFAULT_FORWARD_KP)
    parser.add_argument("--kp-lateral", type=float, default=DEFAULT_LATERAL_KP)
    parser.add_argument("--max-forward-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--max-lateral-command", type=float, default=DEFAULT_MAX_COMMAND)
    parser.add_argument("--kp-surge", type=float, default=DEFAULT_KP_SURGE)
    parser.add_argument("--ki-surge", type=float, default=DEFAULT_KI_SURGE)
    parser.add_argument("--kp-sway", type=float, default=DEFAULT_KP_SWAY)
    parser.add_argument("--ki-sway", type=float, default=DEFAULT_KI_SWAY)
    parser.add_argument("--max-surge", type=float, default=DEFAULT_MAX_SURGE)
    parser.add_argument("--max-sway", type=float, default=DEFAULT_MAX_SWAY)
    parser.add_argument(
        "--integral-limit-surge",
        type=float,
        default=DEFAULT_INTEGRAL_LIMIT_SURGE,
    )
    parser.add_argument(
        "--integral-limit-sway",
        type=float,
        default=DEFAULT_INTEGRAL_LIMIT_SWAY,
    )
    parser.add_argument(
        "--max-thruster-command",
        type=float,
        default=DEFAULT_MAX_THRUSTER_COMMAND,
        help="Simulation mixer limit applied to each HoloOcean thruster command.",
    )
    parser.add_argument("--max-duration", type=float, default=DEFAULT_MAX_DURATION)
    parser.add_argument("--dvl-forward-index", type=int, default=0)
    parser.add_argument("--dvl-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--dvl-lateral-index", type=int, default=1)
    parser.add_argument("--dvl-lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--warmup-ticks", type=int, default=10)
    parser.add_argument("--speed-warning-threshold", type=float, default=1.0)
    parser.add_argument("--max-dvl-speed-warning-threshold", type=float, default=1.5)
    parser.add_argument("--world", choices=World.list_worlds(), default=World.SimpleUnderwater)
    parser.add_argument(
        "--headless",
        action="store_false",
        dest="show_viewport",
        help="Run without the HoloOcean viewport.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=PROJECT_ROOT / "results" / "step_02c_pi_compensation_comparison",
    )
    parser.add_argument(
        "--resume-dir",
        type=Path,
        default=None,
        help="Resume an existing timestamped comparison directory and skip completed mode runs.",
    )
    parser.add_argument(
        "--retry-count",
        type=int,
        default=2,
        help="Number of retries for a failed HoloOcean run before aborting.",
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=5.0,
        help="Seconds to wait before retrying a failed HoloOcean run.",
    )
    parser.add_argument(
        "--run-settle-delay",
        type=float,
        default=1.0,
        help="Seconds to wait between mode runs so the simulator can release resources.",
    )
    parser.set_defaults(show_viewport=True)
    return parser.parse_args()


def run_comparison(args: argparse.Namespace) -> None:
    validate_args(args)

    batch_dir = args.resume_dir if args.resume_dir is not None else (
        args.results_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    batch_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving Step 2C PI compensation comparison outputs to: {batch_dir}")
    print(f"Viewport enabled: {args.show_viewport}")
    print(f"Shared max duration for all modes: {args.max_duration:g} s")
    if args.resume_dir is not None:
        print("Resume mode: completed mode runs with summary.json will be skipped.")

    expected_pairs = build_expected_pairs(args)
    rows = load_completed_rows(batch_dir, expected_pairs, args)
    completed_mode_runs = {(row["run_id"], row["mode"]) for row in rows}
    if rows:
        print(f"Loaded completed mode runs: {len(rows)}")
        write_comparison_outputs(
            args,
            batch_dir,
            rows,
            expected_mode_run_count=len(expected_pairs) * len(MODES),
            write_plots=False,
        )

    total_pairs = len(expected_pairs)
    for pair_number, expected_pair in enumerate(expected_pairs, start=1):
        target_distance = expected_pair["target_distance"]
        current_y = expected_pair["current_y"]
        repetition_index = expected_pair["repetition_index"]
        base_run_id = expected_pair["run_id"]
        print(
            f"[{pair_number}/{total_pairs}] target={target_distance:g} m, "
            f"current_y={current_y:g} m/s, repetition={repetition_index}"
        )

        for mode in MODES:
            key = (base_run_id, mode)
            if key in completed_mode_runs:
                print(f"  skipping completed mode={mode}, run_id={base_run_id}")
                continue

            runner, run_args = build_mode_runner_and_args(
                args,
                mode,
                target_distance,
                current_y,
            )
            summary = run_experiment_with_retries(
                args=args,
                run_label=f"{base_run_id}_{mode}",
                runner=runner,
                run_args=run_args,
                output_dir=batch_dir / f"{base_run_id}_{mode}",
            )
            rows.append(
                summary_to_row(
                    summary,
                    run_id=base_run_id,
                    mode=mode,
                    target_distance=target_distance,
                    repetition_index=repetition_index,
                    args=args,
                )
            )
            completed_mode_runs.add(key)
            write_comparison_outputs(
                args,
                batch_dir,
                rows,
                expected_mode_run_count=len(expected_pairs) * len(MODES),
                write_plots=False,
            )
            sleep_between_runs(args)

    aggregate_summary = write_comparison_outputs(
        args,
        batch_dir,
        rows,
        expected_mode_run_count=len(expected_pairs) * len(MODES),
        write_plots=True,
    )

    print("\nStep 2C PI compensation comparison summary")
    print(f"Total runs: {len(rows)}")
    print(f"Results directory: {batch_dir}")
    for group in aggregate_summary["groups"]:
        print(
            f"target={group['target_distance']:g} m, current_y={group['current_y']:g} m/s: "
            "Step 2C lateral drift reduction="
            f"{group['dvl_pi_velocity_compensation_lateral_drift_reduction_percentage']:.1f}% "
            f"(valid={group['dvl_pi_velocity_compensation_lateral_drift_reduction_valid']})"
        )


def validate_args(args: argparse.Namespace) -> None:
    if not args.target_distances:
        raise ValueError("At least one target distance is required.")
    if any(distance <= 0 for distance in args.target_distances):
        raise ValueError("All target distances must be positive.")
    if not args.current_y_values:
        raise ValueError("At least one current-y value is required.")
    if args.repetitions <= 0:
        raise ValueError("repetitions must be positive.")
    if args.forward_command < 0:
        raise ValueError("forward-command must be zero or positive.")
    if args.max_duration <= 0:
        raise ValueError("max-duration must be positive.")
    if args.ticks_per_sec <= 0:
        raise ValueError("ticks-per-sec must be positive.")
    if args.warmup_ticks < 0:
        raise ValueError("warmup-ticks must be zero or positive.")
    if args.max_forward_command < 0 or args.max_lateral_command < 0:
        raise ValueError("Step 2B max command values must be non-negative.")
    if args.max_thruster_command < 0:
        raise ValueError("max-thruster-command must be non-negative.")
    if args.kp_surge < 0 or args.kp_sway < 0:
        raise ValueError("PI proportional gains must be non-negative.")
    if args.ki_surge < 0 or args.ki_sway < 0:
        raise ValueError("PI integral gains must be non-negative.")
    if not 0.0 <= args.max_surge <= 1.0:
        raise ValueError("max-surge must be in [0, 1].")
    if not 0.0 <= args.max_sway <= 1.0:
        raise ValueError("max-sway must be in [0, 1].")
    if args.integral_limit_surge < 0 or args.integral_limit_sway < 0:
        raise ValueError("integral limits must be non-negative.")
    if args.retry_count < 0:
        raise ValueError("retry-count must be zero or positive.")
    if args.retry_delay < 0:
        raise ValueError("retry-delay must be zero or positive.")
    if args.run_settle_delay < 0:
        raise ValueError("run-settle-delay must be zero or positive.")


def build_expected_pairs(args: argparse.Namespace) -> list[dict]:
    expected_pairs = []
    for target_distance in args.target_distances:
        for current_y in args.current_y_values:
            for repetition_index in range(1, args.repetitions + 1):
                expected_pairs.append(
                    {
                        "run_id": make_run_id(
                            target_distance,
                            current_y,
                            repetition_index,
                        ),
                        "target_distance": float(target_distance),
                        "current_y": float(current_y),
                        "repetition_index": int(repetition_index),
                    }
                )
    return expected_pairs


def load_completed_rows(
    batch_dir: Path,
    expected_pairs: list[dict],
    args: argparse.Namespace,
) -> list[dict]:
    rows = []
    for expected_pair in expected_pairs:
        for mode in MODES:
            run_id = expected_pair["run_id"]
            summary_path = batch_dir / f"{run_id}_{mode}" / "summary.json"
            if not summary_path.exists():
                continue
            with summary_path.open("r", encoding="utf-8") as file:
                summary = json.load(file)
            rows.append(
                summary_to_row(
                    summary,
                    run_id=run_id,
                    mode=mode,
                    target_distance=expected_pair["target_distance"],
                    repetition_index=expected_pair["repetition_index"],
                    args=args,
                )
            )
    return rows


def build_mode_runner_and_args(
    args: argparse.Namespace,
    mode: str,
    target_distance: float,
    current_y: float,
):
    if mode == "no_compensation":
        return run_forward_distance_experiment, build_no_compensation_args(
            args,
            target_distance,
            current_y,
        )
    if mode == "dvl_velocity_compensation":
        return run_dvl_velocity_compensation_experiment, build_step2b_args(
            args,
            target_distance,
            current_y,
        )
    if mode == "dvl_pi_velocity_compensation":
        return run_dvl_pi_velocity_compensation_experiment, build_step2c_args(
            args,
            target_distance,
            current_y,
        )
    raise ValueError(f"Unknown mode: {mode}")


def build_no_compensation_args(
    args: argparse.Namespace,
    target_distance: float,
    current_y: float,
) -> argparse.Namespace:
    return argparse.Namespace(
        target_distance=target_distance,
        forward_command=args.forward_command,
        current_x=args.current_x,
        current_y=current_y,
        current_z=args.current_z,
        ticks_per_sec=args.ticks_per_sec,
        max_duration=args.max_duration,
        warmup_ticks=args.warmup_ticks,
        dvl_forward_index=args.dvl_forward_index,
        dvl_forward_sign=args.dvl_forward_sign,
        speed_warning_threshold=args.speed_warning_threshold,
        max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
        diagnostic_distance_check=False,
        make_current_plots=True,
        current_api_method="env.set_ocean_currents(agent_name, velocity)",
        scenario_name="Step02C_Comparison_No_Compensation",
        experiment_label="Step 2C comparison no-compensation",
        summary_title="Step 2C comparison no-compensation summary",
        world=args.world,
        results_dir=args.results_dir,
        show_viewport=args.show_viewport,
    )


def build_step2b_args(
    args: argparse.Namespace,
    target_distance: float,
    current_y: float,
) -> argparse.Namespace:
    return argparse.Namespace(
        target_distance=target_distance,
        desired_forward_velocity=args.desired_forward_velocity,
        desired_lateral_velocity=args.desired_lateral_velocity,
        current_x=args.current_x,
        current_y=current_y,
        current_z=args.current_z,
        kp_forward=args.kp_forward,
        kp_lateral=args.kp_lateral,
        max_forward_command=args.max_forward_command,
        max_lateral_command=args.max_lateral_command,
        max_thruster_command=args.max_thruster_command,
        base_vertical_command=0.0,
        ticks_per_sec=args.ticks_per_sec,
        max_duration=args.max_duration,
        warmup_ticks=args.warmup_ticks,
        dvl_forward_index=args.dvl_forward_index,
        dvl_forward_sign=args.dvl_forward_sign,
        dvl_lateral_index=args.dvl_lateral_index,
        dvl_lateral_sign=args.dvl_lateral_sign,
        speed_warning_threshold=args.speed_warning_threshold,
        max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
        world=args.world,
        results_dir=args.results_dir,
        show_viewport=args.show_viewport,
    )


def build_step2c_args(
    args: argparse.Namespace,
    target_distance: float,
    current_y: float,
) -> argparse.Namespace:
    return argparse.Namespace(
        target_distance=target_distance,
        desired_forward_velocity=args.desired_forward_velocity,
        desired_lateral_velocity=args.desired_lateral_velocity,
        current_x=args.current_x,
        current_y=current_y,
        current_z=args.current_z,
        kp_surge=args.kp_surge,
        ki_surge=args.ki_surge,
        kp_sway=args.kp_sway,
        ki_sway=args.ki_sway,
        max_surge=args.max_surge,
        max_sway=args.max_sway,
        integral_limit_surge=args.integral_limit_surge,
        integral_limit_sway=args.integral_limit_sway,
        max_thruster_command=args.max_thruster_command,
        ticks_per_sec=args.ticks_per_sec,
        max_duration=args.max_duration,
        warmup_ticks=args.warmup_ticks,
        dvl_forward_index=args.dvl_forward_index,
        dvl_forward_sign=args.dvl_forward_sign,
        dvl_lateral_index=args.dvl_lateral_index,
        dvl_lateral_sign=args.dvl_lateral_sign,
        speed_warning_threshold=args.speed_warning_threshold,
        max_dvl_speed_warning_threshold=args.max_dvl_speed_warning_threshold,
        world=args.world,
        results_dir=args.results_dir,
        show_viewport=args.show_viewport,
    )


def run_experiment_with_retries(
    args: argparse.Namespace,
    run_label: str,
    runner,
    run_args: argparse.Namespace,
    output_dir: Path,
) -> dict:
    attempts = args.retry_count + 1
    for attempt_index in range(1, attempts + 1):
        try:
            return runner(
                run_args,
                output_dir=output_dir,
                print_terminal_summary=False,
            )
        except Exception as exc:
            if attempt_index >= attempts:
                raise
            print(
                "Warning: HoloOcean run failed during attempt "
                f"{attempt_index}/{attempts} for {run_label}: {exc}"
            )
            if args.retry_delay > 0:
                print(f"Waiting {args.retry_delay:g} s before retrying.")
                time.sleep(args.retry_delay)

    raise RuntimeError("unreachable retry state")


def sleep_between_runs(args: argparse.Namespace) -> None:
    if args.run_settle_delay > 0:
        time.sleep(args.run_settle_delay)


def make_run_id(target_distance: float, current_y: float, repetition_index: int) -> str:
    return (
        f"target_{format_float(target_distance)}m_"
        f"current_y_{format_float(current_y)}_"
        f"rep_{repetition_index:02d}"
    )


def format_float(value: float) -> str:
    return f"{value:g}".replace("-", "neg").replace(".", "p")


def summary_to_row(
    summary: dict,
    run_id: str,
    mode: str,
    target_distance: float,
    repetition_index: int,
    args: argparse.Namespace,
) -> dict:
    stop_reason = str(summary["stop_reason"])
    target_reached = bool(summary.get("target_reached", stop_reason == "target_reached"))
    if stop_reason == "timeout":
        target_reached = False

    return {
        "run_id": run_id,
        "mode": mode,
        "target_distance": float(target_distance),
        "repetition_index": int(repetition_index),
        "current_x": float(summary["current_x"]),
        "current_y": float(summary["current_y"]),
        "current_z": float(summary["current_z"]),
        "current_magnitude": float(summary["current_magnitude"]),
        "max_duration_s": float(args.max_duration),
        "forward_command": float(getattr(args, "forward_command", 0.0)),
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "dvl_estimated_distance": float(summary["dvl_estimated_distance"]),
        "pose_forward_displacement": float(summary["pose_forward_displacement"]),
        "pose_euclidean_displacement": float(summary["pose_euclidean_displacement"]),
        "absolute_error": float(summary["absolute_error"]),
        "percentage_error": float(summary["percentage_error"]),
        "lateral_drift": float(summary["lateral_drift"]),
        "final_position_error": float(summary["final_position_error"]),
        "duration": float(summary["duration"]),
        "sample_count": int(summary["sample_count"]),
        "stop_reason": stop_reason,
        "target_reached": target_reached,
        "mean_forward_velocity_error": summary.get("mean_forward_velocity_error", ""),
        "std_forward_velocity_error": summary.get("std_forward_velocity_error", ""),
        "mean_lateral_velocity_error": summary.get("mean_lateral_velocity_error", ""),
        "std_lateral_velocity_error": summary.get("std_lateral_velocity_error", ""),
        "mean_abs_lateral_velocity_error": summary.get(
            "mean_abs_lateral_velocity_error",
            "",
        ),
        "max_abs_lateral_velocity_error": summary.get(
            "max_abs_lateral_velocity_error",
            "",
        ),
        "mean_surge_velocity_error": summary.get("mean_surge_velocity_error", ""),
        "mean_abs_surge_velocity_error": summary.get(
            "mean_abs_surge_velocity_error",
            "",
        ),
        "mean_sway_velocity_error": summary.get("mean_sway_velocity_error", ""),
        "mean_abs_sway_velocity_error": summary.get(
            "mean_abs_sway_velocity_error",
            "",
        ),
        "mean_command_effort": summary.get("mean_command_effort", ""),
        "max_command_effort": summary.get("max_command_effort", ""),
        "saturation_fraction": summary.get("saturation_fraction", ""),
        "surge_saturation_fraction": summary.get("surge_saturation_fraction", ""),
        "sway_saturation_fraction": summary.get("sway_saturation_fraction", ""),
        "mean_dt": float(summary["mean_dt"]),
        "std_dt": float(summary["std_dt"]),
        "dt_source": summary["dt_source"],
    }


def write_all_runs_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def write_comparison_outputs(
    args: argparse.Namespace,
    batch_dir: Path,
    rows: list[dict],
    expected_mode_run_count: int,
    write_plots: bool,
) -> dict:
    write_all_runs_csv(batch_dir / "all_runs_summary.csv", rows)
    aggregate_summary = build_aggregate_summary(
        args,
        batch_dir,
        rows,
        expected_mode_run_count,
    )
    write_json(batch_dir / "aggregate_summary.json", aggregate_summary)

    if write_plots and rows:
        plot_mode_metric_comparison(
            rows,
            "lateral_drift",
            "Lateral drift [m]",
            batch_dir / "lateral_drift_comparison.png",
        )
        plot_mode_metric_comparison(
            rows,
            "final_position_error",
            "Final position error [m]",
            batch_dir / "final_position_error_comparison.png",
        )
        plot_mode_metric_comparison(
            rows,
            "duration",
            "Duration [s]",
            batch_dir / "duration_comparison.png",
        )
        plot_mode_metric_comparison(
            rows,
            "target_reached",
            "Target reached rate",
            batch_dir / "target_reached_rate_comparison.png",
        )
        plot_saturation_summary(rows, batch_dir / "saturation_summary.png")

    return aggregate_summary


def build_aggregate_summary(
    args: argparse.Namespace,
    batch_dir: Path,
    rows: list[dict],
    expected_mode_run_count: int,
) -> dict:
    groups = []
    grouped_rows: dict[tuple[float, float], list[dict]] = defaultdict(list)
    for row in rows:
        grouped_rows[(row["target_distance"], row["current_y"])].append(row)

    for key in sorted(grouped_rows):
        target_distance, current_y = key
        group_rows = grouped_rows[key]
        rows_by_mode = {
            mode: [row for row in group_rows if row["mode"] == mode]
            for mode in MODES
        }
        group = {
            "target_distance": float(target_distance),
            "current_y": float(current_y),
            "number_of_pairs": min(len(rows_by_mode[mode]) for mode in MODES),
            "max_duration_s": float(args.max_duration),
        }

        for mode in MODES:
            mode_rows = rows_by_mode[mode]
            group[f"{mode}_mean_lateral_drift"] = mean_metric(
                mode_rows,
                "lateral_drift",
            )
            group[f"{mode}_mean_final_position_error"] = mean_metric(
                mode_rows,
                "final_position_error",
            )
            group[f"{mode}_mean_duration"] = mean_metric(mode_rows, "duration")
            group[f"{mode}_target_reached_rate"] = target_reached_rate(mode_rows)
            group[f"{mode}_timeout_rate"] = timeout_rate(mode_rows)

        pi_rows = rows_by_mode["dvl_pi_velocity_compensation"]
        group["dvl_pi_velocity_compensation_saturation_fraction"] = mean_optional_metric(
            pi_rows,
            "saturation_fraction",
        )
        group[
            "dvl_pi_velocity_compensation_surge_saturation_fraction"
        ] = mean_optional_metric(pi_rows, "surge_saturation_fraction")
        group[
            "dvl_pi_velocity_compensation_sway_saturation_fraction"
        ] = mean_optional_metric(pi_rows, "sway_saturation_fraction")

        no_comp_rows = rows_by_mode["no_compensation"]
        no_comp_lateral = group["no_compensation_mean_lateral_drift"]
        no_comp_final = group["no_compensation_mean_final_position_error"]
        for mode in ("dvl_velocity_compensation", "dvl_pi_velocity_compensation"):
            mode_rows = rows_by_mode[mode]
            lateral_reduction = reduction_percentage(
                no_comp_lateral,
                group[f"{mode}_mean_lateral_drift"],
            )
            final_reduction = reduction_percentage(
                no_comp_final,
                group[f"{mode}_mean_final_position_error"],
            )
            valid = reduction_is_valid(no_comp_rows, mode_rows)
            group[f"{mode}_lateral_drift_reduction_percentage"] = lateral_reduction
            group[f"{mode}_lateral_drift_reduction_valid"] = valid
            group[
                f"{mode}_final_position_error_reduction_percentage"
            ] = final_reduction
            group[f"{mode}_final_position_error_reduction_valid"] = valid

        groups.append(group)

    return {
        "output_dir": str(batch_dir),
        "target_distances": [float(value) for value in args.target_distances],
        "current_y_values": [float(value) for value in args.current_y_values],
        "repetitions": int(args.repetitions),
        "max_duration_s": float(args.max_duration),
        "expected_mode_runs": int(expected_mode_run_count),
        "completed_mode_runs": len(rows),
        "complete": len(rows) == expected_mode_run_count,
        "desired_forward_velocity": float(args.desired_forward_velocity),
        "desired_lateral_velocity": float(args.desired_lateral_velocity),
        "modes": MODES,
        "groups": groups,
    }


def mean_metric(rows: list[dict], key: str) -> float:
    if not rows:
        return 0.0
    return float(np.mean([float(row[key]) for row in rows]))


def mean_optional_metric(rows: list[dict], key: str) -> float:
    values = [float(row[key]) for row in rows if row.get(key, "") != ""]
    if not values:
        return 0.0
    return float(np.mean(values))


def target_reached_rate(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(np.mean([1.0 if bool(row["target_reached"]) else 0.0 for row in rows]))


def timeout_rate(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(np.mean([1.0 if row["stop_reason"] == "timeout" else 0.0 for row in rows]))


def reduction_percentage(baseline: float, improved: float) -> float:
    if abs(baseline) < 1e-9:
        return 0.0
    return float(100.0 * (baseline - improved) / abs(baseline))


def reduction_is_valid(baseline_rows: list[dict], compared_rows: list[dict]) -> bool:
    return bool(
        baseline_rows
        and compared_rows
        and target_reached_rate(baseline_rows) == 1.0
        and target_reached_rate(compared_rows) == 1.0
        and timeout_rate(baseline_rows) == 0.0
        and timeout_rate(compared_rows) == 0.0
    )


def plot_saturation_summary(rows: list[dict], output_path: Path | str) -> Path:
    pi_rows = [row for row in rows if row["mode"] == "dvl_pi_velocity_compensation"]
    path = Path(output_path)
    if not pi_rows:
        fig, ax = plt.subplots(figsize=(8, 4.8))
        ax.set_xlabel("Lateral current y [m/s]")
        ax.set_ylabel("Saturation fraction")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return path

    fig, ax = plt.subplots(figsize=(8, 4.8))
    targets = sorted({float(row["target_distance"]) for row in pi_rows})
    for target in targets:
        target_rows = [row for row in pi_rows if float(row["target_distance"]) == target]
        current_values = sorted({float(row["current_y"]) for row in target_rows})
        saturation_values = []
        for current_y in current_values:
            values = [
                float(row["saturation_fraction"])
                for row in target_rows
                if float(row["current_y"]) == current_y
                and row.get("saturation_fraction", "") != ""
            ]
            saturation_values.append(float(np.mean(values)) if values else 0.0)
        ax.plot(
            current_values,
            saturation_values,
            marker="o",
            label=f"target={target:g} m",
        )

    ax.set_xlabel("Lateral current y [m/s]")
    ax.set_ylabel("Step 2C saturation fraction")
    ax.set_ylim(bottom=0.0, top=1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize="small")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
        file.write("\n")


if __name__ == "__main__":
    main()
