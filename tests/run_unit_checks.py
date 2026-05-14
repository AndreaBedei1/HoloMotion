from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from controllers.dvl_velocity_controller import DVLVelocityTrackingController
from estimators.dvl_distance import DVLDistanceEstimator
from lib.current import EXTREME_CURRENT_WARNING, CurrentConfig
from metrics.distance import compute_distance_metrics


def assert_close(actual: float, expected: float, tolerance: float = 1e-9) -> None:
    if abs(float(actual) - float(expected)) > tolerance:
        raise AssertionError(f"Expected {expected}, got {actual}.")


def assert_vector_close(actual, expected, tolerance: float = 1e-9) -> None:
    actual_array = np.asarray(actual, dtype=float)
    expected_array = np.asarray(expected, dtype=float)
    if not np.allclose(actual_array, expected_array, atol=tolerance, rtol=0.0):
        raise AssertionError(f"Expected {expected_array}, got {actual_array}.")


def test_dvl_distance_estimator_integrates_velocity() -> None:
    estimator = DVLDistanceEstimator(forward_axis_index=0, forward_axis_sign=1.0)
    for _ in range(10):
        estimator.update([0.5, 0.0, 0.0], dt_s=0.2)
    assert_close(estimator.distance_m, 1.0)
    assert_close(estimator.last_raw_forward_velocity_mps, 0.5)
    assert_close(estimator.last_used_forward_velocity_mps, 0.5)


def test_current_config_magnitude_and_warning() -> None:
    moderate = CurrentConfig(1.0, 2.0, 2.0)
    assert_close(moderate.magnitude, 3.0)
    if moderate.warning() != "":
        raise AssertionError("A 3.0 m/s current should not trigger the >3.0 warning.")

    extreme = CurrentConfig(0.0, 3.1, 0.0)
    assert_close(extreme.magnitude, 3.1)
    if extreme.warning() != EXTREME_CURRENT_WARNING:
        raise AssertionError("Extreme current warning text does not match.")


def test_velocity_controller_zero_error() -> None:
    controller = DVLVelocityTrackingController(
        kp_forward=10.0,
        kp_lateral=10.0,
        max_forward_command=2.0,
        max_lateral_command=2.0,
        max_thruster_command=2.0,
    )
    command = controller.command(
        desired_forward_velocity=0.3,
        desired_lateral_velocity=0.0,
        measured_forward_velocity=0.3,
        measured_lateral_velocity=0.0,
    )
    assert_vector_close(command[4:8], [0.0, 0.0, 0.0, 0.0])


def test_velocity_controller_forward_error_pattern() -> None:
    controller = DVLVelocityTrackingController(
        kp_forward=1.0,
        kp_lateral=1.0,
        max_forward_command=2.0,
        max_lateral_command=2.0,
        max_thruster_command=2.0,
    )
    command = controller.command(
        desired_forward_velocity=1.0,
        desired_lateral_velocity=0.0,
        measured_forward_velocity=0.0,
        measured_lateral_velocity=0.0,
    )
    assert_vector_close(command[4:8], [1.0, 1.0, 1.0, 1.0])


def test_velocity_controller_lateral_error_pattern() -> None:
    controller = DVLVelocityTrackingController(
        kp_forward=1.0,
        kp_lateral=1.0,
        max_forward_command=2.0,
        max_lateral_command=2.0,
        max_thruster_command=2.0,
    )
    command = controller.command(
        desired_forward_velocity=0.0,
        desired_lateral_velocity=1.0,
        measured_forward_velocity=0.0,
        measured_lateral_velocity=0.0,
    )
    assert_vector_close(command[4:8], [1.0, -1.0, 1.0, -1.0])


def test_velocity_controller_final_clipping() -> None:
    controller = DVLVelocityTrackingController(
        kp_forward=10.0,
        kp_lateral=10.0,
        max_forward_command=2.0,
        max_lateral_command=2.0,
        max_thruster_command=1.5,
    )
    command = controller.command(
        desired_forward_velocity=1.0,
        desired_lateral_velocity=1.0,
        measured_forward_velocity=0.0,
        measured_lateral_velocity=0.0,
    )
    assert_vector_close(command[4:8], [1.5, 0.0, 1.5, 0.0])
    if not controller.last_final_clipping_applied:
        raise AssertionError("Expected final clipping on the last command.")
    if not controller.final_clipping_applied:
        raise AssertionError("Expected cumulative final clipping flag to be set.")


def test_compute_distance_metrics_final_position_error() -> None:
    samples = [
        {
            "time": 0.0,
            "pose_x": 0.0,
            "pose_y": 0.0,
            "pose_z": 0.0,
            "pose_forward_x": 1.0,
            "pose_forward_y": 0.0,
            "pose_forward_z": 0.0,
            "pose_right_x": 0.0,
            "pose_right_y": 1.0,
            "pose_right_z": 0.0,
            "dvl_distance_estimated": 0.0,
        },
        {
            "time": 5.0,
            "pose_x": 3.0,
            "pose_y": 4.0,
            "pose_z": 0.0,
            "pose_forward_x": 1.0,
            "pose_forward_y": 0.0,
            "pose_forward_z": 0.0,
            "pose_right_x": 0.0,
            "pose_right_y": 1.0,
            "pose_right_z": 0.0,
            "dvl_distance_estimated": 3.0,
        },
    ]
    metrics = compute_distance_metrics(
        samples=samples,
        target_distance_m=3.0,
        stop_reason="target_reached",
    )
    assert_close(metrics.pose_forward_displacement_m, 3.0)
    assert_close(metrics.lateral_drift_m, 4.0)
    assert_close(metrics.pose_ground_truth_displacement_m, 5.0)
    assert_close(metrics.final_position_error_m, 4.0)


def main() -> None:
    checks = [
        test_dvl_distance_estimator_integrates_velocity,
        test_current_config_magnitude_and_warning,
        test_velocity_controller_zero_error,
        test_velocity_controller_forward_error_pattern,
        test_velocity_controller_lateral_error_pattern,
        test_velocity_controller_final_clipping,
        test_compute_distance_metrics_final_position_error,
    ]
    for check in checks:
        check()
        print(f"PASS {check.__name__}")
    print(f"All {len(checks)} unit checks passed.")


if __name__ == "__main__":
    main()
