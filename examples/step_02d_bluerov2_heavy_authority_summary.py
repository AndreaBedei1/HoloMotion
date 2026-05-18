from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from actuation.bluerov2_heavy_authority import BlueROV2HeavyAuthority
from actuation.bluerov2_heavy_config import BlueROV2HeavyThrusterLayout
from actuation.t200_thruster_model import T200ThrusterModel


def _format_thrust(kgf: float, newtons: float) -> str:
    return f"{kgf:.2f} kgf ({newtons:.2f} N)"


def _print_authority_summary(label: str, model: T200ThrusterModel) -> None:
    layout = BlueROV2HeavyThrusterLayout()
    authority = BlueROV2HeavyAuthority(layout=layout, thruster_model=model)
    performance = model.performance

    print(f"\n{label}")
    print("-" * len(label))
    print(
        "Per-thruster forward thrust: "
        f"{_format_thrust(performance.max_forward_thrust_kgf, performance.max_forward_thrust_n)}"
    )
    print(
        "Per-thruster reverse thrust: "
        f"{_format_thrust(performance.max_reverse_thrust_kgf, performance.max_reverse_thrust_n)}"
    )
    print(f"Full-throttle current: {performance.max_current_a:.0f} A")
    print(f"Full-throttle power: {performance.max_power_w:.0f} W")
    print(
        "Raw 8-thruster forward/reverse sum: "
        f"{authority.total_8_thruster_forward_sum_n:.2f} N / "
        f"{authority.total_8_thruster_reverse_sum_n:.2f} N"
    )
    print(
        "Horizontal 4-thruster forward/reverse sum: "
        f"{authority.horizontal_4_thruster_forward_sum_n:.2f} N / "
        f"{authority.horizontal_4_thruster_reverse_sum_n:.2f} N"
    )
    print(
        "Vertical 4-thruster forward/reverse sum: "
        f"{authority.vertical_4_thruster_forward_sum_n:.2f} N / "
        f"{authority.vertical_4_thruster_reverse_sum_n:.2f} N"
    )
    print(
        "Forward/reverse asymmetry ratio: "
        f"{authority.forward_reverse_asymmetry_ratio:.3f}"
    )


def main() -> None:
    print("BlueROV2 Heavy-style / T200 actuator authority summary")
    print("======================================================")
    print("Layout: 8 total T200 thrusters, 4 horizontal, 4 vertical")

    _print_authority_summary("12 V", T200ThrusterModel.from_12v())
    _print_authority_summary("16 V nominal", T200ThrusterModel.from_nominal_16v())
    _print_authority_summary("20 V maximum", T200ThrusterModel.from_20v())

    print("\nWarnings")
    print("--------")
    print("HoloOcean command units are not assumed to be Newtons.")
    print("This is an actuator authority model, not a real motor-order map.")
    print(
        "The current Step 2C saturation_fraction is saturation relative to "
        "software limits, not verified physical T200 saturation."
    )

    print("\nRecommended next experiment")
    print("---------------------------")
    print(
        "Run a Step 2D actuator-authority sweep over max_thruster_command "
        "values and controller limits to determine whether current_y=2.0 "
        "fails because of conservative software limits, mixer allocation, "
        "or true vehicle authority limitations."
    )


if __name__ == "__main__":
    main()
