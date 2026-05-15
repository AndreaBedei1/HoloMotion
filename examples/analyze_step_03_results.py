from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from experiments.altitude_hold import (
    build_step_03_aggregate,
    print_step_03_aggregate_table,
    write_aggregate_csv,
    write_plots_for_batch,
)


def main() -> None:
    args = parse_args()
    rows = read_summary_rows(args.batch_dir / "summary.csv")
    aggregate_rows = build_step_03_aggregate(rows)
    write_aggregate_csv(args.batch_dir / "aggregate_by_condition.csv", aggregate_rows)
    if not args.no_plots:
        write_plots_for_batch(args.batch_dir, rows)
    print_step_03_aggregate_table(aggregate_rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate Step 3 aggregate CSV and plots from a batch summary.csv."
    )
    parser.add_argument("batch_dir", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args()


def read_summary_rows(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(f"Step 3 summary CSV not found: {path}")
    with path.open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


if __name__ == "__main__":
    main()
