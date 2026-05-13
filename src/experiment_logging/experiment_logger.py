from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

import numpy as np


class ExperimentLogger:
    """Write experiment configuration, trajectory samples, and summaries."""

    def __init__(self, output_dir: Path):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @classmethod
    def create_timestamped(cls, base_dir: Path | str) -> "ExperimentLogger":
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return cls(Path(base_dir) / timestamp)

    def write_trajectory(self, samples: Sequence[dict], filename: str = "trajectory.csv") -> Path:
        path = self.output_dir / filename
        if not samples:
            raise ValueError("Cannot write an empty trajectory.")

        fieldnames = list(samples[0].keys())
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for sample in samples:
                writer.writerow({key: _to_jsonable(value) for key, value in sample.items()})
        return path

    def write_summary(self, summary: dict, filename: str = "summary.json") -> Path:
        path = self.output_dir / filename
        with path.open("w", encoding="utf-8") as file:
            json.dump(_to_jsonable(summary), file, indent=2)
            file.write("\n")
        return path

    def write_run_config(self, config: dict, filename: str = "run_config.yaml") -> Path:
        path = self.output_dir / filename
        with path.open("w", encoding="utf-8") as file:
            _write_yaml_mapping(file, _to_jsonable(config))
        return path


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _to_jsonable(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def _write_yaml_mapping(file, data: dict, indent: int = 0) -> None:
    prefix = " " * indent
    for key, value in data.items():
        if isinstance(value, dict):
            file.write(f"{prefix}{key}:\n")
            _write_yaml_mapping(file, value, indent + 2)
        else:
            file.write(f"{prefix}{key}: {_format_yaml_scalar(value)}\n")


def _format_yaml_scalar(value: Any) -> str:
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    return str(value)
