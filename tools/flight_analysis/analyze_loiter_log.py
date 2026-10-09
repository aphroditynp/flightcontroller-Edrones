#!/usr/bin/env python3
"""Calculate loiter metrics from an SD logger CSV captured during flight."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


METRIC_NAMES = (
    "radial_error_rms_m",
    "radial_error_mean_abs_m",
    "radial_error_max_abs_m",
    "final_radius_m",
    "target_radius_m",
    "period_mean_s",
)


def analyze_log(path: Path, transient_fraction: float = 0.25) -> dict[str, float]:
    """Analyze finite LOIT rows, excluding the initial capture transient."""
    with path.open(newline="", encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))

    loiter_rows = [
        row for row in rows
        if row.get("mode") == "LOIT"
        and all(_is_finite(row.get(field, "")) for field in (
            "radial_error_m", "target_radius_m", "l1_period_s"
        ))
    ]
    if not loiter_rows:
        raise ValueError(f"No complete LOIT rows found in {path}")

    if not 0.0 <= transient_fraction < 1.0:
        raise ValueError("transient_fraction must be in [0, 1)")
    steady_start = int(transient_fraction * len(loiter_rows))
    steady_rows = loiter_rows[steady_start:] or loiter_rows

    errors = [float(row["radial_error_m"]) for row in steady_rows]
    periods = [float(row["l1_period_s"]) for row in steady_rows]
    final_row = steady_rows[-1]
    target_radius = float(final_row["target_radius_m"])
    final_radius = target_radius + float(final_row["radial_error_m"])
    return {
        "radial_error_rms_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "radial_error_mean_abs_m": sum(abs(error) for error in errors) / len(errors),
        "radial_error_max_abs_m": max(abs(error) for error in errors),
        "final_radius_m": final_radius,
        "target_radius_m": target_radius,
        "period_mean_s": sum(periods) / len(periods),
    }


def _is_finite(value: str) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="LOGnnn.CSV copied from the SD card")
    parser.add_argument("--transient-fraction", type=float, default=0.25,
                        help="fraction of LOIT rows to exclude at the start (default: 0.25)")
    parser.add_argument("--json", dest="json_path", type=Path,
                        help="also write the metrics as a JSON file")
    args = parser.parse_args()

    metrics = analyze_log(args.csv_path, args.transient_fraction)
    print(json.dumps(metrics, indent=2))
    if args.json_path:
        args.json_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
