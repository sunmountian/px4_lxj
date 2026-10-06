#!/usr/bin/env python3

"""Calibrate V2 normal-model priors from independent nominal ULogs.

The PX4 detector publishes per-window normal-model estimates in
sensor_attack_status. This tool uses only valid, zero-quality-flag nominal
windows and gives each independent flight equal weight when estimating the
common horizontal actuator-scale and actuator-dynamic priors.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required to calibrate normal priors") from exc


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-valid-windows", type=int, default=20)
    parser.add_argument("--minimum-scale-sd", type=float, default=0.02)
    parser.add_argument("--minimum-dynamic-sd", type=float, default=0.05)
    return parser.parse_args()


def read_manifest(path):
    rows = []

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"flight_id", "scenario", "nominal", "ulog"}

        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            missing = sorted(required - set(reader.fieldnames or []))
            raise ValueError(f"manifest is missing columns: {missing}")

        for line_number, row in enumerate(reader, start=2):
            if row["nominal"].strip().lower() not in {"1", "true", "yes"}:
                raise ValueError(
                    f"line {line_number}: normal-prior calibration accepts nominal flights only"
                )

            ulog = Path(row["ulog"]).expanduser()

            if not ulog.is_absolute():
                ulog = (path.parent / ulog).resolve()

            if not ulog.is_file():
                raise FileNotFoundError(f"line {line_number}: ULog not found: {ulog}")

            rows.append(
                {
                    "flight_id": row["flight_id"].strip(),
                    "scenario": row["scenario"].strip(),
                    "ulog": ulog,
                }
            )

    if not rows:
        raise ValueError("manifest contains no flights")

    identifiers = [row["flight_id"] for row in rows]

    if len(set(identifiers)) != len(identifiers):
        raise ValueError("flight_id values must be unique")

    return rows


def get_dataset(ulog, name):
    matches = [dataset for dataset in ulog.data_list if dataset.name == name]

    if len(matches) != 1:
        raise ValueError(f"{name}: expected one instance, found {len(matches)}")

    return matches[0].data


def load_flight(row, minimum_windows):
    ulog = ULog(str(row["ulog"]), None, disable_str_exceptions=True)
    status = get_dataset(ulog, "sensor_attack_status")
    vehicle_status = get_dataset(ulog, "vehicle_status")
    required = {
        "valid",
        "data_quality_flags",
        "normal_scale_n",
        "normal_scale_e",
        "normal_dynamic_n",
        "normal_dynamic_e",
    }
    missing = sorted(required - set(status))

    if missing:
        raise ValueError(f"{row['ulog']}: missing V2 status fields: {missing}")

    if np.count_nonzero(np.asarray(vehicle_status["failsafe"], dtype=bool)):
        raise ValueError(f"{row['flight_id']}: nominal flight contains failsafe samples")

    valid = np.asarray(status["valid"], dtype=bool)
    quality = np.asarray(status["data_quality_flags"], dtype=np.uint32)
    scale = np.column_stack(
        [
            np.asarray(status["normal_scale_n"], dtype=float),
            np.asarray(status["normal_scale_e"], dtype=float),
        ]
    )
    dynamic = np.column_stack(
        [
            np.asarray(status["normal_dynamic_n"], dtype=float),
            np.asarray(status["normal_dynamic_e"], dtype=float),
        ]
    )
    usable = (
        valid
        & (quality == 0)
        & np.all(np.isfinite(scale), axis=1)
        & np.all(np.isfinite(dynamic), axis=1)
    )

    if np.count_nonzero(usable) < minimum_windows:
        raise ValueError(
            f"{row['flight_id']}: only {np.count_nonzero(usable)} usable windows; "
            f"need at least {minimum_windows}"
        )

    scale_values = scale[usable].reshape(-1)
    dynamic_values = dynamic[usable].reshape(-1)
    return {
        **row,
        "usable_windows": int(np.count_nonzero(usable)),
        "scale_mean": float(np.mean(scale_values)),
        "scale_variance": float(np.var(scale_values, ddof=1)),
        "dynamic_mean": float(np.mean(dynamic_values)),
        "dynamic_variance": float(np.var(dynamic_values, ddof=1)),
    }


def equal_flight_moments(flights, prefix):
    means = np.asarray([flight[f"{prefix}_mean"] for flight in flights], dtype=float)
    variances = np.asarray(
        [flight[f"{prefix}_variance"] for flight in flights], dtype=float
    )
    mean = float(np.mean(means))
    variance = float(np.mean(variances + (means - mean) ** 2))
    return mean, math.sqrt(max(0.0, variance))


def main():
    args = parse_args()

    if args.minimum_valid_windows < 2:
        raise ValueError("--minimum-valid-windows must be at least two")

    rows = read_manifest(args.manifest)
    flights = [
        load_flight(row, args.minimum_valid_windows)
        for row in rows
    ]
    scale_mean, scale_sd = equal_flight_moments(flights, "scale")
    dynamic_mean, dynamic_sd = equal_flight_moments(flights, "dynamic")
    scale_sd = max(scale_sd, args.minimum_scale_sd)
    dynamic_sd = max(dynamic_sd, args.minimum_dynamic_sd)
    candidate = {
        "SAD_AS_MU": scale_mean,
        "SAD_AS_SD": scale_sd,
        "SAD_AD_MU": dynamic_mean,
        "SAD_AD_SD": dynamic_sd,
    }
    report = {
        "schema": "px4.sensor_attack_detector.normal_prior_calibration.v1",
        "nominal_only": True,
        "flight_weighting": "equal_independent_flight",
        "candidate_parameters": candidate,
        "flights": [
            {**flight, "ulog": str(flight["ulog"])}
            for flight in flights
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(candidate, indent=2, sort_keys=True))
    print(f"report={args.output}")


if __name__ == "__main__":
    main()
