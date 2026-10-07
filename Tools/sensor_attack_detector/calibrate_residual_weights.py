#!/usr/bin/env python3

"""Calibrate V2 A/V/P residual weights from independent nominal ULogs.

The detector publishes post-H0 residual RMS values for acceleration, velocity,
and position on both horizontal axes. This tool estimates one physical noise
scale per residual order with equal flight weighting and returns inverse-
variance weights. Online code additionally divides each order by its sample
count, so these parameters provide the remaining 1/sigma_g^2 factor.
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
    raise SystemExit("pyulog is required to calibrate residual weights") from exc


FROZEN_PARAMETERS = (
    "SAD_ACT_SRC",
    "SAD_THR_GAIN",
    "SAD_THR_MAP",
    "SAD_DRAG_K",
    "SAD_DRAG_MOD",
    "SAD_DYN_LAG",
    "SAD_REG",
    "SAD_AS_MU",
    "SAD_AD_MU",
    "SAD_WA",
    "SAD_WV",
    "SAD_WP",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-valid-windows", type=int, default=20)
    parser.add_argument("--minimum-flights", type=int, default=3)
    parser.add_argument("--minimum-accel-sigma", type=float, default=1e-3)
    parser.add_argument("--minimum-velocity-sigma", type=float, default=1e-3)
    parser.add_argument("--minimum-position-sigma", type=float, default=1e-3)
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
                    f"line {line_number}: residual-weight calibration accepts nominal flights only"
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

    if len(rows) == 0:
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


def get_effective_parameter(ulog, name):
    value = ulog.initial_parameters.get(name)

    for _, changed_name, changed_value in ulog.changed_parameters:
        if changed_name == name:
            value = changed_value

    return value


def load_flight(row, minimum_windows):
    ulog = ULog(str(row["ulog"]), None, disable_str_exceptions=True)
    status = get_dataset(ulog, "sensor_attack_status")
    vehicle_status = get_dataset(ulog, "vehicle_status")
    required = {
        "valid",
        "data_quality_flags",
        "normal_rms_accel_n",
        "normal_rms_accel_e",
        "normal_rms_velocity_n",
        "normal_rms_velocity_e",
        "normal_rms_position_n",
        "normal_rms_position_e",
    }
    missing = sorted(required - set(status))

    if missing:
        raise ValueError(f"{row['ulog']}: missing V2 RMS fields: {missing}")

    if np.count_nonzero(np.asarray(vehicle_status["failsafe"], dtype=bool)):
        raise ValueError(f"{row['flight_id']}: nominal flight contains failsafe samples")

    valid = np.asarray(status["valid"], dtype=bool)
    quality = np.asarray(status["data_quality_flags"], dtype=np.uint32)
    usable = valid & (quality == 0)

    if np.count_nonzero(usable) < minimum_windows:
        raise ValueError(
            f"{row['flight_id']}: only {np.count_nonzero(usable)} usable windows; "
            f"need at least {minimum_windows}"
        )

    order_variances = {}

    for order in ("accel", "velocity", "position"):
        values = np.column_stack(
            [
                np.asarray(status[f"normal_rms_{order}_n"], dtype=float),
                np.asarray(status[f"normal_rms_{order}_e"], dtype=float),
            ]
        )
        mask = usable & np.all(np.isfinite(values), axis=1)

        if np.count_nonzero(mask) < minimum_windows:
            raise ValueError(
                f"{row['flight_id']}: insufficient finite {order} RMS windows"
            )

        # Each window/axis RMS^2 is an estimate of residual variance.
        order_variances[order] = float(np.mean(values[mask] ** 2))

    parameters = {
        name: get_effective_parameter(ulog, name)
        for name in FROZEN_PARAMETERS
    }

    if any(value is None for value in parameters.values()):
        missing_parameters = sorted(
            name for name, value in parameters.items() if value is None
        )
        raise ValueError(
            f"{row['flight_id']}: missing frozen parameters: {missing_parameters}"
        )

    return {
        **row,
        "parameters": parameters,
        "usable_windows": int(np.count_nonzero(usable)),
        **{f"{order}_variance": value for order, value in order_variances.items()},
    }


def verify_frozen_parameters(flights):
    reference = flights[0]["parameters"]

    for flight in flights[1:]:
        for name in FROZEN_PARAMETERS:
            left = float(reference[name])
            right = float(flight["parameters"][name])

            if not math.isclose(left, right, rel_tol=1e-7, abs_tol=1e-7):
                raise ValueError(
                    f"{flight['flight_id']}: {name}={right} does not match {left}"
                )

    return {name: float(value) for name, value in reference.items()}


def equal_flight_sigma(flights, order, minimum_sigma):
    variance = float(
        np.mean([flight[f"{order}_variance"] for flight in flights])
    )
    return max(math.sqrt(max(0.0, variance)), minimum_sigma)


def main():
    args = parse_args()

    rows = read_manifest(args.manifest)

    if len(rows) < args.minimum_flights:
        raise ValueError(
            f"need at least {args.minimum_flights} independent flights; got {len(rows)}"
        )

    flights = [
        load_flight(row, args.minimum_valid_windows)
        for row in rows
    ]
    frozen = verify_frozen_parameters(flights)
    sigma_a = equal_flight_sigma(flights, "accel", args.minimum_accel_sigma)
    sigma_v = equal_flight_sigma(flights, "velocity", args.minimum_velocity_sigma)
    sigma_p = equal_flight_sigma(flights, "position", args.minimum_position_sigma)
    candidate = {
        "SAD_WA": 1.0 / (sigma_a * sigma_a),
        "SAD_WV": 1.0 / (sigma_v * sigma_v),
        "SAD_WP": 1.0 / (sigma_p * sigma_p),
    }
    report = {
        "schema": "px4.sensor_attack_detector.residual_weight_calibration.v1",
        "nominal_only": True,
        "flight_weighting": "equal_independent_flight",
        "frozen_parameters": frozen,
        "residual_sigma": {
            "acceleration": sigma_a,
            "velocity": sigma_v,
            "position": sigma_p,
        },
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
