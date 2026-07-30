#!/usr/bin/env python3

"""Estimate SAD_THR_GAIN from dedicated nominal level-hover ULogs.

This is a physical input-model calibration, not an attack detector.  For a
steady level hover, the vertical component of actuator-produced acceleration
balances gravity:

    SAD_THR_GAIN * sum(normalized_motor_output) * R_zz = g.

Only landed-state, velocity, and tilt checks select usable hover samples.
One median estimate is formed per independent flight, followed by a median
across flights so that long logs do not receive extra weight.
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
    raise SystemExit("pyulog is required to calibrate actuator gain") from exc


GRAVITY_M_S2 = 9.80665


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate SAD_THR_GAIN from independent nominal hover ULogs."
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pwm-min", type=float, default=1000.0)
    parser.add_argument("--pwm-max", type=float, default=2000.0)
    parser.add_argument("--maximum-horizontal-speed", type=float, default=0.15)
    parser.add_argument("--maximum-vertical-speed", type=float, default=0.10)
    parser.add_argument("--maximum-tilt-degrees", type=float, default=5.0)
    parser.add_argument("--minimum-samples-per-flight", type=int, default=20)
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
            nominal = row["nominal"].strip().lower()
            scenario = row["scenario"].strip().lower()

            if nominal not in {"1", "true", "yes"}:
                raise ValueError(
                    f"line {line_number}: actuator-gain calibration accepts "
                    "nominal flights only"
                )

            if scenario != "hover":
                raise ValueError(
                    f"line {line_number}: scenario must be 'hover', got {scenario!r}"
                )

            ulog = Path(row["ulog"]).expanduser()

            if not ulog.is_absolute():
                ulog = (path.parent / ulog).resolve()

            if not ulog.is_file():
                raise FileNotFoundError(f"line {line_number}: ULog not found: {ulog}")

            rows.append(
                {
                    "flight_id": row["flight_id"].strip(),
                    "scenario": scenario,
                    "ulog": ulog,
                }
            )

    identifiers = [row["flight_id"] for row in rows]

    if len(set(identifiers)) != len(identifiers):
        raise ValueError("flight_id values must be unique")

    if not rows:
        raise ValueError("manifest contains no hover flights")

    return rows


def get_dataset(ulog, name):
    datasets = [dataset for dataset in ulog.data_list if dataset.name == name]

    if len(datasets) != 1:
        raise ValueError(
            f"{name}: expected exactly one instance, found {len(datasets)}"
        )

    return datasets[0].data


def previous_sample(source_timestamp, source_values, target_timestamp):
    indices = np.searchsorted(source_timestamp, target_timestamp, side="right") - 1
    indices = np.clip(indices, 0, source_timestamp.size - 1)
    return source_values[indices]


def interpolated(source_timestamp, source_values, target_timestamp):
    return np.interp(target_timestamp, source_timestamp, source_values)


def estimate_flight_gain(
    row,
    pwm_min,
    pwm_max,
    maximum_horizontal_speed,
    maximum_vertical_speed,
    maximum_tilt_degrees,
    minimum_samples,
):
    ulog = ULog(str(row["ulog"]), None, disable_str_exceptions=True)
    actuator = get_dataset(ulog, "actuator_outputs")
    attitude = get_dataset(ulog, "vehicle_attitude")
    local_position = get_dataset(ulog, "vehicle_local_position")
    land = get_dataset(ulog, "vehicle_land_detected")

    actuator_timestamp = np.asarray(actuator["timestamp"], dtype=np.uint64)
    attitude_timestamp = np.asarray(attitude["timestamp_sample"], dtype=np.uint64)
    position_timestamp = np.asarray(local_position["timestamp_sample"], dtype=np.uint64)
    land_timestamp = np.asarray(land["timestamp"], dtype=np.uint64)

    motor_outputs = np.column_stack(
        [np.asarray(actuator[f"output[{index}]"], dtype=float) for index in range(4)]
    )
    normalized = np.clip(
        (motor_outputs - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0
    )
    thrust_indicator = np.sum(normalized, axis=1)

    vx = interpolated(
        position_timestamp,
        np.asarray(local_position["vx"], dtype=float),
        actuator_timestamp,
    )
    vy = interpolated(
        position_timestamp,
        np.asarray(local_position["vy"], dtype=float),
        actuator_timestamp,
    )
    vz = interpolated(
        position_timestamp,
        np.asarray(local_position["vz"], dtype=float),
        actuator_timestamp,
    )
    xy_valid = previous_sample(
        position_timestamp,
        np.asarray(local_position["v_xy_valid"], dtype=bool),
        actuator_timestamp,
    )
    z_valid = previous_sample(
        position_timestamp,
        np.asarray(local_position["v_z_valid"], dtype=bool),
        actuator_timestamp,
    )
    landed = previous_sample(
        land_timestamp,
        np.asarray(land["landed"], dtype=bool),
        actuator_timestamp,
    )

    quaternion = np.column_stack(
        [
            interpolated(
                attitude_timestamp,
                np.asarray(attitude[f"q[{index}]"], dtype=float),
                actuator_timestamp,
            )
            for index in range(4)
        ]
    )
    quaternion_norm = np.linalg.norm(quaternion, axis=1)
    quaternion = quaternion / quaternion_norm[:, None]
    rotation_zz = 1.0 - 2.0 * (
        quaternion[:, 1] * quaternion[:, 1]
        + quaternion[:, 2] * quaternion[:, 2]
    )

    finite = (
        np.all(np.isfinite(motor_outputs), axis=1)
        & np.isfinite(vx)
        & np.isfinite(vy)
        & np.isfinite(vz)
        & np.isfinite(rotation_zz)
        & (quaternion_norm > 0.5)
    )
    level_hover = (
        finite
        & ~landed
        & xy_valid
        & z_valid
        & (np.hypot(vx, vy) <= maximum_horizontal_speed)
        & (np.abs(vz) <= maximum_vertical_speed)
        & (rotation_zz >= math.cos(math.radians(maximum_tilt_degrees)))
        & (thrust_indicator > 0.1)
    )
    sample_gain = GRAVITY_M_S2 / (
        thrust_indicator[level_hover] * rotation_zz[level_hover]
    )

    if sample_gain.size < minimum_samples:
        raise ValueError(
            f"{row['flight_id']}: only {sample_gain.size} stable hover samples; "
            f"need at least {minimum_samples}"
        )

    median = float(np.median(sample_gain))
    absolute_deviation = float(np.median(np.abs(sample_gain - median)))
    return {
        **row,
        "stable_sample_count": int(sample_gain.size),
        "gain_median": median,
        "gain_mad": absolute_deviation,
        "gain_minimum": float(np.min(sample_gain)),
        "gain_maximum": float(np.max(sample_gain)),
    }


def calibrate(rows, args):
    flights = [
        estimate_flight_gain(
            row,
            args.pwm_min,
            args.pwm_max,
            args.maximum_horizontal_speed,
            args.maximum_vertical_speed,
            args.maximum_tilt_degrees,
            args.minimum_samples_per_flight,
        )
        for row in rows
    ]
    per_flight = np.asarray([flight["gain_median"] for flight in flights])
    gain = float(np.median(per_flight))
    between_flight_mad = float(np.median(np.abs(per_flight - gain)))
    return gain, between_flight_mad, flights


def write_flight_csv(path, flights):
    fields = [
        "flight_id",
        "scenario",
        "ulog",
        "stable_sample_count",
        "gain_median",
        "gain_mad",
        "gain_minimum",
        "gain_maximum",
    ]

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(flights)


def main():
    args = parse_args()

    if not args.pwm_max > args.pwm_min:
        raise ValueError("--pwm-max must be greater than --pwm-min")

    if args.minimum_samples_per_flight < 1:
        raise ValueError("--minimum-samples-per-flight must be positive")

    rows = read_manifest(args.manifest)
    gain, between_flight_mad, flights = calibrate(rows, args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    flight_csv = args.output.with_name(f"{args.output.stem}_flights.csv")
    report = {
        "schema": "px4.sensor_attack_detector.actuator_gain_calibration.v1",
        "nominal_hover_only": True,
        "manifest": str(args.manifest.resolve()),
        "flight_count": len(flights),
        "candidate_parameter": {"SAD_THR_GAIN": gain},
        "between_flight_mad": between_flight_mad,
        "selection": {
            "maximum_horizontal_speed_m_s": args.maximum_horizontal_speed,
            "maximum_vertical_speed_m_s": args.maximum_vertical_speed,
            "maximum_tilt_degrees": args.maximum_tilt_degrees,
            "minimum_samples_per_flight": args.minimum_samples_per_flight,
            "pwm_min": args.pwm_min,
            "pwm_max": args.pwm_max,
        },
        "flights": [
            {**flight, "ulog": str(flight["ulog"])}
            for flight in flights
        ],
    }

    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")

    write_flight_csv(flight_csv, flights)
    print(json.dumps(report["candidate_parameter"], indent=2, sort_keys=True))
    print(f"report={args.output}")
    print(f"flights={flight_csv}")


if __name__ == "__main__":
    main()
