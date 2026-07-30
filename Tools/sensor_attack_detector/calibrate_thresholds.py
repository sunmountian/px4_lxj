#!/usr/bin/env python3

"""Calibrate PX4 sensor_attack_detector sequential thresholds from nominal ULogs.

This tool does not implement the detector or recompute its GLRT. It consumes
the raw GLRT values published by the PX4 module in sensor_attack_status and
only estimates fixed normalization and sequential-decision parameters.
"""

import argparse
import csv
import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required to calibrate detector thresholds") from exc


SPLITS = {"fit", "calibration", "test"}
FROZEN_PARAMETERS = (
    "SAD_ACT_SRC",
    "SAD_REG",
    "SAD_THR_GAIN",
    "SAD_WA",
    "SAD_WP",
    "SAD_WV",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate GLRT normalization and a flight-level CUSUM threshold."
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        required=True,
        help="CSV with flight_id, split, scenario, nominal, and ulog columns.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output JSON report path. A per-flight CSV is written beside it.",
    )
    parser.add_argument(
        "--false-alarm-rate",
        type=float,
        default=0.05,
        help="Target probability of at least one alert in a nominal flight.",
    )
    parser.add_argument(
        "--drift",
        type=float,
        default=0.5,
        help="Fixed one-sided CUSUM drift SAD_CUS_DR.",
    )
    parser.add_argument(
        "--consecutive",
        type=int,
        default=3,
        help="Required consecutive threshold crossings SAD_CONSEC.",
    )
    return parser.parse_args()


def read_manifest(path):
    rows = []

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"flight_id", "split", "scenario", "nominal", "ulog"}

        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            missing = sorted(required - set(reader.fieldnames or []))
            raise ValueError(f"manifest is missing columns: {missing}")

        for line_number, row in enumerate(reader, start=2):
            split = row["split"].strip().lower()
            nominal = row["nominal"].strip().lower()

            if split not in SPLITS:
                raise ValueError(f"line {line_number}: invalid split {split!r}")

            if nominal not in {"1", "true", "yes"}:
                raise ValueError(
                    f"line {line_number}: calibration accepts nominal flights only"
                )

            ulog = Path(row["ulog"]).expanduser()

            if not ulog.is_absolute():
                ulog = (path.parent / ulog).resolve()

            if not ulog.is_file():
                raise FileNotFoundError(f"line {line_number}: ULog not found: {ulog}")

            rows.append(
                {
                    "flight_id": row["flight_id"].strip(),
                    "split": split,
                    "scenario": row["scenario"].strip(),
                    "ulog": ulog,
                }
            )

    identifiers = [row["flight_id"] for row in rows]

    if len(set(identifiers)) != len(identifiers):
        raise ValueError("flight_id values must be unique")

    if not any(row["split"] == "fit" for row in rows):
        raise ValueError("manifest must contain at least one fit flight")

    return rows


def get_dataset(ulog, name):
    datasets = [dataset for dataset in ulog.data_list if dataset.name == name]

    if len(datasets) != 1:
        raise ValueError(
            f"{name}: expected exactly one instance, found {len(datasets)}"
        )

    return datasets[0].data


def load_flight(row):
    ulog = ULog(str(row["ulog"]), None, disable_str_exceptions=True)
    data = get_dataset(ulog, "sensor_attack_status")
    vehicle_status = get_dataset(ulog, "vehicle_status")
    required_fields = {
        "timestamp_sample",
        "valid",
        "glrt_score",
        "data_quality_flags",
        "imu_sample_count",
        "gps_sample_count",
    }
    missing = sorted(required_fields - set(data))

    if missing:
        raise ValueError(f"{row['ulog']}: missing status fields: {missing}")

    timestamp = np.asarray(data["timestamp_sample"], dtype=np.uint64)
    valid = np.asarray(data["valid"], dtype=bool)
    score = np.asarray(data["glrt_score"], dtype=float)
    quality = np.asarray(data["data_quality_flags"], dtype=np.uint32)

    if timestamp.size == 0:
        raise ValueError(f"{row['ulog']}: sensor_attack_status is empty")

    if np.any(np.diff(timestamp.astype(np.int64)) <= 0):
        raise ValueError(f"{row['ulog']}: status timestamps are not increasing")

    usable = valid & np.isfinite(score)

    if not np.any(usable):
        raise ValueError(f"{row['ulog']}: no valid finite GLRT samples")

    if "failsafe" not in vehicle_status:
        raise ValueError(f"{row['ulog']}: vehicle_status.failsafe is missing")

    failsafe_count = int(
        np.count_nonzero(np.asarray(vehicle_status["failsafe"], dtype=bool))
    )

    if failsafe_count:
        raise ValueError(
            f"{row['ulog']}: nominal calibration flight contains "
            f"{failsafe_count} failsafe samples"
        )

    parameters = {
        name: ulog.initial_parameters.get(name)
        for name in FROZEN_PARAMETERS
    }
    missing_parameters = sorted(
        name for name, value in parameters.items() if value is None
    )

    if missing_parameters:
        raise ValueError(
            f"{row['ulog']}: missing frozen parameters: {missing_parameters}"
        )

    return {
        **row,
        "timestamp": timestamp,
        "valid": valid,
        "score": score,
        "usable": usable,
        "quality": quality,
        "parameters": parameters,
        "duration_s": float(timestamp[-1] - timestamp[0]) * 1e-6,
        "valid_count": int(np.count_nonzero(usable)),
        "invalid_count": int(timestamp.size - np.count_nonzero(usable)),
        "quality_nonzero_count": int(np.count_nonzero(quality)),
        "failsafe_count": failsafe_count,
        "imu_sample_median": float(np.median(np.asarray(data["imu_sample_count"])[usable])),
        "gps_sample_median": float(np.median(np.asarray(data["gps_sample_count"])[usable])),
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


def sequential_statistics(flight, mean, standard_deviation, drift, consecutive):
    cusum = 0.0
    maximum_cusum = 0.0
    alarm_statistic = 0.0
    recent = deque(maxlen=consecutive)

    for valid, score in zip(flight["valid"], flight["score"]):
        if not valid or not math.isfinite(float(score)):
            cusum = 0.0
            recent.clear()
            continue

        normalized = (float(score) - mean) / standard_deviation
        cusum = max(0.0, cusum + normalized - drift)
        maximum_cusum = max(maximum_cusum, cusum)
        recent.append(cusum)

        if len(recent) == consecutive:
            alarm_statistic = max(alarm_statistic, min(recent))

    return maximum_cusum, alarm_statistic


def order_statistic_threshold(values, false_alarm_rate):
    ordered = sorted(float(value) for value in values)
    sample_count = len(ordered)
    rank = int(math.ceil((sample_count + 1) * (1.0 - false_alarm_rate)))
    supported = rank <= sample_count

    if supported:
        threshold = ordered[rank - 1]
    else:
        threshold = None

    pilot_threshold = float(np.nextafter(ordered[-1], math.inf))
    return {
        "finite_sample_supported": supported,
        "required_rank": rank,
        "calibration_flight_count": sample_count,
        "recommended_threshold": threshold,
        "pilot_threshold": pilot_threshold,
        "minimum_flights_for_target": int(math.ceil(1.0 / false_alarm_rate)) - 1,
    }


def write_flight_csv(path, summaries):
    fields = [
        "flight_id",
        "split",
        "scenario",
        "ulog",
        "duration_s",
        "valid_count",
        "invalid_count",
        "quality_nonzero_count",
        "failsafe_count",
        "imu_sample_median",
        "gps_sample_median",
        "score_mean",
        "score_standard_deviation",
        "score_maximum",
        "maximum_cusum",
        "alarm_statistic",
        "candidate_alert",
    ]

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)


def main():
    args = parse_args()

    if not 0.0 < args.false_alarm_rate < 1.0:
        raise ValueError("--false-alarm-rate must be between zero and one")

    if args.drift < 0.0:
        raise ValueError("--drift must be nonnegative")

    if args.consecutive < 1:
        raise ValueError("--consecutive must be positive")

    flights = [load_flight(row) for row in read_manifest(args.manifest)]
    frozen_parameters = verify_frozen_parameters(flights)
    fit_scores = np.concatenate(
        [
            flight["score"][flight["usable"]]
            for flight in flights
            if flight["split"] == "fit"
        ]
    )
    mean = float(np.mean(fit_scores))
    standard_deviation = float(np.std(fit_scores, ddof=1))

    if not math.isfinite(standard_deviation) or standard_deviation <= 1e-9:
        raise ValueError("fit GLRT standard deviation is zero or invalid")

    calibration_flights = [
        flight for flight in flights if flight["split"] == "calibration"
    ]

    if not calibration_flights:
        raise ValueError("manifest must contain at least one calibration flight")

    for flight in flights:
        maximum, alarm_statistic = sequential_statistics(
            flight, mean, standard_deviation, args.drift, args.consecutive
        )
        flight["maximum_cusum"] = maximum
        flight["alarm_statistic"] = alarm_statistic

    threshold_result = order_statistic_threshold(
        [flight["alarm_statistic"] for flight in calibration_flights],
        args.false_alarm_rate,
    )
    candidate_threshold = (
        threshold_result["recommended_threshold"]
        if threshold_result["recommended_threshold"] is not None
        else threshold_result["pilot_threshold"]
    )
    summaries = []

    for flight in flights:
        usable_scores = flight["score"][flight["usable"]]
        summaries.append(
            {
                "flight_id": flight["flight_id"],
                "split": flight["split"],
                "scenario": flight["scenario"],
                "ulog": str(flight["ulog"]),
                "duration_s": flight["duration_s"],
                "valid_count": flight["valid_count"],
                "invalid_count": flight["invalid_count"],
                "quality_nonzero_count": flight["quality_nonzero_count"],
                "failsafe_count": flight["failsafe_count"],
                "imu_sample_median": flight["imu_sample_median"],
                "gps_sample_median": flight["gps_sample_median"],
                "score_mean": float(np.mean(usable_scores)),
                "score_standard_deviation": float(np.std(usable_scores, ddof=1)),
                "score_maximum": float(np.max(usable_scores)),
                "maximum_cusum": flight["maximum_cusum"],
                "alarm_statistic": flight["alarm_statistic"],
                "candidate_alert": flight["alarm_statistic"] > candidate_threshold,
            }
        )

    report = {
        "schema": "px4.sensor_attack_detector.threshold_calibration.v1",
        "manifest": str(args.manifest.resolve()),
        "nominal_only": True,
        "detector_source": "PX4 sensor_attack_status.glrt_score",
        "flight_count": len(flights),
        "split_counts": {
            split: sum(flight["split"] == split for flight in flights)
            for split in sorted(SPLITS)
        },
        "frozen_parameters": frozen_parameters,
        "fit": {
            "glrt_sample_count": int(fit_scores.size),
            "SAD_GLRT_MU": mean,
            "SAD_GLRT_SD": standard_deviation,
            "SAD_CUS_DR": args.drift,
            "SAD_CONSEC": args.consecutive,
        },
        "flight_level_threshold": {
            "target_false_alarm_rate": args.false_alarm_rate,
            **threshold_result,
        },
        "candidate_parameters": {
            "SAD_GLRT_MU": mean,
            "SAD_GLRT_SD": standard_deviation,
            "SAD_CUS_DR": args.drift,
            "SAD_THRESH": candidate_threshold,
            "SAD_CONSEC": args.consecutive,
            "status": (
                "calibrated"
                if threshold_result["finite_sample_supported"]
                else "pilot_only_insufficient_calibration_flights"
            ),
        },
        "flights": summaries,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    flight_csv = args.output.with_name(f"{args.output.stem}_flights.csv")
    write_flight_csv(flight_csv, summaries)

    print(json.dumps(report["candidate_parameters"], indent=2, sort_keys=True))
    print(f"report={args.output}")
    print(f"flights={flight_csv}")

    if not threshold_result["finite_sample_supported"]:
        print(
            "warning: calibration set is too small for the requested "
            "finite-sample flight-level false-alarm target",
            file=sys.stderr,
        )


if __name__ == "__main__":
    sys.exit(main())
