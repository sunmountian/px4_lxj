#!/usr/bin/env python3

"""Collect a predeclared batch of independent nominal SITL flights."""

import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit("pyulog is required to validate collected ULogs") from exc


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNERS = {
    "hover": SCRIPT_DIR / "run_nominal_hover.py",
    "maneuver": SCRIPT_DIR / "run_nominal_maneuver.py",
}
RESULT_FIELDS = (
    "flight_id",
    "split",
    "scenario",
    "status",
    "runner",
    "vn",
    "ve",
    "duration_s",
    "run_log",
    "ulog",
    "failsafe_count",
    "valid_score_count",
    "glrt_maximum",
    "message",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect and validate a predeclared nominal SITL batch."
    )
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--thr-gain", type=float, required=True)
    parser.add_argument("--glrt-mean", type=float, default=0.0)
    parser.add_argument("--glrt-standard-deviation", type=float, default=1.0)
    parser.add_argument("--cusum-drift", type=float, default=0.5)
    parser.add_argument("--threshold", type=float, default=10000.0)
    parser.add_argument("--consecutive", type=int, default=3)
    parser.add_argument(
        "--expected-split",
        choices=("calibration", "test"),
        default="calibration",
        help="Required split for every scheduled flight.",
    )
    parser.add_argument("--limit", type=int)
    return parser.parse_args()


def read_schedule(path, expected_split):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    required = {
        "flight_id",
        "split",
        "scenario",
        "runner",
        "vn",
        "ve",
        "duration_s",
    }
    fields = set(rows[0].keys()) if rows else set()

    if not rows or not required.issubset(fields):
        raise ValueError(
            f"schedule is empty or missing columns: {sorted(required - fields)}"
        )

    identifiers = [row["flight_id"].strip() for row in rows]

    if len(set(identifiers)) != len(identifiers):
        raise ValueError("schedule flight_id values must be unique")

    for row in rows:
        row["flight_id"] = row["flight_id"].strip()
        row["split"] = row["split"].strip().lower()
        row["scenario"] = row["scenario"].strip()
        row["runner"] = row["runner"].strip().lower()

        if row["split"] != expected_split:
            raise ValueError(
                f"{row['flight_id']}: expected split {expected_split!r}, "
                f"found {row['split']!r}"
            )

        if row["runner"] not in RUNNERS:
            raise ValueError(
                f"{row['flight_id']}: unsupported runner {row['runner']!r}"
            )

        row["vn"] = float(row["vn"] or 0.0)
        row["ve"] = float(row["ve"] or 0.0)
        row["duration_s"] = float(row["duration_s"])

        if row["duration_s"] <= 0.0:
            raise ValueError(f"{row['flight_id']}: duration_s must be positive")

    return rows


def read_inventory(path):
    if not path.is_file():
        return []

    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_inventory(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def parse_output_path(stdout, prefix):
    matches = [
        line[len(prefix) :].strip()
        for line in stdout.splitlines()
        if line.startswith(prefix)
    ]

    if len(matches) != 1:
        raise ValueError(f"runner did not report exactly one {prefix.rstrip('=')}")

    return Path(matches[0])


def get_dataset(ulog, name):
    datasets = [dataset for dataset in ulog.data_list if dataset.name == name]

    if len(datasets) != 1:
        raise ValueError(
            f"{name}: expected exactly one instance, found {len(datasets)}"
        )

    return datasets[0].data


def get_effective_parameter(ulog, name):
    value = ulog.initial_parameters.get(name)

    for _, changed_name, changed_value in ulog.changed_parameters:
        if changed_name == name:
            value = changed_value

    return value


def validate_ulog(path, expected_parameters):
    ulog = ULog(str(path), None, disable_str_exceptions=True)
    status = get_dataset(ulog, "sensor_attack_status")
    vehicle_status = get_dataset(ulog, "vehicle_status")
    failsafe_count = int(
        np.count_nonzero(
            np.asarray(vehicle_status["failsafe"], dtype=bool)
        )
    )

    if failsafe_count:
        raise ValueError(f"ULog contains {failsafe_count} failsafe samples")

    for name, expected in expected_parameters.items():
        actual = get_effective_parameter(ulog, name)

        if actual is None or not np.isclose(
            float(actual), float(expected), rtol=1e-6, atol=1e-6
        ):
            raise ValueError(f"{name}={actual!r}, expected {expected}")

    valid = np.asarray(status["valid"], dtype=bool)
    score = np.asarray(status["glrt_score"], dtype=float)
    usable = valid & np.isfinite(score)

    if not np.any(usable):
        raise ValueError("ULog contains no valid finite GLRT samples")

    return {
        "failsafe_count": failsafe_count,
        "valid_score_count": int(np.count_nonzero(usable)),
        "glrt_maximum": float(np.max(score[usable])),
    }


def run_flight(row, args):
    command = [
        sys.executable,
        str(RUNNERS[row["runner"]]),
        "--mode",
        "baseline",
        "--takeoff-altitude",
        "3",
        "--attack-duration",
        str(row["duration_s"]),
    ]

    if row["runner"] == "maneuver":
        command.extend(
            [
                "--vn",
                str(row["vn"]),
                "--ve",
                str(row["ve"]),
            ]
        )

    environment = os.environ.copy()
    environment.update(
        {
            "SAD_CAL_COM_RCL_EXCEPT": "7",
            "SAD_CAL_COM_RC_IN_MODE": "4",
            "SAD_CAL_COM_OF_LOSS_T": "2.0",
            "SAD_CAL_THR_GAIN": str(args.thr_gain),
            "SAD_CAL_GLRT_MU": str(args.glrt_mean),
            "SAD_CAL_GLRT_SD": str(args.glrt_standard_deviation),
            "SAD_CAL_CUS_DR": str(args.cusum_drift),
            "SAD_CAL_THRESH": str(args.threshold),
            "SAD_CAL_CONSEC": str(args.consecutive),
        }
    )
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
        timeout=max(240.0, row["duration_s"] + 180.0),
    )
    combined_output = completed.stdout + completed.stderr

    if completed.returncode != 0:
        raise RuntimeError(
            f"runner exited with {completed.returncode}: "
            f"{combined_output[-2000:]}"
        )

    run_log = parse_output_path(completed.stdout, "run_log=")
    ulog = parse_output_path(completed.stdout, "ulog=")

    if not run_log.is_file() or not ulog.is_file():
        raise FileNotFoundError("runner output paths do not exist")

    run_log_text = run_log.read_text(encoding="utf-8", errors="replace")

    if "failsafe" in run_log_text.lower():
        raise ValueError("PX4 console log contains a failsafe event")

    metrics = validate_ulog(
        ulog,
        {
            "SAD_THR_GAIN": args.thr_gain,
            "SAD_GLRT_MU": args.glrt_mean,
            "SAD_GLRT_SD": args.glrt_standard_deviation,
            "SAD_CUS_DR": args.cusum_drift,
            "SAD_THRESH": args.threshold,
            "SAD_CONSEC": args.consecutive,
        },
    )
    return run_log, ulog, metrics


def main():
    args = parse_args()
    schedule = read_schedule(args.schedule, args.expected_split)
    inventory = read_inventory(args.inventory)
    completed_ids = {
        row["flight_id"]
        for row in inventory
        if row.get("status") == "accepted"
    }
    pending = [row for row in schedule if row["flight_id"] not in completed_ids]

    if args.limit is not None:
        pending = pending[: args.limit]

    print(
        f"schedule={len(schedule)} accepted={len(completed_ids)} "
        f"pending_this_run={len(pending)}",
        flush=True,
    )

    for index, row in enumerate(pending, start=1):
        print(
            f"[{index}/{len(pending)}] start {row['flight_id']} "
            f"{row['scenario']}",
            flush=True,
        )
        result = {
            "flight_id": row["flight_id"],
            "split": row["split"],
            "scenario": row["scenario"],
            "status": "rejected",
            "runner": row["runner"],
            "vn": row["vn"],
            "ve": row["ve"],
            "duration_s": row["duration_s"],
            "run_log": "",
            "ulog": "",
            "failsafe_count": "",
            "valid_score_count": "",
            "glrt_maximum": "",
            "message": "",
        }

        try:
            run_log, ulog, metrics = run_flight(row, args)
            result.update(
                {
                    "status": "accepted",
                    "run_log": str(run_log),
                    "ulog": str(ulog),
                    **metrics,
                }
            )
            print(
                f"[{index}/{len(pending)}] accepted {row['flight_id']} "
                f"glrt_max={metrics['glrt_maximum']:.6f}",
                flush=True,
            )

        except Exception as exc:
            result["message"] = str(exc)
            print(
                f"[{index}/{len(pending)}] rejected {row['flight_id']}: {exc}",
                flush=True,
            )

        inventory.append(result)
        write_inventory(args.inventory, inventory)

    accepted = sum(row.get("status") == "accepted" for row in inventory)
    print(f"complete accepted_total={accepted}", flush=True)


if __name__ == "__main__":
    main()
