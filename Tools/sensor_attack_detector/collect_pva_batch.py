#!/usr/bin/env python3

"""Run and evaluate a predeclared batch of steady-hover PVA attacks."""

import argparse
import csv
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog

from collect_nominal_batch import get_dataset, get_effective_parameter


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER = SCRIPT_DIR / "run_nominal_hover.py"
FIELDS = (
    "flight_id",
    "scenario",
    "status",
    "north_m",
    "east_m",
    "ramp_s",
    "duration_s",
    "run_log",
    "ulog",
    "valid_score_count",
    "glrt_maximum",
    "cusum_maximum",
    "pre_attack_alert_samples",
    "post_attack_alert_samples",
    "detected",
    "detection_delay_s",
    "attack_offset_at_alert_m",
    "groundtruth_displacement_at_alert_m",
    "direction_cosine",
    "message",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect and evaluate frozen-parameter PVA attack flights."
    )
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--thr-gain", type=float, required=True)
    parser.add_argument("--glrt-mean", type=float, required=True)
    parser.add_argument("--glrt-standard-deviation", type=float, required=True)
    parser.add_argument("--cusum-drift", type=float, required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--consecutive", type=int, required=True)
    return parser.parse_args()


def read_csv(path):
    if not path.is_file():
        return []

    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def parse_output_value(stdout, prefix):
    matches = [
        line[len(prefix) :].strip()
        for line in stdout.splitlines()
        if line.startswith(prefix)
    ]

    if len(matches) != 1:
        raise ValueError(f"runner did not report exactly one {prefix}")

    return matches[0]


def smoother_step(value):
    phase = float(np.clip(value, 0.0, 1.0))
    return 6.0 * phase**5 - 15.0 * phase**4 + 10.0 * phase**3


def trigger_boot_timestamp(ulog, trigger_wall_s):
    gps = get_dataset(ulog, "vehicle_gps_position")
    timestamp = np.asarray(gps["timestamp"], dtype=np.float64)
    utc = np.asarray(gps["time_utc_usec"], dtype=np.float64)
    usable = np.isfinite(timestamp) & np.isfinite(utc) & (utc > 0.0)

    if not np.any(usable):
        raise ValueError("vehicle_gps_position has no usable UTC timestamps")

    boot_minus_utc = float(np.median(timestamp[usable] - utc[usable]))
    return trigger_wall_s * 1e6 + boot_minus_utc


def groundtruth_displacement(ulog, start_us, end_us):
    groundtruth = get_dataset(ulog, "vehicle_local_position_groundtruth")
    timestamp = np.asarray(groundtruth["timestamp"], dtype=np.float64)
    north = np.asarray(groundtruth["x"], dtype=np.float64)
    east = np.asarray(groundtruth["y"], dtype=np.float64)
    start_index = int(np.argmin(np.abs(timestamp - start_us)))
    end_index = int(np.argmin(np.abs(timestamp - end_us)))
    return float(
        math.hypot(
            north[end_index] - north[start_index],
            east[end_index] - east[start_index],
        )
    )


def validate_and_summarize(
    ulog_path,
    trigger_wall_s,
    row,
    expected_parameters,
):
    ulog = ULog(str(ulog_path), None, disable_str_exceptions=True)
    vehicle_status = get_dataset(ulog, "vehicle_status")
    failsafe = np.asarray(vehicle_status["failsafe"], dtype=bool)

    if np.any(failsafe):
        raise ValueError(
            f"ULog contains {int(np.count_nonzero(failsafe))} failsafe samples"
        )

    for name, expected in expected_parameters.items():
        actual = get_effective_parameter(ulog, name)

        if actual is None or not np.isclose(
            float(actual), float(expected), rtol=1e-6, atol=1e-6
        ):
            raise ValueError(f"{name}={actual!r}, expected {expected}")

    status = get_dataset(ulog, "sensor_attack_status")
    timestamp = np.asarray(status["timestamp"], dtype=np.float64)
    valid = np.asarray(status["valid"], dtype=bool)
    glrt = np.asarray(status["glrt_score"], dtype=np.float64)
    cusum = np.asarray(status["cusum_score"], dtype=np.float64)
    alerts = np.asarray(status["attack_detected"], dtype=bool)
    usable = valid & np.isfinite(glrt) & np.isfinite(cusum)

    if not np.any(usable):
        raise ValueError("ULog contains no valid finite detector samples")

    trigger_us = trigger_boot_timestamp(ulog, trigger_wall_s)
    pre_attack_alerts = int(np.count_nonzero(alerts & (timestamp < trigger_us)))
    post_indices = np.flatnonzero(alerts & usable & (timestamp >= trigger_us))
    detected = len(post_indices) > 0
    delay_s = math.nan
    attack_offset_m = math.nan
    displacement_m = math.nan
    direction_cosine = math.nan

    if detected:
        first_index = int(post_indices[0])
        first_alert_us = float(timestamp[first_index])
        delay_s = max(0.0, (first_alert_us - trigger_us) * 1e-6)
        target_norm = math.hypot(row["north_m"], row["east_m"])
        attack_offset_m = target_norm * smoother_step(delay_s / row["ramp_s"])
        displacement_m = groundtruth_displacement(
            ulog, trigger_us, first_alert_us
        )
        estimate_n = float(status["direction_n"][first_index])
        estimate_e = float(status["direction_e"][first_index])
        estimate_norm = math.hypot(estimate_n, estimate_e)

        if target_norm > 0.0 and estimate_norm > 0.0:
            direction_cosine = (
                estimate_n * row["north_m"]
                + estimate_e * row["east_m"]
            ) / (estimate_norm * target_norm)

    return {
        "valid_score_count": int(np.count_nonzero(usable)),
        "glrt_maximum": float(np.max(glrt[usable])),
        "cusum_maximum": float(np.max(cusum[usable])),
        "pre_attack_alert_samples": pre_attack_alerts,
        "post_attack_alert_samples": int(
            np.count_nonzero(alerts & (timestamp >= trigger_us))
        ),
        "detected": str(detected).lower(),
        "detection_delay_s": "" if not detected else delay_s,
        "attack_offset_at_alert_m": "" if not detected else attack_offset_m,
        "groundtruth_displacement_at_alert_m": (
            "" if not detected else displacement_m
        ),
        "direction_cosine": "" if not detected else direction_cosine,
    }


def main():
    args = parse_args()
    schedule = read_csv(args.schedule)
    inventory = read_csv(args.inventory)
    completed = {
        row["flight_id"]
        for row in inventory
        if row.get("status") == "accepted"
    }

    for row in schedule:
        row["north_m"] = float(row["north_m"])
        row["east_m"] = float(row["east_m"])
        row["ramp_s"] = float(row["ramp_s"])
        row["duration_s"] = float(row["duration_s"])

    pending = [row for row in schedule if row["flight_id"] not in completed]
    print(
        f"schedule={len(schedule)} accepted={len(completed)} "
        f"pending_this_run={len(pending)}",
        flush=True,
    )

    expected_parameters = {
        "SAD_THR_GAIN": args.thr_gain,
        "SAD_GLRT_MU": args.glrt_mean,
        "SAD_GLRT_SD": args.glrt_standard_deviation,
        "SAD_CUS_DR": args.cusum_drift,
        "SAD_THRESH": args.threshold,
        "SAD_CONSEC": args.consecutive,
    }

    for index, row in enumerate(pending, start=1):
        print(
            f"[{index}/{len(pending)}] start {row['flight_id']} "
            f"{row['scenario']}",
            flush=True,
        )
        result = {
            field: ""
            for field in FIELDS
        }
        result.update(
            {
                "flight_id": row["flight_id"],
                "scenario": row["scenario"],
                "status": "rejected",
                "north_m": row["north_m"],
                "east_m": row["east_m"],
                "ramp_s": row["ramp_s"],
                "duration_s": row["duration_s"],
            }
        )
        command = [
            sys.executable,
            str(RUNNER),
            "--mode",
            "gps-accel",
            "--north",
            str(row["north_m"]),
            "--east",
            str(row["east_m"]),
            "--ramp",
            str(row["ramp_s"]),
            "--takeoff-altitude",
            "3",
            "--attack-duration",
            str(row["duration_s"]),
        ]
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

        try:
            completed_process = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                env=environment,
                timeout=max(300.0, row["duration_s"] + 180.0),
            )
            combined_output = (
                completed_process.stdout + completed_process.stderr
            )

            if completed_process.returncode != 0:
                raise RuntimeError(
                    f"runner exited with {completed_process.returncode}: "
                    f"{combined_output[-2000:]}"
                )

            run_log = Path(
                parse_output_value(completed_process.stdout, "run_log=")
            )
            ulog = Path(parse_output_value(completed_process.stdout, "ulog="))
            trigger_wall_s = float(
                parse_output_value(completed_process.stdout, "trigger_time=")
            )
            log_text = run_log.read_text(encoding="utf-8", errors="replace")

            if "steady replacement sensor injection triggered" not in log_text:
                raise ValueError("attack trigger confirmation missing")

            if "failsafe" in log_text.lower():
                raise ValueError("PX4 console log contains a failsafe event")

            metrics = validate_and_summarize(
                ulog,
                trigger_wall_s,
                row,
                expected_parameters,
            )
            result.update(metrics)
            result["run_log"] = str(run_log)
            result["ulog"] = str(ulog)
            result["status"] = "accepted"
            print(
                f"[{index}/{len(pending)}] accepted {row['flight_id']} "
                f"detected={metrics['detected']} "
                f"delay={metrics['detection_delay_s']}",
                flush=True,
            )

        except Exception as exc:
            result["message"] = str(exc)
            print(
                f"[{index}/{len(pending)}] rejected "
                f"{row['flight_id']}: {exc}",
                flush=True,
            )

        inventory = [
            previous
            for previous in inventory
            if previous["flight_id"] != row["flight_id"]
        ]
        inventory.append(result)
        write_csv(args.inventory, inventory)

    accepted_total = sum(
        row.get("status") == "accepted" for row in inventory
    )
    print(f"complete accepted_total={accepted_total}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
