#!/usr/bin/env python3

"""Self-contained PX4 SITL smoke runner for the V2 sensor attack detector.

This runner intentionally avoids the legacy calibration monkey-patches. It
freezes the complete detector parameter set through the PX4 shell, restarts
the detector so its eight-second window begins after parameter application,
then flies either a nominal smooth 90-degree turn or the retained 100 m / 20 s
coordinated PVA hover attack.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil
from pyulog import ULog


PX4_ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = PX4_ROOT / "build" / "sad_v2_smoke"
ULOG_ROOT = PX4_ROOT / "build" / "px4_sitl_default" / "tmp" / "rootfs" / "log"

VELOCITY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("turn", "maneuver", "pva-hover"), required=True)
    parser.add_argument("--maneuver", choices=("accel", "slalom", "hard-turn"), default="slalom")
    parser.add_argument("--speed", type=float, default=1.3)
    parser.add_argument("--turn-duration", type=float, default=6.0)
    parser.add_argument("--straight-before", type=float, default=12.0)
    parser.add_argument("--straight-after", type=float, default=20.0)
    parser.add_argument("--takeoff-altitude", type=float, default=3.0)
    parser.add_argument("--setpoint-rate", type=float, default=20.0)
    parser.add_argument("--url", default="udpin:0.0.0.0:14540")
    parser.add_argument("--thr-gain", type=float, default=3.47118998)
    parser.add_argument("--map-mode", type=int, choices=(0, 1), default=0)
    parser.add_argument("--map-idle", type=float, default=0.0)
    parser.add_argument("--weight-acceleration", type=float, default=1.0)
    parser.add_argument("--weight-velocity", type=float, default=1.0)
    parser.add_argument("--weight-position", type=float, default=1.0)
    parser.add_argument("--regularization", type=float, default=0.1)
    parser.add_argument("--normal-scale-mean", type=float, default=0.0)
    parser.add_argument("--normal-scale-sd", type=float, default=100.0)
    parser.add_argument("--normal-dynamic-mean", type=float, default=0.0)
    parser.add_argument("--normal-dynamic-sd", type=float, default=100.0)
    parser.add_argument("--pva-north", type=float, default=100.0)
    parser.add_argument("--pva-east", type=float, default=0.0)
    parser.add_argument("--pva-ramp", type=float, default=20.0)
    parser.add_argument("--pva-profile", choices=("smootherstep", "sinusoidal"), default="smootherstep")
    parser.add_argument("--pva-observation", type=float, default=8.0)
    parser.add_argument("--glrt-mean", type=float, default=0.0)
    parser.add_argument("--glrt-sd", type=float, default=1.0)
    parser.add_argument("--cusum-drift", type=float, default=0.5)
    parser.add_argument("--threshold", type=float, default=1000000.0)
    parser.add_argument("--consecutive", type=int, default=3)
    return parser.parse_args()


def wait_for_log_text(path, needle, timeout_s):
    deadline = time.monotonic() + timeout_s
    offset = 0

    while time.monotonic() < deadline:
        if path.is_file():
            with path.open("r", encoding="utf-8", errors="replace") as stream:
                stream.seek(offset)
                text = stream.read()
                offset = stream.tell()

            if needle in text:
                return

        time.sleep(0.2)

    raise TimeoutError(f"did not observe {needle!r} in {path}")


def terminate_process(process):
    if process.poll() is not None:
        return

    for sig, timeout_s in ((signal.SIGINT, 15), (signal.SIGTERM, 10)):
        os.killpg(process.pid, sig)

        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            pass

    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=10)


def shell(process, command, settle_s=0.08):
    process.stdin.write(command + "\n")
    process.stdin.flush()
    time.sleep(settle_s)


def latest_ulog_after(start_time):
    if not ULOG_ROOT.exists():
        return None

    candidates = [
        path
        for path in ULOG_ROOT.rglob("*.ulg")
        if path.stat().st_mtime >= start_time
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def command_long(master, command, params, timeout_s=5.0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *params,
    )
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)

        if ack is not None and ack.command == command:
            return ack

    return None


def heartbeat_matches_mode(heartbeat, mode):
    _, expected_main_mode, expected_sub_mode = mavutil.px4_map[mode]
    custom_mode = int(heartbeat.custom_mode)
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    return (
        main_mode == expected_main_mode
        and (expected_sub_mode == 0 or sub_mode == expected_sub_mode)
    )


def set_px4_mode(master, mode, timeout_s=20.0, keepalive=None):
    mode_flag, main_mode, sub_mode = mavutil.px4_map[mode]
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        if keepalive is not None:
            keepalive()

        ack = command_long(
            master,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [mode_flag, main_mode, sub_mode, 0, 0, 0, 0],
            timeout_s=2.0,
        )

        if ack is not None and ack.result in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            break

        time.sleep(0.1)
    else:
        raise TimeoutError(f"{mode} command was not accepted")

    while time.monotonic() < deadline:
        if keepalive is not None:
            keepalive()

        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)

        if (
            heartbeat is not None
            and heartbeat.get_srcSystem() == master.target_system
            and heartbeat_matches_mode(heartbeat, mode)
        ):
            return

    raise TimeoutError(f"vehicle did not enter {mode}")


def wait_armed(master, timeout_s=20.0):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)

        if heartbeat is None or heartbeat.get_srcSystem() != master.target_system:
            continue

        if int(heartbeat.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return

    raise TimeoutError("vehicle did not arm")


def force_arm(master):
    ack = command_long(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [1.0, 21196.0, 0, 0, 0, 0, 0],
        timeout_s=10.0,
    )

    if ack is None:
        raise TimeoutError("force-arm command was not acknowledged")

    if ack.result not in (
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    ):
        raise RuntimeError(f"force-arm command rejected with result {ack.result}")

    wait_armed(master)


def request_local_position(master, rate_hz=20.0):
    command_long(
        master,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        [
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            int(1_000_000 / rate_hz),
            0,
            0,
            0,
            0,
            0,
        ],
    )


def send_velocity_setpoint(master, vn, ve, vd=0.0):
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_TYPE_MASK,
        0,
        0,
        0,
        vn,
        ve,
        vd,
        0,
        0,
        0,
        0,
        0,
    )


def takeoff_offboard(master, altitude_m, rate_hz=20.0, timeout_s=45.0):
    # PX4 pre-arm checks require us to leave the default Manual main state
    # when no RC/manual-control stream exists. Prime Offboard setpoints first,
    # switch to Offboard, then arm and climb with a NED vertical velocity.
    for _ in range(int(rate_hz)):
        send_velocity_setpoint(master, 0.0, 0.0, 0.0)
        master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.01)
        time.sleep(1.0 / rate_hz)

    set_px4_mode(
        master,
        "OFFBOARD",
        keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0, 0.0),
    )
    force_arm(master)

    deadline = time.monotonic() + timeout_s
    reached_altitude = False

    while time.monotonic() < deadline:
        send_velocity_setpoint(
            master,
            0.0,
            0.0,
            0.0 if reached_altitude else -0.8,
        )
        message = master.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=0.05
        )

        if message is not None and float(message.z) <= -(altitude_m - 0.2):
            reached_altitude = True

        if reached_altitude:
            return

        time.sleep(max(0.0, 1.0 / rate_hz - 0.01))

    raise TimeoutError("offboard takeoff altitude not reached")


def stream_velocity(master, velocity_function, duration_s, rate_hz):
    start = time.monotonic()
    next_send = start

    while time.monotonic() - start < duration_s:
        now = time.monotonic()

        if now >= next_send:
            elapsed = now - start
            vn, ve = velocity_function(elapsed)
            send_velocity_setpoint(master, vn, ve, 0.0)
            next_send += 1.0 / rate_hz

        master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.01)


def wait_stable_hover(
    master,
    altitude_m,
    timeout_s=90.0,
    rate_hz=20.0,
    keepalive=None,
):
    stable_since = None
    deadline = time.monotonic() + timeout_s
    next_send = time.monotonic()

    while time.monotonic() < deadline:
        now = time.monotonic()

        if keepalive is not None and now >= next_send:
            keepalive()
            next_send += 1.0 / rate_hz

        message = master.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=0.05
        )

        if message is None:
            continue

        stable = (
            abs(float(message.z) + altitude_m) < 0.5
            and math.hypot(float(message.vx), float(message.vy)) < 0.2
            and abs(float(message.vz)) < 0.15
        )

        if stable:
            if stable_since is None:
                stable_since = time.monotonic()

            if time.monotonic() - stable_since >= 3.0:
                return
        else:
            stable_since = None

    raise TimeoutError("stable hover not reached")


def smootherstep(x):
    x = min(1.0, max(0.0, x))
    return 6.0 * x**5 - 15.0 * x**4 + 10.0 * x**3


def apply_detector_parameters(process, args):
    parameters = {
        "COM_RCL_EXCEPT": 7,
        "COM_RC_IN_MODE": 4,
        "COM_OF_LOSS_T": 2.0,
        "MIS_TAKEOFF_ALT": args.takeoff_altitude,
        "SAD_THR_GAIN": args.thr_gain,
        "SAD_MAP_MODE": args.map_mode,
        "SAD_MAP_IDLE": args.map_idle,
        "SAD_WA": args.weight_acceleration,
        "SAD_WV": args.weight_velocity,
        "SAD_WP": args.weight_position,
        "SAD_REG": args.regularization,
        "SAD_AS_MU": args.normal_scale_mean,
        "SAD_AS_SD": args.normal_scale_sd,
        "SAD_AD_MU": args.normal_dynamic_mean,
        "SAD_AD_SD": args.normal_dynamic_sd,
        "SAD_GLRT_MU": args.glrt_mean,
        "SAD_GLRT_SD": args.glrt_sd,
        "SAD_CUS_DR": args.cusum_drift,
        "SAD_THRESH": args.threshold,
        "SAD_CONSEC": args.consecutive,
    }
    shell(process, "sensor_attack_detector stop", 0.2)

    for name, value in parameters.items():
        shell(process, f"param set {name} {value}")

    shell(process, "sensor_attack_detector start", 0.3)
    return parameters


def get_dataset(ulog, name):
    datasets = [item for item in ulog.data_list if item.name == name]

    if len(datasets) != 1:
        raise ValueError(f"{name}: expected one instance, found {len(datasets)}")

    return datasets[0].data


def summarize_detector(ulog_path, scenario):
    ulog = ULog(str(ulog_path), None, disable_str_exceptions=True)
    status = get_dataset(ulog, "sensor_attack_status")
    valid = np.asarray(status["valid"], dtype=bool)
    quality = np.asarray(status["data_quality_flags"], dtype=np.uint32)
    glrt = np.asarray(status["glrt_score"], dtype=float)
    cost_null = (
        np.asarray(status["cost_null_n"], dtype=float)
        + np.asarray(status["cost_null_e"], dtype=float)
    )
    cost_attack = (
        np.asarray(status["cost_attack_n"], dtype=float)
        + np.asarray(status["cost_attack_e"], dtype=float)
    )
    usable = valid & np.isfinite(glrt) & np.isfinite(cost_null) & np.isfinite(cost_attack)
    clean = usable & (quality == 0)

    result = {
        "status_samples": int(valid.size),
        "valid_samples": int(np.count_nonzero(usable)),
        "clean_samples": int(np.count_nonzero(clean)),
    }

    if scenario == "pva-hover":
        marker_timestamp = None

        for timestamp, name, value in ulog.changed_parameters:
            if name == "SAD_THRESH":
                marker_timestamp = int(timestamp)

        if marker_timestamp is not None:
            sample_timestamp = np.asarray(status["timestamp_sample"], dtype=np.uint64)
            pre_attack = usable & (sample_timestamp < marker_timestamp)
            post_attack = usable & (sample_timestamp >= marker_timestamp)
            result["attack_marker_timestamp"] = marker_timestamp
            detected = np.asarray(status["attack_detected"], dtype=bool)
            pre_alert = usable & (sample_timestamp < marker_timestamp) & detected
            post_alert = usable & (sample_timestamp >= marker_timestamp) & detected
            result["pre_attack_alert_count"] = int(np.count_nonzero(pre_alert))
            result["post_attack_alert_count"] = int(np.count_nonzero(post_alert))

            if np.any(post_alert):
                first_alert_timestamp = int(sample_timestamp[np.flatnonzero(post_alert)[0]])
                result["first_alert_timestamp"] = first_alert_timestamp
                result["first_alert_delay_s"] = (
                    first_alert_timestamp - marker_timestamp
                ) * 1e-6

            if np.any(pre_attack):
                result.update(
                    {
                        "pre_attack_glrt_mean": float(np.mean(glrt[pre_attack])),
                        "pre_attack_cost_null_mean": float(np.mean(cost_null[pre_attack])),
                        "pre_attack_cost_attack_mean": float(np.mean(cost_attack[pre_attack])),
                        "pre_attack_glrt_sd": float(np.std(glrt[pre_attack], ddof=1))
                        if np.count_nonzero(pre_attack) > 1
                        else 0.0,
                        "pre_attack_glrt_max": float(np.max(glrt[pre_attack])),
                    }
                )

            if np.any(post_attack):
                result.update(
                    {
                        "post_attack_glrt_mean": float(np.mean(glrt[post_attack])),
                        "post_attack_glrt_max": float(np.max(glrt[post_attack])),
                        "post_attack_cost_null_mean": float(np.mean(cost_null[post_attack])),
                        "post_attack_cost_attack_mean": float(np.mean(cost_attack[post_attack])),
                        "post_attack_cost_reduction_fraction": float(
                            np.mean(
                                (cost_null[post_attack] - cost_attack[post_attack])
                                / np.maximum(cost_null[post_attack], 1e-9)
                            )
                        ),
                        "post_attack_sample_count": int(np.count_nonzero(post_attack)),
                    }
                )

    detected = np.asarray(status["attack_detected"], dtype=bool)
    result["alert_samples"] = int(np.count_nonzero(usable & detected))

    if "cusum_score" in status and np.any(usable):
        cusum = np.asarray(status["cusum_score"], dtype=float)
        threshold_values = np.asarray(status["threshold"], dtype=float)
        finite_cusum = usable & np.isfinite(cusum)
        finite_threshold = usable & np.isfinite(threshold_values)

        if np.any(finite_cusum):
            result["cusum_max"] = float(np.max(cusum[finite_cusum]))

        if np.any(finite_threshold):
            result["threshold_value"] = float(np.median(threshold_values[finite_threshold]))

            if "cusum_max" in result:
                result["cusum_margin"] = result["threshold_value"] - result["cusum_max"]

    if np.any(usable):
        result.update(
            {
                "glrt_mean": float(np.mean(glrt[usable])),
                "glrt_sd": float(np.std(glrt[usable], ddof=1)),
                "glrt_p95": float(np.percentile(glrt[usable], 95)),
                "glrt_max": float(np.max(glrt[usable])),
                "normal_scale_n_mean": float(
                    np.mean(np.asarray(status["normal_scale_n"], dtype=float)[usable])
                ),
                "normal_scale_e_mean": float(
                    np.mean(np.asarray(status["normal_scale_e"], dtype=float)[usable])
                ),
                "normal_dynamic_n_mean": float(
                    np.mean(np.asarray(status["normal_dynamic_n"], dtype=float)[usable])
                ),
                "normal_dynamic_e_mean": float(
                    np.mean(np.asarray(status["normal_dynamic_e"], dtype=float)[usable])
                ),
                "normal_scale_all_sd": float(
                    np.std(
                        np.concatenate(
                            [
                                np.asarray(status["normal_scale_n"], dtype=float)[usable],
                                np.asarray(status["normal_scale_e"], dtype=float)[usable],
                            ]
                        ),
                        ddof=1,
                    )
                ),
                "normal_dynamic_all_sd": float(
                    np.std(
                        np.concatenate(
                            [
                                np.asarray(status["normal_dynamic_n"], dtype=float)[usable],
                                np.asarray(status["normal_dynamic_e"], dtype=float)[usable],
                            ]
                        ),
                        ddof=1,
                    )
                ),
            }
        )

    return result


def run_turn(master, args):
    speed = args.speed
    rate = args.setpoint_rate
    send_velocity_setpoint(master, speed, 0.0)

    for _ in range(int(rate)):
        send_velocity_setpoint(master, speed, 0.0)
        time.sleep(1.0 / rate)

    set_px4_mode(
        master,
        "OFFBOARD",
        keepalive=lambda: send_velocity_setpoint(master, speed, 0.0),
    )
    stream_velocity(master, lambda _: (speed, 0.0), args.straight_before, rate)
    turn_start_wall = time.time()

    def turn_velocity(elapsed):
        phase = smootherstep(elapsed / args.turn_duration)
        angle = 0.5 * math.pi * phase
        return speed * math.cos(angle), speed * math.sin(angle)

    stream_velocity(master, turn_velocity, args.turn_duration, rate)
    turn_end_wall = time.time()
    stream_velocity(master, lambda _: (0.0, speed), args.straight_after, rate)
    stream_velocity(master, lambda _: (0.0, 0.0), 3.0, rate)
    set_px4_mode(
        master,
        "LAND",
        keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0),
    )
    return {"turn_start_wall": turn_start_wall, "turn_end_wall": turn_end_wall}



def run_maneuver(master, args):
    rate = args.setpoint_rate
    speed = args.speed

    send_velocity_setpoint(master, 0.0, 0.0)

    for _ in range(int(rate)):
        send_velocity_setpoint(master, 0.0, 0.0)
        time.sleep(1.0 / rate)

    set_px4_mode(
        master,
        "OFFBOARD",
        keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0),
    )
    start_wall = time.time()

    if args.maneuver == "accel":
        peak = max(1.8, speed)

        def velocity_profile(elapsed):
            if elapsed < 4.0:
                return peak * smootherstep(elapsed / 4.0), 0.0
            if elapsed < 8.0:
                return peak, 0.0
            if elapsed < 12.0:
                return peak * (1.0 - smootherstep((elapsed - 8.0) / 4.0)), 0.0
            if elapsed < 16.0:
                return -0.8 * peak * smootherstep((elapsed - 12.0) / 4.0), 0.0
            if elapsed < 20.0:
                return -0.8 * peak, 0.0
            return -0.8 * peak * (1.0 - smootherstep((elapsed - 20.0) / 4.0)), 0.0

        duration = 24.0

    elif args.maneuver == "slalom":
        travel_speed = max(1.5, speed)
        heading_amplitude = math.radians(60.0)
        period = 8.0

        def velocity_profile(elapsed):
            angle = heading_amplitude * math.sin(2.0 * math.pi * elapsed / period)
            return travel_speed * math.cos(angle), travel_speed * math.sin(angle)

        duration = 32.0

    else:
        travel_speed = max(2.0, speed)

        def velocity_profile(elapsed):
            if elapsed < 8.0:
                angle = 0.0
            elif elapsed < 10.0:
                angle = 0.5 * math.pi * smootherstep((elapsed - 8.0) / 2.0)
            elif elapsed < 16.0:
                angle = 0.5 * math.pi
            elif elapsed < 18.0:
                angle = 0.5 * math.pi * (1.0 - smootherstep((elapsed - 16.0) / 2.0))
            else:
                angle = 0.0

            return travel_speed * math.cos(angle), travel_speed * math.sin(angle)

        duration = 26.0

    stream_velocity(master, velocity_profile, duration, rate)
    stream_velocity(master, lambda _: (0.0, 0.0), 4.0, rate)
    end_wall = time.time()
    set_px4_mode(
        master,
        "LAND",
        keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0),
    )
    return {
        "maneuver_start_wall": start_wall,
        "maneuver_end_wall": end_wall,
        "maneuver": args.maneuver,
    }

def main():
    args = parse_args()
    RUN_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = RUN_ROOT / f"{args.scenario}_{stamp}"
    run_dir.mkdir(parents=True)
    run_log = run_dir / "px4.log"
    metadata_path = run_dir / "result.json"
    trigger_file = Path(f"/tmp/sad_v2_{os.getpid()}.trigger")

    if trigger_file.exists():
        trigger_file.unlink()

    environment = os.environ.copy()
    environment["HEADLESS"] = "1"
    environment["PX4_NO_FOLLOW_MODE"] = "1"
    environment["PX4_SIM_SPEED_FACTOR"] = "1"

    if args.scenario == "pva-hover":
        environment.update(
            {
                "PX4_STEADY_ATTACK_ENABLE": "1",
                "PX4_STEADY_ATTACK_SCENARIO": "hover",
                "PX4_STEADY_ATTACK_TRIGGER_FILE": str(trigger_file),
                "PX4_STEADY_ATTACK_NORTH_M": str(args.pva_north),
                "PX4_STEADY_ATTACK_EAST_M": str(args.pva_east),
                "PX4_STEADY_ATTACK_DOWN_M": "0",
                "PX4_STEADY_ATTACK_RAMP_S": str(args.pva_ramp),
                "PX4_STEADY_ATTACK_PROFILE": args.pva_profile,
                "PX4_STEADY_ATTACK_GPS": "1",
                "PX4_STEADY_ATTACK_GPS_VEL_CONSISTENT": "1",
                "PX4_STEADY_ATTACK_IMU": "1",
                "PX4_STEADY_ATTACK_ACCEL": "1",
                "PX4_STEADY_ATTACK_GYRO": "0",
                "PX4_STEADY_ATTACK_MAG": "0",
                "PX4_STEADY_ATTACK_BARO": "0",
            }
        )

    start_wall = time.time()

    with run_log.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            ["make", "px4_sitl_default", "gazebo_iris"],
            cwd=str(PX4_ROOT),
            env=environment,
            stdin=subprocess.PIPE,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

        event_times = {}

        try:
            wait_for_log_text(run_log, "Startup script returned successfully", 120)
            parameters = apply_detector_parameters(process, args)

            master = mavutil.mavlink_connection(args.url, autoreconnect=True)
            master.wait_heartbeat(timeout=60)
            request_local_position(master)
            takeoff_offboard(
                master,
                args.takeoff_altitude,
                rate_hz=args.setpoint_rate,
            )
            wait_stable_hover(
                master,
                args.takeoff_altitude,
                rate_hz=args.setpoint_rate,
                keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0, 0.0),
            )

            if args.scenario == "turn":
                event_times.update(run_turn(master, args))

            elif args.scenario == "maneuver":
                event_times.update(run_maneuver(master, args))

            else:
                # Leave an in-ULog attack-onset marker while keeping the
                # sequential threshold effectively unchanged.
                marker_delta = max(1e-3, abs(args.threshold) * 1e-6)
                shell(
                    process,
                    f"param set SAD_THRESH {args.threshold + marker_delta}",
                    0.15,
                )
                trigger_file.write_text("trigger\n", encoding="utf-8")
                event_times["attack_trigger_wall"] = time.time()
                stream_velocity(
                    master,
                    lambda _: (0.0, 0.0),
                    args.pva_ramp + args.pva_observation,
                    args.setpoint_rate,
                )
                set_px4_mode(
                    master,
                    "LAND",
                    keepalive=lambda: send_velocity_setpoint(master, 0.0, 0.0, 0.0),
                )

            time.sleep(5.0)

        finally:
            terminate_process(process)

    if trigger_file.exists():
        trigger_file.unlink()

    ulog_path = latest_ulog_after(start_wall)

    if ulog_path is None:
        raise RuntimeError("no ULog produced")

    result = {
        "scenario": args.scenario,
        "run_log": str(run_log),
        "ulog": str(ulog_path),
        "parameters": parameters,
        "events": event_times,
        "detector": summarize_detector(ulog_path, args.scenario),
    }

    if args.scenario == "pva-hover":
        result["attack_spec"] = {
            "north_m": args.pva_north,
            "east_m": args.pva_east,
            "ramp_s": args.pva_ramp,
            "profile": args.pva_profile,
            "observation_s": args.pva_observation,
        }
    elif args.scenario == "turn":
        result["turn_spec"] = {
            "speed_m_s": args.speed,
            "turn_duration_s": args.turn_duration,
            "straight_before_s": args.straight_before,
            "straight_after_s": args.straight_after,
        }
    else:
        result["maneuver_spec"] = {
            "maneuver": args.maneuver,
            "speed_m_s": args.speed,
        }
    metadata_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    print(f"result={metadata_path}")
    print(f"ulog={ulog_path}")
    print(f"run_log={run_log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
