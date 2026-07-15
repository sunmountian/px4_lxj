#!/usr/bin/env python3

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError as exc:
    raise SystemExit('pymavlink is required to run this script') from exc


PX4_ROOT = Path(__file__).resolve().parents[1]
RUN_LOG_DIR = Path('/home/lixj/proj/px4_attack_runs')
ULOG_ROOT = PX4_ROOT / 'build' / 'px4_sitl_default' / 'tmp' / 'rootfs' / 'log'


def command_long(master, command, params, retries=3):
    for _ in range(retries):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command,
            0,
            *params,
        )
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

        if ack is not None and ack.command == command:
            return ack

    return None


def request_local_position(master, rate_hz):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        int(1_000_000 / rate_hz),
        0,
        0,
        0,
        0,
        0,
    )


def wait_stable_hover(master, target_altitude_m, stable_seconds, timeout_s):
    stable_since = None
    first_stable_time = None
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)

        if msg is None:
            continue

        horizontal_speed = math.hypot(float(msg.vx), float(msg.vy))
        altitude_error = abs(float(msg.z) + target_altitude_m)
        stable = altitude_error < 0.30 and horizontal_speed < 0.15 and abs(float(msg.vz)) < 0.10

        if stable:
            if stable_since is None:
                stable_since = time.monotonic()
                first_stable_time = stable_since

            if time.monotonic() - stable_since >= stable_seconds:
                return {
                    'x': float(msg.x),
                    'y': float(msg.y),
                    'z': float(msg.z),
                    'vx': float(msg.vx),
                    'vy': float(msg.vy),
                    'vz': float(msg.vz),
                    'first_stable_elapsed_s': first_stable_time,
                }

        else:
            stable_since = None
            first_stable_time = None

    raise TimeoutError('stable hover was not detected before timeout')


def latest_ulog_after(start_time):
    if not ULOG_ROOT.exists():
        return None

    logs = [path for path in ULOG_ROOT.rglob('*.ulg') if path.stat().st_mtime >= start_time]

    if not logs:
        return None

    return max(logs, key=lambda path: path.stat().st_mtime)


def summarize_ulog(path):
    try:
        from pyulog import ULog
        import numpy as np
    except ImportError:
        return {'error': 'pyulog/numpy unavailable'}

    result = {}
    ulog = ULog(str(path), None, disable_str_exceptions=True)

    def get_dataset(name):
        try:
            return ulog.get_dataset(name)
        except Exception:
            return None

    def values(dataset, field):
        return np.asarray(dataset.data[field], dtype=float)

    def stat(array):
        array = np.asarray(array, dtype=float)
        array = array[np.isfinite(array)]

        if array.size == 0:
            return None

        return {
            'mean': float(np.mean(array)),
            'p95': float(np.percentile(array, 95)),
            'max': float(np.max(array)),
            'last': float(array[-1]),
        }

    estimator_status = get_dataset('estimator_status')
    ratios = get_dataset('estimator_innovation_test_ratios')
    local_position = get_dataset('vehicle_local_position')
    gps = get_dataset('vehicle_gps_position')

    if estimator_status is not None:
        for field in ['pos_test_ratio', 'vel_test_ratio', 'hgt_test_ratio', 'mag_test_ratio']:
            result[field] = stat(values(estimator_status, field))

        for field in ['innovation_check_flags', 'gps_check_fail_flags', 'filter_fault_flags']:
            data = values(estimator_status, field)
            result[field] = {
                'max': int(np.max(data)),
                'nonzero': int(np.count_nonzero(data)),
                'count': int(data.size),
            }

    if ratios is not None:
        for field in ['heading', 'gps_hpos[0]', 'gps_hvel[0]', 'gps_vpos', 'gps_vvel']:
            result[field] = stat(values(ratios, field))

    if local_position is not None:
        result['local_position_last'] = {
            field: float(values(local_position, field)[-1])
            for field in ['x', 'y', 'z', 'vx', 'vy', 'vz']
        }

    if gps is not None:
        lat = values(gps, 'lat') * 1e-7
        lon = values(gps, 'lon') * 1e-7
        alt = values(gps, 'alt') * 1e-3
        radius = 6378137.0
        lat0 = math.radians(float(lat[0]))
        north = (lat - lat[0]) * math.pi / 180.0 * radius
        east = (lon - lon[0]) * math.pi / 180.0 * radius * math.cos(lat0)
        up = alt - alt[0]
        result['gps_offset_last'] = {
            'north': float(north[-1]),
            'east': float(east[-1]),
            'up': float(up[-1]),
        }

    return result


def terminate_process(process):
    if process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)

    try:
        process.wait(timeout=15)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(process.pid, signal.SIGTERM)

    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description='Run a stable-hover PX4 SITL steady attack experiment.')
    parser.add_argument('--mode', choices=['baseline', 'gps-only', 'gps-accel'], default='gps-accel')
    parser.add_argument('--north', type=float, default=5.0)
    parser.add_argument('--east', type=float, default=0.0)
    parser.add_argument('--down', type=float, default=0.0)
    parser.add_argument('--ramp', type=float, default=30.0)
    parser.add_argument('--takeoff-altitude', type=float, default=10.0)
    parser.add_argument('--stable-seconds', type=float, default=5.0)
    parser.add_argument('--hover-timeout', type=float, default=90.0)
    parser.add_argument('--attack-duration', type=float, default=80.0)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--local-position-rate', type=float, default=20.0)
    parser.add_argument('--preflight-wait', type=float, default=15.0)
    args = parser.parse_args()

    RUN_LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    run_log = RUN_LOG_DIR / f'steady_{args.mode}_{timestamp}.log'
    trigger_file = Path(f'/tmp/px4_steady_attack_{os.getpid()}.trigger')

    if trigger_file.exists():
        trigger_file.unlink()

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_NO_FOLLOW_MODE'] = '1'
    env['PX4_SIM_SPEED_FACTOR'] = '1'

    if args.mode != 'baseline':
        env['PX4_STEADY_ATTACK_ENABLE'] = '1'
        env['PX4_STEADY_ATTACK_SCENARIO'] = 'hover'
        env['PX4_STEADY_ATTACK_TRIGGER_FILE'] = str(trigger_file)
        env['PX4_STEADY_ATTACK_NORTH_M'] = str(args.north)
        env['PX4_STEADY_ATTACK_EAST_M'] = str(args.east)
        env['PX4_STEADY_ATTACK_DOWN_M'] = str(args.down)
        env['PX4_STEADY_ATTACK_RAMP_S'] = str(args.ramp)
        env['PX4_STEADY_ATTACK_GPS'] = '1'
        env['PX4_STEADY_ATTACK_GPS_VEL_CONSISTENT'] = '1'
        env['PX4_STEADY_ATTACK_IMU'] = '0' if args.mode == 'gps-only' else '1'
        env['PX4_STEADY_ATTACK_ACCEL'] = '1'
        env['PX4_STEADY_ATTACK_GYRO'] = '1'
        env['PX4_STEADY_ATTACK_MAG'] = '1'
        env['PX4_STEADY_ATTACK_BARO'] = '1'

    start_wall_time = time.time()

    with run_log.open('w', encoding='utf-8') as log_file:
        process = subprocess.Popen(
            ['make', 'px4_sitl_default', 'gazebo_iris'],
            cwd=str(PX4_ROOT),
            env=env,
            stdin=subprocess.PIPE,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
        )

        try:
            # Pipe commander takeoff via stdin (proven to work in this setup)
            time.sleep(args.preflight_wait)
            process.stdin.write('commander takeoff\n')
            process.stdin.flush()

            # Also connect MAVLink for telemetry and stable hover detection
            master = mavutil.mavlink_connection(args.url, autoreconnect=True)
            master.wait_heartbeat(timeout=60)
            request_local_position(master, args.local_position_rate)

            hover_state = wait_stable_hover(master, args.takeoff_altitude, args.stable_seconds, args.hover_timeout)
            trigger_time = None

            if args.mode != 'baseline':
                trigger_file.write_text('trigger\n', encoding='utf-8')
                trigger_time = time.time()

            time.sleep(args.attack_duration)
            process.stdin.write('commander land\n')
            process.stdin.flush()
            time.sleep(5)

        finally:
            terminate_process(process)

    if trigger_file.exists():
        trigger_file.unlink()

    ulog = latest_ulog_after(start_wall_time)
    summary = summarize_ulog(ulog) if ulog is not None else {'error': 'no ULog found'}

    print(f'run_log={run_log}')
    print(f'ulog={ulog}')
    print(f'mode={args.mode}')

    if args.mode != 'baseline':
        print(f'trigger_file={trigger_file}')

    print('summary=')
    print(summary)


if __name__ == '__main__':
    sys.exit(main())
