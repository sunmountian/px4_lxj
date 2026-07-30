#!/usr/bin/env python3

"""Run the existing constant-velocity experiment with robust PX4 mode checks.

Some pymavlink versions return ``UNKNOWN`` from ``mode_string_v10()`` for PX4
heartbeats. The experiment itself is left untouched; this wrapper verifies the
raw PX4 custom main/sub-mode fields instead.
"""

import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1]

if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from pymavlink import mavutil  # noqa: E402

import constant_velocity_attack_experiment as experiment  # noqa: E402
from attack_experiment_utils import command_long  # noqa: E402
from runtime_parameters import (  # noqa: E402
    patch_float_parameter_setter,
    patch_gcs_heartbeat,
    patch_offboard_setpoint_keepalive,
    patch_preflight_console_parameters,
)


def heartbeat_matches_mode(heartbeat, mode):
    _, expected_main_mode, expected_sub_mode = mavutil.px4_map[mode]
    custom_mode = int(heartbeat.custom_mode)
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    main_matches = main_mode == expected_main_mode
    sub_matches = expected_sub_mode == 0 or sub_mode == expected_sub_mode
    return main_matches and sub_matches


def set_px4_mode_raw(master, mode, timeout_s=30, keepalive=None):
    # The source experiment stops its explicit setpoint loop immediately
    # before requesting LAND.  Keep publishing a zero-velocity setpoint while
    # the mode command/heartbeat acknowledgement is in flight; otherwise a
    # slow acknowledgement can exceed COM_OF_LOSS_T and create a spurious
    # offboard-loss failsafe at the end of an otherwise valid flight.
    if (
        mode == "LAND"
        and keepalive is None
        and getattr(set_px4_mode_raw, "_offboard_entered", False)
    ):
        keepalive = lambda: experiment.send_velocity_setpoint(
            master, 0.0, 0.0, 0.0
        )

    if mode == "OFFBOARD":
        experiment.sad_set_offboard_keepalive_active(True)

    mode_flag, main_mode, sub_mode = mavutil.px4_map[mode]
    deadline = time.monotonic() + timeout_s
    last_ack = None

    while time.monotonic() < deadline:
        last_ack = command_long(
            master,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [mode_flag, main_mode, sub_mode, 0, 0, 0, 0],
            retries=1,
            timeout_s=min(3, max(0.1, deadline - time.monotonic())),
            keepalive=keepalive,
        )

        if last_ack is not None and last_ack.result in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        ):
            break

        if (
            last_ack is not None
            and last_ack.result != mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
        ):
            raise RuntimeError(f"{mode} mode command was rejected: {last_ack}")

        if keepalive is not None:
            keepalive()

        time.sleep(0.05 if keepalive is not None else 0.25)
    else:
        raise TimeoutError(
            f"{mode} mode command was not accepted; last_ack={last_ack}"
        )

    last_state = None

    while time.monotonic() < deadline:
        if keepalive is not None:
            keepalive()

        heartbeat = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=0.05 if keepalive is not None else 1,
        )

        if heartbeat is None or heartbeat.get_srcSystem() != master.target_system:
            continue

        last_state = {
            "armed": bool(
                heartbeat.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ),
            "custom_mode": int(heartbeat.custom_mode),
        }

        if heartbeat_matches_mode(heartbeat, mode):
            if mode == "OFFBOARD":
                set_px4_mode_raw._offboard_entered = True

            elif mode == "LAND":
                set_px4_mode_raw._offboard_entered = False
                experiment.sad_set_offboard_keepalive_active(False)

            last_state["mode"] = mode
            return last_state

    raise TimeoutError(f"vehicle did not enter {mode}; last_state={last_state}")


def main():
    experiment.set_px4_mode = set_px4_mode_raw
    patch_gcs_heartbeat(experiment)
    patch_offboard_setpoint_keepalive(experiment)
    patch_preflight_console_parameters(experiment)
    patch_float_parameter_setter(experiment)

    return experiment.main()


if __name__ == "__main__":
    sys.exit(main())
