"""Inject calibration-only parameters into an existing SITL experiment."""

import os
import threading
import time


PARAMETER_ENVIRONMENT = {
    "SAD_THR_GAIN": "SAD_CAL_THR_GAIN",
    "SAD_WA": "SAD_CAL_WA",
    "SAD_WV": "SAD_CAL_WV",
    "SAD_WP": "SAD_CAL_WP",
    "SAD_REG": "SAD_CAL_REG",
    "SAD_AS_MU": "SAD_CAL_AS_MU",
    "SAD_AS_SD": "SAD_CAL_AS_SD",
    "SAD_AD_MU": "SAD_CAL_AD_MU",
    "SAD_AD_SD": "SAD_CAL_AD_SD",
    "SAD_GLRT_MU": "SAD_CAL_GLRT_MU",
    "SAD_GLRT_SD": "SAD_CAL_GLRT_SD",
    "SAD_CUS_DR": "SAD_CAL_CUS_DR",
    "SAD_THRESH": "SAD_CAL_THRESH",
    "COM_OF_LOSS_T": "SAD_CAL_COM_OF_LOSS_T",
}

INTEGER_PARAMETER_ENVIRONMENT = {
    "SAD_CONSEC": "SAD_CAL_CONSEC",
}


def patch_gcs_heartbeat(experiment, period_s=0.5):
    """Keep the calibration MAVLink data link alive during takeoff."""

    original_connection = experiment.mavutil.mavlink_connection

    def mavlink_connection(*args, **kwargs):
        master = original_connection(*args, **kwargs)

        def send_heartbeat():
            while True:
                try:
                    master.mav.heartbeat_send(
                        experiment.mavutil.mavlink.MAV_TYPE_GCS,
                        experiment.mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        experiment.mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                except Exception:
                    pass

                time.sleep(period_s)

        threading.Thread(target=send_heartbeat, daemon=True).start()
        return master

    experiment.mavutil.mavlink_connection = mavlink_connection


def patch_offboard_setpoint_keepalive(experiment, period_s=0.1):
    """Keep the latest velocity target alive across Python scheduling stalls.

    The foreground experiment still owns the requested velocity.  This helper
    only repeats its most recent target while OFFBOARD is active, preventing a
    transient host-side scheduling delay from being recorded as a flight
    failsafe in an otherwise valid calibration run.
    """

    original_send = experiment.send_velocity_setpoint
    target_lock = threading.Lock()
    target = {
        "active": False,
        "master": None,
        "vn": 0.0,
        "ve": 0.0,
        "vd": 0.0,
    }

    def send_velocity_setpoint(master, vn, ve, vd):
        with target_lock:
            target.update(
                {
                    "master": master,
                    "vn": float(vn),
                    "ve": float(ve),
                    "vd": float(vd),
                }
            )

        return original_send(master, vn, ve, vd)

    def set_active(active):
        with target_lock:
            target["active"] = bool(active)

    def repeat_latest_target():
        while True:
            with target_lock:
                snapshot = target.copy()

            if snapshot["active"] and snapshot["master"] is not None:
                try:
                    original_send(
                        snapshot["master"],
                        snapshot["vn"],
                        snapshot["ve"],
                        snapshot["vd"],
                    )
                except Exception:
                    pass

            time.sleep(period_s)

    experiment.send_velocity_setpoint = send_velocity_setpoint
    experiment.sad_set_offboard_keepalive_active = set_active
    threading.Thread(target=repeat_latest_target, daemon=True).start()


def patch_preflight_console_parameters(experiment):
    """Apply calibration parameters immediately after logger startup.

    The PX4 shell handles float and bitmask parameter types correctly.  Writing
    here also makes the configured values active before the detector can fill
    its first eight-second GLRT window, rather than waiting for the later
    MAVLink connection.
    """

    console_parameters = []

    for mapping in (
        PARAMETER_ENVIRONMENT,
        INTEGER_PARAMETER_ENVIRONMENT,
    ):
        console_parameters.extend(
            (parameter, os.environ[environment])
            for parameter, environment in mapping.items()
            if environment in os.environ
        )

    offboard_loss_timeout = float(
        os.environ.get("SAD_CAL_COM_OF_LOSS_T", "2.0")
    )

    if not 0.5 <= offboard_loss_timeout <= 60.0:
        raise ValueError(
            "SAD_CAL_COM_OF_LOSS_T must be between 0.5 and 60 seconds"
        )

    if "SAD_CAL_COM_OF_LOSS_T" not in os.environ:
        console_parameters.append(
            ("COM_OF_LOSS_T", str(offboard_loss_timeout))
        )

    if "SAD_CAL_COM_RCL_EXCEPT" in os.environ:
        exception_mask = int(os.environ["SAD_CAL_COM_RCL_EXCEPT"])

        if exception_mask < 0:
            raise ValueError("SAD_CAL_COM_RCL_EXCEPT must be non-negative")

        console_parameters.append(("COM_RCL_EXCEPT", str(exception_mask)))

    rc_input_mode = int(os.environ.get("SAD_CAL_COM_RC_IN_MODE", "4"))

    if rc_input_mode != 4:
        raise ValueError(
            "SAD_CAL_COM_RC_IN_MODE must be 4 for no-RC calibration"
        )

    console_parameters.append(("COM_RC_IN_MODE", str(rc_input_mode)))

    if not console_parameters:
        return

    original_wait_for_log_text = experiment.wait_for_log_text
    original_send_console_command = experiment.send_console_command
    applied = False

    def wait_for_log_text(process, log_path, needle, timeout_s):
        nonlocal applied
        response = original_wait_for_log_text(
            process, log_path, needle, timeout_s
        )

        if needle == "Opened full log file" and not applied:
            original_wait_for_log_text(
                process,
                log_path,
                "Startup script returned successfully",
                timeout_s,
            )

            for parameter, value in console_parameters:
                original_send_console_command(
                    process, f"param set {parameter} {value}"
                )
                time.sleep(0.1)

            applied = True

        return response

    experiment.wait_for_log_text = wait_for_log_text


def patch_float_parameter_setter(experiment):
    values = {
        parameter: float(os.environ[environment])
        for parameter, environment in PARAMETER_ENVIRONMENT.items()
        if environment in os.environ
    }

    if not values:
        return

    original = experiment.set_float_parameter
    applied = False

    def set_float_parameter(master, name, value, timeout_s=10):
        nonlocal applied
        response = original(master, name, value, timeout_s)

        if name == "MIS_TAKEOFF_ALT" and not applied:
            for parameter, candidate in values.items():
                original(master, parameter, candidate, timeout_s)

            applied = True

        return response

    experiment.set_float_parameter = set_float_parameter
