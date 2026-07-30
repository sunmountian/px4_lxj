import sys
from pathlib import Path
from types import SimpleNamespace

MODULE_DIR = Path(__file__).resolve().parents[1]

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from runtime_parameters import patch_preflight_console_parameters  # noqa: E402


def test_console_bitmask_is_applied_after_logger_startup(monkeypatch):
    calls = []

    def wait_for_log_text(process, log_path, needle, timeout_s):
        calls.append(("wait", needle))
        return "ready"

    def send_console_command(process, command):
        calls.append(("command", command))

    experiment = SimpleNamespace(
        wait_for_log_text=wait_for_log_text,
        send_console_command=send_console_command,
    )
    monkeypatch.setenv("SAD_CAL_COM_RCL_EXCEPT", "6")
    monkeypatch.setenv("SAD_CAL_COM_OF_LOSS_T", "2.0")
    monkeypatch.setenv("SAD_CAL_THR_GAIN", "3.47")

    patch_preflight_console_parameters(experiment)
    response = experiment.wait_for_log_text(
        object(), Path("run.log"), "Opened full log file", 30
    )

    assert response == "ready"
    assert calls == [
        ("wait", "Opened full log file"),
        ("wait", "Startup script returned successfully"),
        ("command", "param set SAD_THR_GAIN 3.47"),
        ("command", "param set COM_OF_LOSS_T 2.0"),
        ("command", "param set COM_RCL_EXCEPT 6"),
        ("command", "param set COM_RC_IN_MODE 4"),
    ]


def test_console_bitmask_is_written_only_once(monkeypatch):
    calls = []
    experiment = SimpleNamespace(
        wait_for_log_text=lambda *args: None,
        send_console_command=lambda process, command: calls.append(command),
    )
    monkeypatch.setenv("SAD_CAL_COM_RCL_EXCEPT", "6")

    patch_preflight_console_parameters(experiment)
    experiment.wait_for_log_text(None, None, "Opened full log file", 30)
    experiment.wait_for_log_text(None, None, "Opened full log file", 30)

    assert calls == [
        "param set COM_OF_LOSS_T 2.0",
        "param set COM_RCL_EXCEPT 6",
        "param set COM_RC_IN_MODE 4",
    ]
