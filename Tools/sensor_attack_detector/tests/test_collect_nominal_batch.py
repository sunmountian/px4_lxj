import sys
from pathlib import Path
from types import SimpleNamespace

MODULE_DIR = Path(__file__).resolve().parents[1]

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from collect_nominal_batch import get_effective_parameter  # noqa: E402


def test_effective_parameter_uses_last_in_flight_change():
    ulog = SimpleNamespace(
        initial_parameters={"SAD_THRESH": 54.0},
        changed_parameters=[
            (100, "SAD_THRESH", 100.0),
            (200, "OTHER", 1.0),
            (300, "SAD_THRESH", 10000.0),
        ],
    )

    assert get_effective_parameter(ulog, "SAD_THRESH") == 10000.0


def test_effective_parameter_falls_back_to_initial_value():
    ulog = SimpleNamespace(
        initial_parameters={"SAD_CONSEC": 3},
        changed_parameters=[],
    )

    assert get_effective_parameter(ulog, "SAD_CONSEC") == 3
