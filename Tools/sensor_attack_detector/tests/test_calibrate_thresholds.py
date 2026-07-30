import sys
from pathlib import Path

import numpy as np

MODULE_DIR = Path(__file__).resolve().parents[1]

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from calibrate_thresholds import (  # noqa: E402
    order_statistic_threshold,
    sequential_statistics,
)


def test_sequential_statistic_matches_consecutive_rule():
    flight = {
        "valid": np.array([True, True, True, True]),
        "score": np.array([2.0, 2.0, 2.0, 0.0]),
    }
    maximum, alarm_statistic = sequential_statistics(
        flight,
        mean=0.0,
        standard_deviation=1.0,
        drift=0.5,
        consecutive=3,
    )

    assert maximum == 4.5
    assert alarm_statistic == 3.0


def test_invalid_sample_resets_cusum_and_consecutive_history():
    flight = {
        "valid": np.array([True, True, False, True, True]),
        "score": np.full(5, 2.0),
    }
    maximum, alarm_statistic = sequential_statistics(
        flight,
        mean=0.0,
        standard_deviation=1.0,
        drift=0.5,
        consecutive=3,
    )

    assert maximum == 3.0
    assert alarm_statistic == 0.0


def test_five_percent_order_statistic_requires_nineteen_flights():
    insufficient = order_statistic_threshold([1.0, 2.0], 0.05)
    supported = order_statistic_threshold(range(1, 20), 0.05)

    assert not insufficient["finite_sample_supported"]
    assert insufficient["recommended_threshold"] is None
    assert insufficient["minimum_flights_for_target"] == 19
    assert supported["finite_sample_supported"]
    assert supported["required_rank"] == 19
    assert supported["recommended_threshold"] == 19.0
