import sys
from pathlib import Path

import numpy as np

MODULE_DIR = Path(__file__).resolve().parents[1]

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from calibrate_actuator_gain import (  # noqa: E402
    interpolated,
    previous_sample,
)


def test_previous_sample_uses_latest_value_at_or_before_target():
    source_timestamp = np.array([10, 20, 30], dtype=np.uint64)
    source_values = np.array([False, True, False])
    target_timestamp = np.array([12, 20, 29], dtype=np.uint64)

    result = previous_sample(
        source_timestamp, source_values, target_timestamp
    )

    assert result.tolist() == [False, True, True]


def test_interpolated_uses_source_timestamps():
    result = interpolated(
        np.array([10, 20], dtype=np.uint64),
        np.array([1.0, 3.0]),
        np.array([15], dtype=np.uint64),
    )

    assert result.tolist() == [2.0]
