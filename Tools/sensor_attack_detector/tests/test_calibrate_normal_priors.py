import math
import sys
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parents[1]

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from calibrate_normal_priors import equal_flight_prior  # noqa: E402


def test_equal_flight_prior_uses_flight_means_only():
    flights = [
        {"scale_mean": 1.0, "scale_variance": 0.25},
        {"scale_mean": 3.0, "scale_variance": 0.25},
    ]

    mean, standard_deviation = equal_flight_prior(flights, "scale")

    assert mean == 2.0
    assert math.isclose(standard_deviation, math.sqrt(2.0))


def test_equal_flight_moments_include_within_flight_variance():
    flights = [
        {"dynamic_mean": -0.5, "dynamic_variance": 0.04},
        {"dynamic_mean": -0.5, "dynamic_variance": 0.16},
    ]

    mean, standard_deviation = equal_flight_moments(flights, "dynamic")

    assert mean == -0.5
    assert math.isclose(standard_deviation, math.sqrt(0.10))
