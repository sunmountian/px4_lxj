#!/usr/bin/env python3

"""Run a nominal hover with optional candidate detector parameters."""

import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1]

if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import steady_hover_attack_experiment as experiment  # noqa: E402
from runtime_parameters import (  # noqa: E402
    patch_float_parameter_setter,
    patch_gcs_heartbeat,
    patch_preflight_console_parameters,
)


def main():
    patch_gcs_heartbeat(experiment)
    patch_preflight_console_parameters(experiment)
    patch_float_parameter_setter(experiment)
    return experiment.main()


if __name__ == "__main__":
    sys.exit(main())
