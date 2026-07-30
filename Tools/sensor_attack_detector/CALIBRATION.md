# Sensor attack detector threshold calibration

The online detector and its GLRT remain implemented in PX4. The calibration
tool reads only the raw `glrt_score` already published in
`sensor_attack_status`; it does not reconstruct residuals or implement a
second detector in Python.

## 1. Freeze the actuator model first

`SAD_THR_GAIN` is not an alarm threshold. It converts the summed normalized
motor output into acceleration and therefore changes the raw GLRT itself.
Calibrate it from dedicated nominal, steady, level-hover flights before
collecting any threshold-fit, calibration, or test flight:

```sh
python3 Tools/sensor_attack_detector/calibrate_actuator_gain.py \
    --manifest build/sad_threshold_calibration/actuator_gain_manifest.csv \
    --output build/sad_threshold_calibration/actuator_gain_calibration.json
```

The estimator uses the vertical hover balance
`gain * motor_sum * R_zz = g`, forms one median per independent flight, and
then takes the median across flights. Logs used for this physical calibration
must not later be treated as untouched detector test flights.

Changing `SAD_THR_GAIN`, the actuator source, residual weights, or GLRT
regularization invalidates every previously collected raw-GLRT threshold log.

For no-RC SITL collection, use `COM_RCL_EXCEPT=7` so Mission/Takeoff, Hold,
and Offboard are all exempt, and set `COM_RC_IN_MODE=4` to disable stick
input. A value of 6 omits the Mission/Takeoff bit; leaving RC input enabled can
still produce a short pre-Offboard failsafe. Use `COM_OF_LOSS_T=2.0 s` in the
host-driven SITL wrapper so a sub-second host scheduling stall does not create
a synthetic Offboard-loss event. This changes only the experiment link-loss
timeout; the threshold calibrator still rejects every nominal ULog containing
`vehicle_status.failsafe`.

## 2. Calibrate normalization and the flight-level threshold

The manifest is a CSV with one independent nominal flight per row:

```text
flight_id,split,scenario,nominal,ulog
fit_hover_01,fit,hover,true,/absolute/path/to/flight.ulg
cal_hover_01,calibration,hover,true,/absolute/path/to/flight.ulg
test_hover_01,test,hover,true,/absolute/path/to/flight.ulg
```

The three splits have distinct roles:

- `fit` estimates `SAD_GLRT_MU` and `SAD_GLRT_SD`;
- `calibration` selects `SAD_THRESH` from one flight-level alarm statistic
  per nominal flight;
- `test` provides an untouched nominal false-alarm check.

The CUSUM drift and consecutive-crossing count are fixed design inputs. For
`SAD_CONSEC=M`, a flight's calibration statistic is the maximum, over time,
of the minimum CUSUM value in each run of `M` valid consecutive samples. This
matches the PX4 alert rule: a flight alerts if and only if this statistic is
greater than `SAD_THRESH`.

Example:

```sh
python3 Tools/sensor_attack_detector/calibrate_thresholds.py \
    --manifest build/sad_threshold_calibration/nominal_manifest.csv \
    --output build/sad_threshold_calibration/calibration.json \
    --false-alarm-rate 0.05 \
    --drift 0.5 \
    --consecutive 3
```

At least 19 independent calibration flights are needed before the empirical
order-statistic method can return a finite threshold for a 5% flight-level
false-alarm target. Additional independent test flights are still required.

All ULogs in one calibration must use identical values for `SAD_ACT_SRC`,
`SAD_REG`, `SAD_THR_GAIN`, `SAD_WA`, `SAD_WP`, and `SAD_WV`. Changing any of
these parameters invalidates the calibrated normalization and threshold.

The 2026-07-30 SITL calibration and held-out PVA evaluation are summarized in
`results/threshold_calibration_pva_20260730/README.md`.
