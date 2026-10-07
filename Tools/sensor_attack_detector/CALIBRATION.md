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

Changing `SAD_THR_GAIN`, `SAD_THR_MAP`, `SAD_DRAG_K`, `SAD_DRAG_MOD`, `SAD_DYN_LAG`, the actuator source, residual weights, or GLRT regularization invalidates every previously collected raw-GLRT threshold log.

For no-RC SITL collection, use `COM_RCL_EXCEPT=7` so Mission/Takeoff, Hold,
and Offboard are all exempt, and set `COM_RC_IN_MODE=4` to disable stick
input. A value of 6 omits the Mission/Takeoff bit; leaving RC input enabled can
still produce a short pre-Offboard failsafe. Use `COM_OF_LOSS_T=2.0 s` in the
host-driven SITL wrapper so a sub-second host scheduling stall does not create
a synthetic Offboard-loss event. This changes only the experiment link-loss
timeout; the threshold calibrator still rejects every nominal ULog containing
`vehicle_status.failsafe`.

## 2. Calibrate the fixed normal-dynamics correction

The final detector does not estimate actuator-scale and actuator-dynamic
coefficients online. It first removes two calibrated nominal correction terms,

`r* = r - alpha_bar * G_s - beta_bar * G_d`,

and the online H0 model contains only the window bias column. This keeps the
attack GLRT at seven unknowns per horizontal axis: one bias plus six attack
basis coefficients.

Collect a dedicated nominal bootstrap set with sufficient maneuver excitation,
especially turn entry, sustained turning, and turn exit. Estimate the common
horizontal correction coefficients from nominal data only. The existing `calibrate_normal_priors.py` tool is a bootstrap estimator for
ULogs collected with the earlier weak-prior V2 calibration model, where the
scale/dynamic coefficients were still estimated online. It must not be run on
final fixed-model ULogs, whose `normal_scale_*` and `normal_dynamic_*` status
fields simply report the already frozen coefficients. For the fixed model only
the bootstrap means `SAD_AS_MU` and `SAD_AD_MU` are used online. The reported
standard deviations are diagnostics/legacy experiment fields and do not enter
the fixed-model GLRT.

Freeze `SAD_AS_MU` and `SAD_AD_MU` before collecting residual-weight,
threshold-fit, calibration, or test flights. Bootstrap flights used to estimate
these coefficients must not later be counted as untouched detector test flights.

## 2.1 Calibrate residual-order weights

With the fixed correction coefficients frozen, collect a separate nominal set
and run the detector with equal initial residual-order weights. The online code
already divides each residual order by its sample count. Calibrate the remaining
inverse-variance factors from the published post-H0 residual RMS values:

```sh
python3 Tools/sensor_attack_detector/calibrate_residual_weights.py \
    --manifest build/sad_v2/weight_manifest.csv \
    --output build/sad_v2/residual_weight_calibration.json
```

This produces `SAD_WA`, `SAD_WV`, and `SAD_WP`. Freeze them before
collecting any GLRT normalization, CUSUM calibration, or held-out test flight.

## 3. Calibrate normalization and the flight-level threshold

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

All ULogs in one threshold calibration must use identical values for `SAD_ACT_SRC`,
`SAD_REG`, `SAD_THR_GAIN`, `SAD_THR_MAP`, `SAD_DRAG_K`, `SAD_DRAG_MOD`,
`SAD_DYN_LAG`, `SAD_WA`, `SAD_WP`, `SAD_WV`, `SAD_AS_MU`, and `SAD_AD_MU`. Changing any of these parameters invalidates
the calibrated normalization and threshold.

The 2026-07-30 SITL calibration and held-out PVA evaluation are summarized in
`results/threshold_calibration_pva_20260730/README.md`.


## 4. Frozen thesis configuration

The thesis detector configuration frozen from the 2026-10-07 PX4 SITL
calibration uses the linear actuator map with a lightweight first-order
horizontal damping correction. The physical-reference parameters are:

- `SAD_THR_MAP = 0`
- `SAD_THR_GAIN = 3.47118998`
- `SAD_DRAG_K = 0.20 1/s`
- `SAD_DRAG_MOD = 0`
- `SAD_DYN_LAG = 0.20 s`
- `SAD_AS_MU = -0.33720698952674866`
- `SAD_AD_MU = 0.42821773886680603`

The final mixed-normal residual calibration gives:

- `SAD_WA = 311.40721173819645`
- `SAD_WV = 226.9711056439403`
- `SAD_WP = 159.00814015827592`

The nominal GLRT/CUSUM calibration gives:

- `SAD_GLRT_MU = 4.917112030075899`
- `SAD_GLRT_SD = 6.4710112836256135`
- `SAD_CUS_DR = 0.5`
- `SAD_THRESH = 56.313146192854525`
- `SAD_CONSEC = 3`

The calibration protocol used 6 mixed nominal flights for residual-order
weights, 5 nominal flights for GLRT normalization, 19 nominal flights for the
5% flight-level empirical threshold, and 6 held-out nominal flights for the
final false-alarm check. The held-out set contained hover, ordinary turns, and
acceleration/deceleration maneuvers and produced 0/6 detector alerts.

The final attack matrix includes four directions of the 100 m / 20 s
smootherstep attack, 100 m / 30 s and 100 m / 40 s attacks, and an independent
100 m / 20 s sinusoidal trajectory. These attack logs are validation data and
are not used to tune nominal thresholds.
