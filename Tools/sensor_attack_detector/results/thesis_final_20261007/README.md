# Thesis-final sensor attack detector calibration — 2026-10-07

This directory records the frozen PX4 SITL calibration used for the thesis
sensor-attack detector. The detector remains the fixed 7x7 GLRT per horizontal
axis (one nominal bias coefficient plus six attack-basis coefficients), followed
by normalized CUSUM sequential detection.

## Frozen physical reference

- `SAD_THR_MAP = 0` (linear actuator-command sum)
- `SAD_THR_GAIN = 3.47118998`
- `SAD_DRAG_K = 0.20 1/s`
- `SAD_DRAG_MOD = 0`
- `SAD_DYN_LAG = 0.20 s`
- `SAD_AS_MU = -0.33720698952674866`
- `SAD_AD_MU = 0.42821773886680603`

The constant damping term is a lightweight physical-reference correction, not
an additional online fitted degree of freedom.

## Frozen residual and sequential parameters

| Parameter | Value |
|---|---:|
| SAD_WA | 311.40721173819645 |
| SAD_WV | 226.9711056439403 |
| SAD_WP | 159.00814015827592 |
| SAD_GLRT_MU | 4.917112030075899 |
| SAD_GLRT_SD | 6.4710112836256135 |
| SAD_CUS_DR | 0.5 |
| SAD_THRESH | 56.313146192854525 |
| SAD_CONSEC | 3 |

Residual standard deviations from the six-flight mixed-normal weight
calibration were 0.0566677 m/s^2 (acceleration), 0.0663766 m/s (velocity), and
0.0793031 m (position).

## Calibration protocol

- 6 independent nominal flights: A/V/P residual-order weights.
- 5 independent nominal flights: fixed GLRT mean and standard deviation.
- 19 independent nominal flights: 5% flight-level empirical CUSUM threshold.
- 6 held-out nominal flights: final false-alert check.
- Attack flights are validation only and are not used to tune nominal
  normalization or thresholds.

The held-out nominal set contained hover, ordinary turns, and
acceleration/deceleration maneuvers.

| Held-out flight | Scenario | GLRT mean | GLRT max | Max CUSUM | Alert |
|---|---|---:|---:|---:|---|
| test01 | hover | 1.118 | 11.084 | 1.059 | no |
| test02 | turn | 2.453 | 13.865 | 1.044 | no |
| test03 | turn | 4.222 | 22.935 | 8.629 | no |
| test04 | turn | 4.126 | 24.745 | 7.340 | no |
| test05 | accel | 7.247 | 29.182 | 27.674 | no |
| test06 | accel | 8.658 | 34.823 | 46.221 | no |

Held-out nominal alerts: **0 / 6**.

## Attack validation

All listed attack flights had zero pre-attack alerts.

| Attack | First alert delay (s) | Post-attack GLRT mean | Detected |
|---|---:|---:|---|
| +North 100 m / 20 s | 4.136 | 323.620 | yes |
| -North 100 m / 20 s | 4.156 | 1041.108 | yes |
| +East 100 m / 20 s | 4.148 | 316.892 | yes |
| -East 100 m / 20 s | 4.156 | 925.915 | yes |
| +North 100 m / 30 s | 6.144 | 72.349 | yes |
| +North 100 m / 40 s | 8.944 | 25.509 | yes |
| sinusoidal +North 100 m / 20 s | 4.552 | 547.760 | yes |

The increasing detection delay for 20/30/40 s ramps is consistent with the
attack-observability scaling with trajectory curvature (approximately B/T^2).

## Source runs

- Thesis candidate validation: GitHub Actions run 37601664290.
- Thesis final mixed-normal calibration and attack matrix: GitHub Actions run 37601943666.
- Final artifact: `sad-thesis-final-calibration`, artifact id 11475429012.

These are PX4 SITL results. They should not be described as hardware-flight
evidence.


## Independent false-alarm follow-up

After freezing the nominal model, two additional independent nominal validation
batches were run to check flight-level false alarms beyond the original six
held-out flights.

- Frozen nominal FAR validation (20 flights) at the frozen threshold
  `SAD_THRESH = 56.313146192854525`: 2 / 20 flights alerted.
- Threshold-refinement held-out validation (20 new flights) used an independently
  re-estimated threshold of 55.613822937011726 and produced 0 / 20 alerted
  flights.

Because the second independently estimated threshold is slightly *lower* than
the frozen threshold while producing zero alerts on its held-out batch, the two
batches indicate ordinary finite-sample / run-to-run variation rather than a
systematic loss of nominal separation. We therefore retain the original frozen
threshold 56.313146192854525 instead of post-hoc tuning it to one validation
batch. At the retained threshold, the first 20-flight batch has 10% FAR and the
second batch would have no more alerts than observed at the lower threshold;
pooled over the two independent 20-flight follow-up batches this gives at most
2 / 40 = 5% flight-level FAR. Including the original six held-out flights gives
2 / 46 observed false-alert flights across all independent nominal checks.

The threshold-refinement attack recheck still detected all tested attacks with
zero pre-attack alerts:

| Attack | First alert delay (s) | Post-attack GLRT mean | Detected |
|---|---:|---:|---|
| +North 100 m / 20 s | 4.040 | 315.392 | yes |
| +North 100 m / 30 s | 6.104 | 72.987 | yes |
| +North 100 m / 40 s | 8.900 | 25.620 | yes |

The refinement run is GitHub Actions run 37625886666. The lower 55.61 threshold
was used only as an independent sensitivity/validation experiment and is not the
frozen thesis default.
