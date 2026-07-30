# Threshold calibration and PVA evaluation (2026-07-30)

## Frozen detector parameters

The actuator gain was estimated from four dedicated nominal hover flights.
The GLRT normalization was estimated from five independent fit flights. The
final flight-level threshold used 27 independent nominal calibration flights
and a target flight-level false-alarm probability of 5%.

| Parameter | Frozen value |
|---|---:|
| `SAD_THR_GAIN` | 3.47118998 |
| `SAD_GLRT_MU` | 3.99645138 |
| `SAD_GLRT_SD` | 5.98480034 |
| `SAD_CUS_DR` | 0.5 |
| `SAD_THRESH` | 60.4715042 |
| `SAD_CONSEC` | 3 |

The finite-sample order-statistic check reports
`finite_sample_supported=true` with required rank 27 of 27. The detailed
report is
`build/sad_threshold_calibration/formal_calibration_gain347_v2.json`.

## Nominal held-out results

The first candidate threshold, 54.4246855, produced one flight-level false
alarm in ten held-out nominal flights. The failure occurred during a
1.3 m/s northward maneuver and is retained as failed development evidence.
It was not inserted into the calibration manifest.

After adding eight separately collected, predeclared 1.3 m/s calibration
flights, the final threshold above was evaluated on a second untouched set of
eight nominal flights:

- flight-level false alarms: 0/8;
- alert samples: 0;
- maximum held-out CUSUM: 58.2543602;
- threshold margin: 2.2171440.

The second test set includes a 60 s hover, six 1.3 m/s cardinal/diagonal
maneuvers, and one 0.7 m/s diagonal maneuver. The inventory is
`build/sad_threshold_calibration/nominal_test_v2_inventory_gain347.csv`.

The 0/8 result is an empirical held-out check, not by itself a precise
frequentist guarantee on a very low false-alarm probability.

## PVA attack results

All PVA experiments used the frozen detector parameters. The attack modified
GPS position, derivative-consistent GPS velocity, and IMU acceleration using
the same smooth bias trajectory. No attack result was used to change a
threshold.

The retained operational scenario uses a 100 m target bias established in
20 s. For the quintic smootherstep profile this implies:

| Quantity | Value |
|---|---:|
| Target bias | 100 m |
| Establishment time | 20 s |
| Maximum injected velocity | 9.375 m/s |
| Maximum injected acceleration | 1.443 m/s² |
| Observation time after establishment | 5 s |

The scenario was repeated in four cardinal directions with the detector
parameters still frozen:

| Direction | Delay (s) | Injected bias at alert (m) | True displacement at alert (m) |
|---|---:|---:|---:|
| North | 2.423 | 1.471 | 4.693 |
| East | 2.490 | 1.587 | 4.803 |
| South | 2.377 | 1.394 | 4.214 |
| West | 2.444 | 1.506 | 4.554 |

Results:

- detector-level detection rate: 4/4;
- native PX4 failsafe during the attack: 0/4;
- pre-attack detector alerts: 0;
- final GPS attack bias: 100.009–100.019 m;
- final true displacement: 99.776–100.031 m;
- final estimated displacement: 0.334–0.470 m.

The direction estimate at first alert has cosine approximately -1 relative
to the injected GPS-bias direction. At this early phase it points along the
vehicle's physical displacement, which is opposite to the injected bias.
The direction output must therefore be given an explicit semantic convention
before being claimed as attack-direction identification.

The complete four-direction inventory is
`build/sad_threshold_calibration/pva_primary_100m20s_inventory_gain347.csv`.

## Conclusion

Under the validated operational threat model, the calibrated detector
detects the 100 m / 20 s PVA attack in all tested directions and alarms after
approximately 2.4 s, when the true displacement is about 4.2–4.8 m. This is
materially earlier than completion of the 100 m manipulation. Claims remain
limited to this validated amplitude-duration envelope rather than generalized
to arbitrary smooth PVA attacks.
