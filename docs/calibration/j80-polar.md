# J/80 polar calibration

Overall: **PASS**

## Targets

- Beat and fixed TWA up to 75 deg: ORC certificate (`orc_polar.csv`, 2019 J/80 certificate).
- Fixed TWA above 75 deg and the run optimum: Python-VPP main + jib (`pyvpp_polar_nospi.csv`). The certificate has no non-spinnaker velocity table and its 90 deg speed appears to carry a spinnaker.
- Targets are interpolated linearly in TWA, then in TWS.

## Criteria

- BSP within 5% of target at every point, including the beat and run optima.
- Beat VMG within 5% of the certificate beat VMG.
- Beat angle: not gated (reported only).
- Run angle error is reported only.
- An unsettled sim point, a NaN BSP, a point with no target, or a missing beat or run row is a fail.
- A BSP or beat-angle miss inside an accepted residual is reported as accepted and does not fail.

## Accepted residuals

| TWS kn | Point | Metric | Bound | Used | Reason |
|---|---|---|---|---|---|
| 6 | run | bsp | 6% | yes | owner accepted: 6 kn run +5.1% against Python-VPP, marginal |

## Known differences

- Beat angles are narrower than the certificate's. They are reported, not gated; Python-VPP with the same ORC sail method is also 3 to 9 deg narrower.
- The 90 deg target is Python-VPP main + jib rather than the certificate, whose 90 deg speed appears to carry a spinnaker.

![polar](j80-polar.png)

## Summary

| TWS kn | beat TWA err deg | beat BSP err % | beat VMG err % | run TWA err deg | run BSP err % | result |
|---|---|---|---|---|---|---|
| 6 | -2.7 | -3.7 | +0.4 | -5.0 | +5.1 | pass (accepted) |
| 10 | -1.8 | -0.8 | +1.7 | -2.3 | +3.2 | pass |
| 16 | -4.5 | -2.8 | +2.7 | -1.1 | +1.8 | pass |

## TWS 6 kn

| Point | TWA deg | Target | Target kn | Sim kn | Error % | Pass | Heel deg | Leeway deg | Rudder deg | Main / trav / backstay / jib sheet / jib car |
|---|---|---|---|---|---|---|---|---|---|---|
| beat | 39.8 | ORC certificate | 4.37 | 4.21 | -3.7 | pass | -7.5 | 3.5 | 2.2 | 0.98 / -0.67 / 0.00 / 0.94 / 0.99 |
| fixed | 52.0 | ORC certificate | 4.93 | 4.92 | -0.1 | pass | -7.8 | 3.1 | 2.1 | 0.96 / -0.78 / 0.00 / 0.99 / 0.96 |
| fixed | 60.0 | ORC certificate | 5.22 | 5.10 | -2.3 | pass | -7.6 | 2.7 | 1.8 | 0.93 / -0.82 / 0.00 / 0.99 / 0.53 |
| fixed | 75.0 | ORC certificate | 5.44 | 5.21 | -4.2 | pass | -7.1 | 2.2 | 1.4 | 0.94 / 0.02 / 0.00 / 0.89 / 0.49 |
| fixed | 90.0 | Python-VPP | 5.29 | 5.08 | -4.0 | pass | -6.5 | 1.7 | 1.1 | 0.85 / 0.06 / 0.00 / 0.70 / 0.61 |
| fixed | 110.0 | Python-VPP | 4.75 | 4.61 | -2.9 | pass | -4.0 | 1.1 | 0.8 | 0.62 / -0.12 / 0.00 / 0.58 / 0.63 |
| fixed | 120.0 | Python-VPP | 4.10 | 4.22 | +3.0 | pass | -2.4 | 0.8 | 0.6 | 0.39 / -0.45 / 0.00 / 0.54 / 0.76 |
| fixed | 135.0 | Python-VPP | 3.46 | 3.58 | +3.6 | pass | -1.1 | 0.5 | 0.4 | 0.15 / 0.00 / 0.00 / 0.65 / 0.62 |
| fixed | 150.0 | Python-VPP | 3.06 | 3.10 | +1.2 | pass | -0.6 | 0.5 | 0.4 | 0.10 / 0.98 / 0.00 / 0.24 / 0.06 |
| fixed | 165.0 | Python-VPP | 2.83 | 2.93 | +3.6 | pass | -0.3 | 0.3 | 0.2 | 0.05 / 0.00 / 0.00 / 0.05 / 0.50 |
| fixed | 180.0 | Python-VPP | 2.75 | 2.84 | +3.4 | pass | 0.0 | -0.0 | 0.0 | 0.00 / 0.00 / 0.00 / 0.00 / 0.50 |
| run | 171.0 | Python-VPP | 2.76 | 2.90 | +5.1 | accepted | -0.2 | 0.2 | 0.1 | 0.12 / 0.98 / 0.00 / 0.25 / 0.06 |

Beat angle: sim 39.8, target 42.5 deg (error -2.7 deg). Beat VMG: sim 3.24, target 3.22 kn (error +0.4 %).
Run angle: sim 171.0, target 176.0 deg.

Accepted (run 171.0 deg, bsp): owner accepted: 6 kn run +5.1% against Python-VPP, marginal.

## TWS 10 kn

| Point | TWA deg | Target | Target kn | Sim kn | Error % | Pass | Heel deg | Leeway deg | Rudder deg | Main / trav / backstay / jib sheet / jib car |
|---|---|---|---|---|---|---|---|---|---|---|
| beat | 37.1 | ORC certificate | 5.65 | 5.61 | -0.8 | pass | -13.9 | 4.5 | 1.8 | 0.97 / -0.90 / 0.00 / 0.87 / 0.94 |
| fixed | 52.0 | ORC certificate | 6.23 | 6.19 | -0.6 | pass | -14.8 | 4.0 | 1.4 | 0.93 / -0.89 / 0.00 / 0.95 / 0.63 |
| fixed | 60.0 | ORC certificate | 6.38 | 6.27 | -1.7 | pass | -13.7 | 3.6 | 1.2 | 0.94 / -0.31 / 0.00 / 0.94 / 0.50 |
| fixed | 75.0 | ORC certificate | 6.52 | 6.33 | -3.0 | pass | -11.4 | 2.9 | 1.1 | 0.90 / 0.23 / 0.00 / 0.77 / 0.48 |
| fixed | 90.0 | Python-VPP | 6.27 | 6.28 | +0.2 | pass | -8.8 | 2.2 | 1.0 | 0.76 / 0.20 / 0.00 / 0.62 / 0.46 |
| fixed | 110.0 | Python-VPP | 6.01 | 6.03 | +0.4 | pass | -6.3 | 1.3 | 0.6 | 0.50 / 0.07 / 0.00 / 0.62 / 0.46 |
| fixed | 120.0 | Python-VPP | 5.79 | 5.80 | +0.2 | pass | -5.5 | 0.9 | 0.4 | 0.33 / -0.04 / 0.00 / 0.62 / 0.46 |
| fixed | 135.0 | Python-VPP | 5.31 | 5.33 | +0.3 | pass | -3.2 | 0.7 | 0.5 | 0.06 / 0.00 / 0.00 / 0.72 / 0.01 |
| fixed | 150.0 | Python-VPP | 4.84 | 4.86 | +0.4 | pass | -1.9 | 0.5 | 0.4 | 0.01 / 0.98 / 0.00 / 0.70 / 0.01 |
| fixed | 165.0 | Python-VPP | 4.54 | 4.64 | +2.2 | pass | -1.0 | 0.3 | 0.3 | 0.05 / 0.28 / 0.00 / 0.19 / 0.01 |
| fixed | 180.0 | Python-VPP | 4.42 | 4.53 | +2.5 | pass | 0.0 | 0.0 | 0.0 | 0.00 / 0.28 / 0.00 / 0.14 / 0.01 |
| run | 173.7 | Python-VPP | 4.43 | 4.57 | +3.2 | pass | -0.6 | 0.2 | 0.1 | 0.06 / 0.98 / 0.00 / 0.72 / 0.01 |

Beat angle: sim 37.1, target 38.9 deg (error -1.8 deg). Beat VMG: sim 4.47, target 4.40 kn (error +1.7 %).
Run angle: sim 173.7, target 176.0 deg.

## TWS 16 kn

| Point | TWA deg | Target | Target kn | Sim kn | Error % | Pass | Heel deg | Leeway deg | Rudder deg | Main / trav / backstay / jib sheet / jib car |
|---|---|---|---|---|---|---|---|---|---|---|
| beat | 32.7 | ORC certificate | 5.88 | 5.72 | -2.8 | pass | -21.5 | 5.8 | 1.5 | 0.97 / -0.11 / 0.43 / 0.97 / 0.68 |
| fixed | 52.0 | ORC certificate | 6.49 | 6.39 | -1.5 | pass | -21.4 | 4.8 | 0.6 | 0.87 / 0.33 / 0.11 / 0.94 / 0.50 |
| fixed | 60.0 | ORC certificate | 6.75 | 6.62 | -1.9 | pass | -23.0 | 4.3 | -1.2 | 0.81 / -0.06 / 0.38 / 0.85 / 0.50 |
| fixed | 75.0 | ORC certificate | 7.24 | 6.99 | -3.5 | pass | -20.8 | 3.6 | -1.7 | 0.72 / 0.02 / 0.26 / 0.63 / 0.50 |
| fixed | 90.0 | Python-VPP | 7.13 | 7.39 | +3.6 | pass | -17.3 | 2.7 | -2.0 | 0.67 / 0.24 / 0.04 / 0.62 / 0.55 |
| fixed | 110.0 | Python-VPP | 7.02 | 7.14 | +1.7 | pass | -9.2 | 1.7 | -0.1 | 0.38 / 0.27 / 0.00 / 0.62 / 0.45 |
| fixed | 120.0 | Python-VPP | 6.76 | 6.90 | +2.1 | pass | -7.4 | 1.4 | 0.1 | 0.20 / 0.12 / 0.00 / 0.62 / 0.47 |
| fixed | 135.0 | Python-VPP | 6.46 | 6.56 | +1.5 | pass | -8.5 | 1.0 | -0.0 | 0.08 / 0.53 / 0.00 / 0.62 / 0.06 |
| fixed | 150.0 | Python-VPP | 6.28 | 6.39 | +1.7 | pass | -5.7 | 0.7 | 0.2 | 0.08 / 0.71 / 0.00 / 0.62 / 0.01 |
| fixed | 165.0 | Python-VPP | 6.17 | 6.29 | +2.0 | pass | -3.2 | 0.4 | 0.2 | 0.08 / 0.82 / 0.00 / 0.06 / 0.01 |
| fixed | 180.0 | Python-VPP | 6.12 | 6.22 | +1.7 | pass | 0.0 | -0.0 | 0.0 | 0.10 / 0.96 / 0.00 / 0.70 / 0.01 |
| run | 177.9 | Python-VPP | 6.12 | 6.23 | +1.8 | pass | -1.3 | 0.1 | 0.0 | 0.06 / 0.75 / 0.00 / 0.37 / 0.01 |

Beat angle: sim 32.7, target 37.2 deg (error -4.5 deg). Beat VMG: sim 4.81, target 4.68 kn (error +2.7 %).
Run angle: sim 177.9, target 179.0 deg.

