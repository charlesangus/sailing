# J/80 Python-VPP main + jib polar: upwind check against the ORC certificate

Written by `tools/vpp/run_pyvpp.py`. ORC values are the 2019 OD certificate (`orc_polar.csv`), which is with spinnaker; upwind the spinnaker plays no part, so the beat and 52/60 deg speeds are comparable. The ORC beat boat speed is derived as Beat VMG / cos(beat angle) (see `sources.md`), so the VMG columns are the like-for-like comparison of the certificate's printed number.

Differences are Python-VPP minus ORC, relative to ORC. The Python-VPP beat angle is the VMG optimum on a 1 deg TWA grid.

| TWS kn | ORC beat deg | VPP beat deg | ORC VMG kn | VPP VMG kn | VMG diff | ORC beat bsp | VPP beat bsp | bsp diff | ORC 52 | VPP 52 | 52 diff | ORC 60 | VPP 60 | 60 diff |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 6 | 42.5 | 44 | 3.22 | 3.30 | +2.5% | 4.37 | 4.59 | +5.1% | 4.93 | 5.04 | +2.2% | 5.22 | 5.26 | +0.7% |
| 8 | 41.2 | 38 | 3.91 | 4.12 | +5.2% | 5.20 | 5.22 | +0.5% | 5.81 | 5.76 | -0.8% | 6.02 | 5.88 | -2.3% |
| 10 | 38.9 | 34 | 4.40 | 4.47 | +1.7% | 5.65 | 5.39 | -4.6% | 6.23 | 6.03 | -3.2% | 6.38 | 6.16 | -3.4% |
| 12 | 37.8 | 33 | 4.58 | 4.67 | +1.8% | 5.80 | 5.56 | -4.1% | 6.39 | 6.17 | -3.4% | 6.59 | 6.30 | -4.5% |
| 14 | 37.5 | 32 | 4.64 | 4.82 | +3.8% | 5.85 | 5.68 | -2.9% | 6.46 | 6.27 | -2.9% | 6.69 | 6.41 | -4.2% |
| 16 | 37.2 | 31 | 4.68 | 4.89 | +4.5% | 5.88 | 5.71 | -2.9% | 6.49 | 6.35 | -2.1% | 6.75 | 6.50 | -3.7% |
| 20 | 38.1 | 29 | 4.71 | 5.01 | +6.4% | 5.99 | 5.73 | -4.3% | 6.60 | 6.46 | -2.1% | 6.90 | 6.62 | -4.0% |

## Inputs

Python-VPP 9b9fe8f (github.com/marinlauber/Python-VPP, MIT). Sailing displacement 1809 kg = boat 1438 + crew 350 (class maximum) + mainsail 8 + jib 5 + running rigging 8; it floats 40.9 mm below the certificate flotation plane.

| Python-VPP input | value | from |
|---|---|---|
| Lwl | 7.146 m | hull mesh at sailing displacement |
| Vol | 1.6701 m^3 | canoe body below that waterline |
| Bwl | 1.929 m | hull mesh |
| Tc | 0.284 m | hull mesh |
| WSA | 10.455 m^2 | canoe body wetted area, hull mesh |
| Amax | 0.4337 m^2 | max section area, hull mesh |
| Tmax | 1.542 m | certificate draft + sinkage |
| Mass | 1809 kg | sailing displacement |
| Loa, Boa | 8.032, 2.488 m | certificate |
| Ff, Fa | 0.836, 0.586 m | certificate FFM, FAM - sinkage |
| crew_weight | 350 kg | class maximum (certificate default 411 kg exceeds it) |
| Keel Cu, Cl, Span, t/c | 0.960, 0.793, 1.263 m, 0.126 | appendages.json foils; Cl keeps the planform area; t/c is thickness over chord integrated over depth |
| Rudder Cu, Cl, Span, t/c | 0.340, 0.295, 1.145 m, 0.120 | appendages.json foils; Cl keeps the planform area |
| Main P, E, Roach, BAD | 9.144, 3.810 m, 0.2004, 1.300 m | rig.json; roach from the measured area; Python-VPP area 20.91 m^2 (dossier 20.91) |
| Jib I, J, LPG, HBI | 9.563, 2.895, 2.950, 0.703 m | IG, J, HLP, freeboard at mast - sinkage; Python-VPP area 14.11 m^2 (dossier 13.98) |

Righting arm: the hull and appendage meshes heeled at level trim, at the sailing displacement, with G at 0.281 m. The boat's own VCG, +0.067 m, is the one at which the meshes reproduce the certificate's measured RM of 27.0 kg m per degree at 1438 kg (the meshes give GZ(1 deg) = 0.0199 m with G on the flotation plane). Crew CG 0.913 m, mainsail 5.092 m, jib 3.974 m and running rigging 3.954 m above the flotation plane.

| heel deg | 0 | 5 | 10 | 15 | 20 | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| GZ m | 0.000 | 0.067 | 0.130 | 0.187 | 0.235 | 0.274 | 0.307 | 0.334 | 0.345 | 0.344 | 0.333 | 0.314 | 0.289 |

## Crew righting moment in Python-VPP

Python-VPP's own crew moment is `0.8 bmax (cw + 0.7 bmax 89) cos(heel)` in kg m, summed with N m righting moments. `run_pyvpp.py` replaces it with ORC VPP Documentation 2023 eq. 4.30 (section 4.4.3): the crew count is CW / 89 kg, all but two crew sit on the rail at half the beam, and the helm and trimmer sit at 70% of that arm, times g and cos(heel). Beam is the certificate's 2.488 m. At 20 deg heel that is 3401 N m. The ramp over 2.5-7.5 deg of heel is Python-VPP's.

## Known limitations

- Crew moment: replaced with ORC eq. 4.30 in SI (see above). Half the maximum beam is used as the rail arm, which slightly overstates it, since the deck-edge half-beam at the crew station is a little less.
- Beat angle: Python-VPP beats 3-9 deg lower than the certificate from 8 kn up (29-34 deg against 37-39 deg at 10-20 kn), and 1.5 deg higher at 6 kn. The beat optimum is inside the 20-180 deg grid at every TWS, so none is grid-limited, but the lower angle makes the beat VMG +2% to +6% against the certificate at 8-20 kn.
- Python-VPP keeps the jib drawing at every TWA with no blanketing behind the main, so the best run angles come out at 176-179 deg. This polar is the downwind calibration target.
- Its keel volume formula ignores thickness.
- It doubles the rudder aspect ratio.
- The VCG that matches the certificate's RM at 1 deg on the morphed mesh is +0.067 m, against the certificate's VCGM of -0.076 m. This suggests the morphed hull's waterplane inertia is low; it is for the hydrostatics milestone.
