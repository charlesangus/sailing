# J/80 dossier sources

## ORC certificates

### ORC J/80 OD certificate 2019

- URL: https://data.orc.org/public/od/2019/j80.od.pdf
- Type: ORC International One Design certificate, 2019 (Test), offset file `j80.od`, issued 11/04/2019, VPP version 2019 1.01. Includes the 2019 IMS Measurement Certificate (p.2) and the sail plan appendix (p.3).
- It is marked "Invalid for Racing" because it is a one-design reference certificate, not one issued to a particular boat.
- Why chosen: it is class-standard by construction (one-design hull, rig and sail inventory, measurement by DSV 02/09/2002), so it describes a generic J/80 rather than one boat's optimised setup. It is also publicly downloadable.
- Current J/80 certificates exist on data.orc.org, but they sit behind a human-verification gate and were not used.
- The PDF is image-only with no text layer. All values were transcribed by reading the rendered pages.

Every number in `particulars.json` and `orc_polar.csv` is cited as "ORC J/80 OD certificate 2019, p.N".

### Files derived from it

- `particulars.json`: hull, stability, crew, rig and sail particulars. Values are in SI. Where the certificate used another unit (righting moment in kg*m), the original appears in `note`. Displacement is printed as `1.438kg`, with `.` as a thousands separator, so it is 1438 kg.
- `orc_polar.csv`: the p.1 "Velocity Prediction in Knots for True Wind Speeds" table. TWS is 6, 8, 10, 12, 14, 16 and 20 kn. The `fixed` rows are TWA 52, 60, 75, 90, 110, 120, 135 and 150 deg.
  - `beat` rows: the TWA is the certificate's Beat Angle. The certificate gives Beat VMG, not boat speed. `bsp_kn` is derived as `VMG / cos(beat angle)` and rounded to 0.01 kn.
  - `run` rows: the TWA is the certificate's Gybe Angle. `bsp_kn` is derived as `Run VMG / -cos(gybe angle)` and rounded to 0.01 kn.
  - The original VMGs are: beat 3.22, 3.91, 4.40, 4.58, 4.64, 4.68, 4.71; run 3.66, 4.59, 5.29, 5.71, 6.16, 6.59, 8.38 (6 to 20 kn).
  - The 14 kn gybe angle is printed as 176.3 deg, out of line with its neighbours (150.7 and 144.2). It is clearly legible and was transcribed as printed.

### Non-spinnaker

No non-spinnaker J/80 velocity table or separate non-spinnaker certificate was found. The 2019 OD certificate only gives a non-spinnaker time allowance per wind speed (p.1 "Non Spinnaker" row, s/nm: 982.5, 789.6, 685.9, 625.3, 587.5, 561.7, 525.7) and Non Spin GPH/OSN ratings (707.5 / 686.0). There is no boat-speed table. So there is no `orc_nospi_polar.csv`. The downwind main+jib target falls back to a Python-VPP main+jib polar.

### Certificate oddities, transcribed as printed

- Crew Weight Default is 411 kg, which is above the Maximum of 350 kg. The Minimum of 263 kg applies only when the NoR and SI invoke it.
- There is no plain LWL. The certificate gives IMSL 7.237 m, RL 7.210 m and LSM0 6.967 m. `ims_l` stands in for waterline length.

### Not on certificate

- LWL, as a plain static waterline length
- LCB / LCG
- Prismatic coefficient (Cp)
- Midship section coefficient (Cm) and waterplane area
- Keel and rudder geometry (span, chord, area, thickness)
- Hull offsets or lines
