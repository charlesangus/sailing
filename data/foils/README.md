# Foil section data

## `naca0012.csv`

Section lift and drag coefficients of the NACA 0012 at a Reynolds number of
2,000,000, from 0 to 180 degrees angle of attack, used by the core's foil
module (`core/include/sail/foil.hpp`).

- Source: R. E. Sheldahl and P. C. Klimas, *Aerodynamic Characteristics of
  Seven Symmetrical Airfoil Sections Through 180-Degree Angle of Attack for
  Use in Aerodynamic Analysis of Vertical Axis Wind Turbines*, Sandia National
  Laboratories, SAND80-2114, March 1981. Table 2, *Lift and Drag Coefficients
  for the NACA-0012 Airfoil*, the Re = 2000000.0 block (report pages 22 and
  23). Public domain, a US Government report.
- Copy read: the OSTI scan hosted by the UNT Digital Library,
  https://digital.library.unt.edu/ark:/67531/metadc1191254/m2/1/high_res_d/6548367.pdf
  (OSTI ID 6548367), SHA-256
  `af5cb8e9f564121155014b7c88d3b41ef75cd0655bcdc41690cd4f5b6b38bb97`. The
  scan's OCR text layer is noisy, so each value was checked against the
  other Reynolds-number blocks, which repeat the same post-stall numbers (from
  27 degrees up in the Re = 1e6 block, from 18 degrees up for Cd in the
  Re = 1e4 block). Values whose OCR was unreadable or mis-read (a 9 read as
  3) were taken from those duplicates: Cd at 22 degrees, Cl at 30, 135, 140
  and 175 degrees, and the 145 degree row label. Cd at 11 degrees reads
  `.011+0` in the OCR and is taken as 0.0140, between its neighbours.

### Reynolds number

Re = V c / nu. The J/80 keel (mean chord about 0.9 m) at 2 to 3 m/s and the
rudder (0.3 m) at 4 to 6 m/s in water at nu = 1.16e-6 m^2/s are both near
1e6 to 2.5e6, so the 2e6 block is the closest single one. The report's
attached-flow values (0 to 16 degrees at this Re) come from Eppler's
airfoil code, not from tests; past stall the table is the report's
synthesis of wind-tunnel data, independent of Re.

### Layout

A header row `alpha_deg,cl,cd`, then one row per tabulated angle, 0 to 180
degrees, strictly increasing, exactly as the report lists them (1 degree
steps to 27, then 30 and 5 degree steps). Cl at 0 and 180 degrees is zero.

### How the core uses it

- The table is mirrored to negative angles with Cl odd and Cd even, so the
  section is exactly symmetric, and interpolated linearly in angle.
- The stall angle is the first local maximum of Cl above 0 degrees (13
  degrees here, Cl 1.2169); the reverse-flow stall is the first local extreme
  of Cl below 180 degrees (170 degrees, Cl -0.85).
- The section lift slope is the least-squares slope through the origin over
  the points up to half the stall angle (0.11 per degree, 6.30 per radian).
