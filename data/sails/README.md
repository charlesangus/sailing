# Sail coefficient data

Maximum lift and parasitic drag coefficients of an optimally trimmed sail
against apparent wind angle, used by the core's sail module
(`core/include/sail/sails.hpp`). A sail in a boat's `sails.json` reads
`orc_<type>.csv`, its type being its optional `type` entry or else its key.

## `orc_main.csv`, `orc_jib.csv`

- Source: Offshore Racing Congress, *ORC VPP Documentation 2023*,
  section 5.2.1, Table 5.1 (mainsail: CLhi, CDhi, CLlow, CDlow and
  kpm = 0.01379) and section 5.2.2, Table 5.4 (genoa: CLhi, CLlow, one CD
  column and kpj = 0.016), pages 40 and 43. The document is copyright ORC,
  all rights reserved; only the numerical coefficients are reproduced here.
- The same numbers, the high sets, are in the MIT-licensed Python-VPP
  (`SailMod.py`, the `Main` and `Jib` tables), which reads them with a
  not-a-knot cubic spline as the core does. Python-VPP adds a jib row at
  0 degrees (Cd 0.003, Cl 0) that ORC does not have; these files keep ORC's
  first row at 7 degrees, held constant below it.
- ORC's Table 5.2 and 5.5 choose the set from the rig: a masthead backstay
  without runners, as on the J/80, gets mainsail L and headsail M (the high
  CL set with low CD, identical here). Python-VPP uses high for both and so
  does the default `high_coefficients = 1` of a sail; both sets are stored so
  the blend can be changed.

### Layout

`key,value` rows, then the header `awa_deg,cl_high,cd_high,cl_low,cd_low`
and one row per tabulated angle, strictly increasing to 180 degrees. Blank
lines are skipped. Keys:

- `role`: `main` or `headsail`, the sail's part in ORC's combined-rig
  method (section 5.4) and blanketing (section 5.6).
- `kp`: ORC's quadratic viscous drag coefficient.
- `aoa_design_deg`, `max_ease_deg`, `luff_aoa_deg`, `stall_width_deg`,
  `normal_force_coefficient`: the angle-of-attack form used when a sail's
  trim gives an explicit angle of attack. The optimum angle of attack at
  apparent wind angle b is max(min(aoa_design, b), b - max_ease); the stalled
  sail tends to a flat plate of normal-force coefficient Cn. These are not
  ORC values. They are provisional estimates: a design angle of 25 degrees
  for the main and 18 for the genoa, a little below the apparent wind angles
  of maximum lift in the tables (28 degrees for the main, 20 to 27 for the
  genoa), a main that eases out to 90 degrees off the centreline (ORC: at
  180 degrees AWA the main is at about 90 degrees of attack), a genoa sheet
  that runs out of ease once lift falls past 50 degrees, and Cn equal to the
  table's drag at 180 degrees. They do not change the optimally trimmed
  coefficients, which are the tables'.

### Other ORC inputs

The sail module hard-codes two more ORC curves, with their sources:
the effective height factor kheff of Figure 5.14 (revised 2023), read off the
plot at 5 degree steps, and the fcdmult table of section 5.4.3. Python-VPP's
`kheff.dat` is the earlier curve, which peaks at 1.22 near 18 degrees
against 1.45 near 20 for the 2023 one.
