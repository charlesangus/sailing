# Shared hull resistance data

## `ORCi_Drag_Surfaces.csv`

The ORC canoe-body residuary resistance surfaces: Rr per unit displacement over
Froude number, length-volume ratio and beam-draft ratio, used by the core's
`orc` hull resistance model (`core/include/sail/hull_resistance.hpp`).

- Primary source: ORC VPP Documentation 2023, section 6.3.1, *Canoe body - RR
  surfaces (Fn, BTR, LVR)*
  (https://orc.org/uploads/files/ORC-VPP-Documentation-2023.pdf). ORC publishes
  the surface points as a CSV at www.orc.org/rules.
- This copy: byte for byte from Python-VPP (Marin Lauber,
  https://github.com/marinlauber/Python-VPP), file
  `src/python_vpp/dat/ORCi_Drag_Surfaces.csv` at commit
  `9b9fe8f3f63d902a1df207691109e42932d0a9bb`, the commit
  `tools/vpp/fetch_pyvpp.sh` pins. SHA-256
  `a45ffa9d988f655487c809e4c04deefa72be8d14a194dcf4f49b0171a69bd968`. CRLF
  line endings, kept as shipped.
- Licence: Python-VPP is MIT licensed (below).

### Layout

A note line, then 24 blocks, one per Froude number from 0.125 to 0.7 in steps
of 0.025. Each block is a header row (`"RRmult(lvr_i,btr_j)",Fn=,<Fn>`), a row
of 41 BTR values (2.5 to 9) and 41 rows, one per LVR (3 to 9), each the LVR
followed by its 41 values. Multiply a value by displacement (kg) x 9.81 / 1000
for the resistance in N, as the file's note line says and Python-VPP does.

### How the core uses it

- Trilinear interpolation in (Fn, LVR, BTR), with a zero row added at Fn 0 so
  Rr is zero at zero speed.
- LVR and BTR outside the grid are clamped to its edge ("defaults to that of
  the closest point of the surface", section 6.3.1) and flagged; Fn above 0.7
  is extrapolated linearly from the last two rows and flagged. Python-VPP
  extrapolates linearly in all three axes instead.
- This is only the surface half of ORC's canoe-body Rr. ORC 2023 averages it
  50:50 with a neural-network prediction, which is not implemented.
- The boat dossier's `hull_resistance.json` carries an immersed-rail factor on
  Rr, 1 + 0.0004 (heel_deg - 30)^2 above 30 deg. It is **not** in the ORC 2023
  documentation; it is Python-VPP's, kept because the Python-VPP downwind
  target uses it.

## `dsyhs1998_coefficients.csv`

Coefficients of the Keuning & Sonnenberg 1998 Delft Systematic Yacht Hull
Series bare-hull regressions, used by the core's `dsyhs` hull resistance model.
`dsyhs1998_reference.json` is the same model evaluated independently by
`tools/hull/dsyhs1998_reference.py` from this file, for the core's tests.

### Sources

- Primary: J. A. Keuning and U. B. Sonnenberg, *Approximation of the
  hydrodynamic forces on a sailing yacht based on the 'Delft Systematic Yacht
  Hull Series'*, 15th International HISWA Symposium on Yacht Design and Yacht
  Construction, Amsterdam, 16 November 1998, pp. 99-152 (TU Delft Ship
  Hydromechanics report 1175-P). A companion paper, *Approximation of the calm
  water resistance on a sailing yacht based on the Delft Systematic Yacht Hull
  Series*, appeared at the 14th Chesapeake Sailing Yacht Symposium, Annapolis,
  30 January 1999, SNAME, pp. 181-200. Neither could be obtained (the TU Delft
  repository record is access-restricted), so the tables were **not** read from
  the primary source.
- Transcribed from: A. F. Molland, S. R. Turnock and D. A. Hudson, *Ship
  Resistance and Propulsion*, 2nd ed., Cambridge University Press, 2017,
  section 10.3.6, which reproduces the tables "with permission" from Keuning &
  Sonnenberg 1998:
  - `upright`: Table 10.13, *Coefficients for polynomial: Residuary resistance
    of bare hull*, Equation (10.72), a0..a8 at Fn 0.10 to 0.60 in steps of 0.05.
  - `heel20_x1000`: Table 10.14, *Change in residuary resistance of hull at 20°
    heel (coefficients are multiplied by 1000)*, Equation (10.73), u0..u5 at
    Fn 0.25 to 0.55. Stored as printed; the core divides by 1000.
  - `wetted_heel`: Table 10.26, *Change in wetted surface area with heel*,
    Equation (10.111), s0..s3 at 5 to 35 degrees.
- Cross-checked against two theses that reproduce the same tables: P. Teixeira,
  *Development of a Velocity Prediction Program for Sailing Yachts*, MSc thesis,
  Instituto Superior Técnico, Lisbon (Appendix IV, Tables IV-3 and IV-4), and
  an MSc thesis from the Université de Liège, 2022-2023, *Investigation of the
  benefits of hydrofoils on sailing vessels* (Tables 9, 11 and 12). Three
  entries differ between the sources; the value used agrees with Molland and
  one of the theses each time:

  | entry | Molland | Teixeira | Liège | used |
  | --- | --- | --- | --- | --- |
  | a1, Fn 0.15 | -0.1808 | (not tabulated) | -0.1505 | -0.1808 |
  | u0, Fn 0.40 | -0.8659 | -0.8659 | -0.8629 | -0.8659 |
  | u4, Fn 0.45 | -1.0064 | -0.1006 | -1.0064 | -1.0064 |

### The regressions as implemented

- Upright, Molland eq. (10.72):
  Rr / (∇c ρ g) = a0 + (a1 LCBfpp/Lwl + a2 Cp + a3 ∇c^(2/3)/Aw + a4 Bwl/Lwl
  + a5 ∇c^(2/3)/Sc + a6 LCBfpp/LCFfpp + a7 (LCBfpp/Lwl)^2 + a8 Cp^2) ∇c^(1/3)/Lwl.
  This is the 1998 nine-term form; the later Keuning & Katgert 2008 form
  (a0..a7 with Bwl/Tc and Cm, Fn 0.15 to 0.75) is a different regression with
  different coefficients. The Liège thesis prints the a7 term as (LCB/LCF)^2;
  Molland's (LCBfpp/Lwl)^2 is used, and it is the one consistent with a1 ≈ -a7
  in every row.
- LCBfpp and LCFfpp are distances in m aft of the forward perpendicular (the
  forward end of the waterline), as Molland states. Sc is the upright canoe
  wetted area.
- Heel, eqs. (10.73) and (10.74): ΔRr(20°) / (∇c ρ g) = u0 + u1 Lwl/Bwl
  + u2 Bwl/Tc + u3 (Bwl/Tc)^2 + u4 LCB + u5 LCB^2, with LCB in % of Lwl from
  Lwl/2, positive forward (the series' own convention: its LCB range is -8.2 %
  to 0 %); ΔRr(φ) = 6.0 φ^1.7 ΔRr(20°), φ in radians, which is 1.003 at 20°.
- Wetted area, eq. (10.111): Sc(φ) = Sc(0) (1 + (s0 + s1 Bwl/Tc
  + s2 (Bwl/Tc)^2 + s3 Cm) / 100). Friction uses Sc(φ).
- Friction: ITTC-57, Cf = 0.075 / (log10 Re - 2)^2, Re on 0.7 Lwl, on the canoe
  wetted area, with no form factor (1 + k = 1). Both are the series' own
  convention (Molland section 10.3.6.2; N. Kleijweg, *A Bare Hull Upright
  Trimmed Resistance Prediction for High Performance Sailing Yachts*, MSc
  thesis, TU Delft, 2015, section 2.1.2, citing Keuning & Sonnenberg 1998: form
  factors of Series 1 to 4 are 1.03 to 1.07 but no formulation could be
  derived). `hull_resistance.json` `dsyhs.form_factor` overrides it.
- Between table rows the coefficients are linear in Fn (or heel). Below the
  first row they scale linearly to zero at Fn 0 (or 0°). Above Fn 0.60 the
  upright coefficients are extrapolated linearly from the last two rows and
  `fn_extrapolated` is set; above Fn 0.55 the heel coefficients, and above 35°
  the wetted-area coefficients, are held at the last row and `heel_fn_clamped`
  or `heel_clamped` is set.
- The constant a0 makes Rr slightly negative near Fn 0.10 for every series
  hull (about -2e-4 ∇c ρ g); the residuary resistance is floored at zero.
- Range check (`hull_out_of_range`): Lwl/Bwl 2.73 to 5.00, Bwl/Tc 2.46 to
  19.38, Lwl/∇c^(1/3) 4.34 to 8.50, LCB -8.2 % to 0 %, LCF -9.5 % to -1.8 %,
  Cp 0.52 to 0.60, Cm 0.65 to 0.78, Aw/∇c^(2/3) 3.78 to 12.67, as tabulated
  (attributed to Keuning & Sonnenberg 1998) by Teixeira, Table 1, and
  identically by the Liège thesis, Table 8.

### Reference hulls

`dsyhs1998_reference.json` covers DSYHS model 1 (the Series 1 parent) and
model 44 (the Series 4 parent), at model scale from `data/dsyhs/table.csv`, in
fresh water at 15 °C (ITTC 7.5-02-01-03: ρ 999.1 kg/m³, ν 1.1386e-6 m²/s), and
the J/80. `table.csv` has no LCF, so LCBfpp/LCFfpp is the published series
value from Keuning & Katgert 2008 as reproduced by Kleijweg 2015, Table B.1
(0.980 for model 1, 0.947 for model 44). Model 1's computed Cm, 0.638, is just
below the range (published 0.646), so it is flagged out of range; model 44 is
inside it. No printed Rr for an example hull was available to check against.

### Python-VPP licence

```
MIT License

Copyright (c) 2020 Marin Lauber

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
