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
