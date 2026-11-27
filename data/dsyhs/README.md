# Delft Systematic Yacht Hull Series geometries

`table.csv` holds the upright bare canoe-body hydrostatics of every hull in the
DSYHS geometry dataset, computed from its IGES surfaces by
`tools/hull/dsyhs_table.py`. The raw files are not committed;
`tools/fetch_dsyhs.sh` installs the pinned hull-tool dependencies
(`tools/hull/requirements.txt`) into `.venv`, downloads the files into `raw/` and
checks their SHA-256.

## Sources

- J. den Ouden, *Delft Systematic Yacht Hull Series Geometries data*,
  4TU.ResearchData, 2022. doi:[10.4121/21501330](https://doi.org/10.4121/21501330),
  file `geometriesIGSmodelscale.zip`. Licence CC0 1.0.
- J. den Ouden, *Delft Systematic Yacht Hull Series hydrostatics data*,
  4TU.ResearchData, 2022. doi:[10.4121/21501375](https://doi.org/10.4121/21501375),
  file `DSYHS_hydrostatics_modelscale.xlsx`. Licence CC0 1.0. This is the
  series' own parameter table, produced in Maxsurf Stability with each model
  loaded to its test displacement at its LCB. It supplies Tc, which fixes the
  design waterline, and is the reference for the check below.
- The series itself: J. A. Keuning and U. B. Sonnenberg, *Approximation of the
  hydrodynamic forces on a sailing yacht based on the Delft Systematic Yacht
  Hull Series*, 15th International HISWA Symposium, Amsterdam, 1998.

## What the dataset contains

The zip holds 61 IGES files under `IGS/`:

- 51 base hulls, `SYSSER<nn>_surface.igs`, models 1 to 39, 42 to 50 and 60 to 62.
  Models 40 and 41 are not in the dataset.
- 8 reduced-overhang variants of models 23, 27, 42 and 47: `_05` has half the
  original overhang, `_0` has none (transom at the aft perpendicular). The
  hydrostatics table also lists half and zero overhang rows for model 29, which
  have no geometry files.
- `StandardKeel.igs` and `StandardRudder.igs`, the series' standard appendages,
  in separate files. The hull files have no appendages.

## Geometry conventions

- Model scale, millimetres (IGES global unit `MM`). Lwl is 1.6 m (series 1),
  2.0 m (series 2, 3 and models 49, 50), 1.71 m (models 42 to 48) or about
  2.08 m (series 6).
- Starboard half hulls (y <= 0), open at the centreplane and at the sheer,
  except model 43, which is a full hull; the script detects this and does not
  double its results.
- x forward, with x = 0 at the aft perpendicular, so the forward perpendicular
  sits at x = Lwl (within 12 mm, model scale, across all hulls); z up, with
  z = 0 at the lowest point of the canoe body (within 1 mm).
- The hulls are level in the IGES frame: a horizontal plane at z = Tc gives the
  published Lwl to within 0.6 %, so no trim is applied.

## Design waterline and method

The DWL is the horizontal plane z = zmin + Tc, where zmin is the lowest point
of the tessellated hull and Tc is `tc0` from the hydrostatics table. Lwl, Bwl,
volume, LCB, Aw, Ax and Sc are then computed from the geometry, not copied.

Each IGES surface is tessellated with OpenCascade (`cadquery-ocp`, 0.05 mm
linear deflection) and the triangles are clipped at the DWL. The wetted mesh,
the centreplane and the waterplane bound a closed half body. By the divergence
theorem with fields of the form (0, 0, f(x)(z - zDWL)), which vanish on both
planes, the volume and its longitudinal moment are sums over the wetted
triangles alone, so the surface never has to be capped explicitly. Aw is the
projected area of the wetted mesh on the waterplane. Lwl and Bwl come from the
mesh's intersection with the DWL. Ax is the largest of 400 section areas
evenly spaced along Lwl. Half-hull results are doubled.

## Columns of `table.csv`

All lengths in metres, areas in m², volumes in m³, model scale.

| column | meaning |
| --- | --- |
| `hull_id` | DSYHS model number |
| `overhang` | 1 original, 0.5 half, 0 none, as in the hydrostatics table |
| `file` | source IGES file |
| `z_dwl_m` | z of the DWL in the IGES frame, zmin + Tc |
| `x_fp_m` | x of the forward end of the DWL (IGES frame, x = 0 at the AP) |
| `lwl_m`, `bwl_m` | waterline length and beam |
| `tc_m` | canoe-body draft, from the published table (defines the DWL) |
| `vol_m3` | canoe-body displaced volume ∇c |
| `lcb_aft_fp_m` | LCB, metres aft of the forward end of the DWL |
| `lcb_frac_lwl` | LCB as a fraction of Lwl aft of the forward end of the DWL |
| `aw_m2` | waterplane area |
| `ax_m2` | maximum section area |
| `sc_m2` | canoe-body wetted surface |
| `cp`, `cm`, `cb`, `cw` | ∇c/(Ax Lwl), Ax/(Bwl Tc), ∇c/(Lwl Bwl Tc), Aw/(Lwl Bwl) |
| `lwl_bwl`, `bwl_tc` | Lwl/Bwl, Bwl/Tc |
| `lwl_vol13` | displacement-length ratio Lwl/∇c^(1/3) |

The published table gives LCB in metres from Lwl/2, positive forward. The
fraction from the FP is 0.5 - lcb0/lwl0.

## Check against the published table

`tools/hull/dsyhs_table.py --compare <csv>` writes the computed and published
values side by side for every hull. Against the series' own table
(doi:10.4121/21501375, upright columns), the largest error over all 59 hulls is:

| quantity | max error |
| --- | --- |
| Lwl/Bwl | 0.8 % |
| Bwl/Tc | 1.1 % |
| LCB (fraction of Lwl from FP) | 0.4 % |
| Cp | 1.2 % |
| Lwl/∇c^(1/3) | 1.1 % |
| ∇c | 1.3 % |
| Aw | 1.7 % |
| Cm | 2.1 % |
| Sc | 4.6 % |

Tc is an input, so Bwl/Tc checks Bwl alone. Sc exceeds 2 % only for models 11,
13, 18, 20, 31 and 33; the cause has not been traced.

Three hulls in detail, computed (published) and error:

| quantity | model 1 | model 23 | model 60 |
| --- | --- | --- | --- |
| Lwl/Bwl | 3.135 (3.155) -0.62 % | 3.500 (3.472) +0.78 % | 3.903 (3.905) -0.03 % |
| Bwl/Tc | 4.037 (3.992) +1.12 % | 4.072 (4.091) -0.47 % | 5.758 (5.755) +0.05 % |
| LCB, fraction of Lwl aft of FP | 0.5213 (0.5229) -0.30 % | 0.5199 (0.5185) +0.27 % | 0.5455 (0.5455) 0.00 % |
| Cp | 0.5623 (0.5644) -0.38 % | 0.5451 (0.5473) -0.41 % | 0.5415 (0.5413) +0.04 % |
| Lwl/∇c^(1/3) | 4.800 (4.775) +0.52 % | 5.026 (5.001) +0.51 % | 6.006 (6.006) +0.01 % |
| Cm | 0.638 (0.646) -1.31 % | 0.720 (0.721) -0.02 % | 0.748 (0.749) -0.09 % |
| ∇c, m³ | 0.03758 (0.03761) -0.09 % | 0.06359 (0.06397) -0.59 % | 0.04142 (0.04141) +0.03 % |
| Sc, m² | 0.6558 (0.6425) +2.06 % | 0.9406 (0.9239) +1.80 % | 0.8431 (0.8381) +0.59 % |

Computed Lwl runs from 0.3 % short to 0.7 % long of the published value, which
is a round nominal figure for most hulls (1.6 m, 2.0 m, 1.71 m). The IGES headers
describe the files as Rhino models made after the models were measured in 2012,
so small departures from the nominal lines are expected.

`SYSSER47_0` is not quite closed: its transom at x = 0 sits a few millimetres
forward of where the DWL would end, so the waterline runs out at the transom
and Lwl is 1.704 m against 1.710 m published.
