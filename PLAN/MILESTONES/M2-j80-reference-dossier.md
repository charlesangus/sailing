# M2 - J/80 Reference Dossier

Build the data everything else calibrates against: the J/80's ORC certificate polar and
particulars, rig/sail/appendage/mass data, a hull mesh made by morphing the closest DSYHS
hull to the J/80's particulars, and a Python-VPP main+jib polar. Pure data and tooling — no
sim code. Every number recorded with its source and a confidence.

## Scope

**In:**
- `data/boats/j80/`: particulars, ORC polar(s), rig, sails, appendages, mass, hull meshes, hydrostatics, Python-VPP polar, README.
- `tools/hull/` morph tooling and `tools/fetch_dsyhs.sh`.

**Out:**
- Asymmetric spinnaker and sprit data (out of the demo).
- Any other boat (Alberg 30, YD-41 — out of the demo).
- A hand-modelled visual hull (out of the demo).

**Edge cases:**
- Several J/80 certificates disagree (sail inventory, crew weight) → pick the one closest to class-standard configuration, record why, keep the others' polars for spread.
- No J/80 non-spinnaker certificate found → downwind target is the Python-VPP main+jib polar (record that in the README).
- The ORC certificate omits a hull parameter (e.g. LCB, Cp) → take it from the chosen DSYHS parent hull after morphing and mark it "derived".
- The DSYHS IGES surface stops below J/80 sheer height → extend topsides vertically to the J/80 freeboard.

**Assumptions:**
- Public J/80 ORC certificates with polar tables exist on data.orc.org.
- The DSYHS geometries dataset (4TU.ResearchData, IGES, with standard keel/rudder) is downloadable without registration.
- An affine L/B/T scale of a DSYHS hull is fair-looking enough to stand in for the J/80 visually.

## Phase 2.1: Published data

- [x] M2.P1.T1 — Extract J/80 ORC certificate data
  - files: data/boats/j80/particulars.json, data/boats/j80/orc_polar.csv, data/boats/j80/orc_nospi_polar.csv (if found), data/boats/j80/sources.md
  - approach: Search data.orc.org public certificates for J/80s; choose a class-standard one (standard sails, class crew weight). Transcribe hull particulars (LOA, LWL/IMS L, beam, draft, displacement, righting moment at 1°, LPS / stability index, crew weight) to `particulars.json` as `{value, unit, source, confidence}`. Transcribe the polar table (boat speed by TWS 6–20 kn × TWA incl. beat and run angles) to `orc_polar.csv` (columns tws_kn, twa_deg, bsp_kn). Look for a non-spinnaker J/80 certificate and transcribe it the same way if one exists. Record URLs in `sources.md`.
  - verify: CSV loads in pandas with no gaps; spot-check five values against the PDF; `sources.md` names every certificate used.
  - size: M
- [ ] M2.P1.T2 — Rig, sail, appendage and mass data
  - files: data/boats/j80/rig.json, data/boats/j80/sails.json, data/boats/j80/appendages.json, data/boats/j80/mass.json, data/boats/j80/sources.md
  - approach: From the J/80 class rules, J/Boats spec sheet and the certificate: rig dims (I, J, P, E, mast height, spreaders, boom height), main and jib geometry (luff, foot, leech, roach, area, sheeting points, traveler span, jib car track range), keel (draft, fin chord/span/sweep/section, bulb mass and dims), rudder (span, chord, section, stock position), mass breakdown (lightship, ballast, rig, crew count and total class crew weight). Same `{value, unit, source, confidence}` form; mark estimates "estimated" with the method.
  - verify: Sail areas computed from geometry match published areas within 3%; mass items sum to certificate displacement minus crew within 3%.
  - size: M

## Phase 2.2: Hull geometry

- [ ] M2.P2.T1 — Fetch and tabulate the DSYHS geometries
  - files: tools/fetch_dsyhs.sh, tools/hull/dsyhs_table.py, data/dsyhs/table.csv, .gitignore
  - approach: Script downloads the 4TU DSYHS geometries zip into `data/dsyhs/raw/` (gitignored) with a checksum check. Python (`.venv`, a pip IGES/OCC reader such as `cadquery-ocp`) computes each hull's Lwl, Bwl, Tc, displacement, LCB, Cp, Cm at its design waterline, writes `table.csv` (committed).
  - verify: Table has a row per hull; values for two hulls match the parameters published in the DSYHS literature within 2%.
  - size: M
- [ ] M2.P2.T2 — Morph the closest DSYHS hull to the J/80
  - files: tools/hull/morph.py, data/boats/j80/hull_visual.obj, data/boats/j80/hull_physics.obj, data/boats/j80/hull_hydro.json
  - approach: Pick the hull minimising normalised distance to J/80 in (Cp, LCB, Bwl/Tc, Lwl/∇^1/3), using `table.csv`. Tessellate its IGES surface, mirror to a closed watertight hull with a deck at sheer (extend topsides to J/80 freeboard if short), scale affinely in x/y/z to J/80 LWL/BWL/Tc, then adjust the z scale iteratively until displaced volume (hull + keel/bulb + rudder volumes from `appendages.json`) matches certificate displacement. Emit a visual mesh (~20k tris, smooth normals) and a decimated physics mesh (~1–2k tris, watertight, outward normals) in body-frame metres (Context frames). `hull_hydro.json` records the parent hull ID, scale factors, and resulting Lwl, Bwl, Tc, ∇, LCB, Cp, Cm, waterplane area, wetted area.
  - verify: `morph.py --check` prints hydrostatics vs targets: displacement, LWL, BWL within 2%; physics mesh is watertight (trimesh `is_watertight`) and its volume matches the visual mesh within 1%.
  - size: L
- [ ] M2.P2.T3 — Keel and rudder meshes and foil specs
  - files: tools/hull/appendages.py, data/boats/j80/appendages.obj, data/boats/j80/appendages.json
  - approach: From `appendages.json` planform data generate NACA-section keel fin + bulb and rudder meshes (body frame, watertight, for buoyancy and visuals) and add derived foil specs the physics will use: span, root/tip chord, sweep, section thickness, area, geometric aspect ratio, quarter-chord centre position, rudder stock position.
  - verify: Mesh volumes match the bulb and fin volumes implied by their masses/dims within 5%; foil areas match planform data.
  - size: M

## Phase 2.3: Reference polars

- [ ] M2.P3.T1 — Python-VPP main+jib polar
  - files: tools/vpp/run_pyvpp.py, tools/vpp/requirements.txt, data/boats/j80/pyvpp_polar_nospi.csv, data/boats/j80/pyvpp_polar_upwind_check.md
  - approach: Install Python-VPP (Marin Lauber's open-source implementation) into `.venv`; build its J/80 input from the dossier (hull particulars, appendages, rig, main + jib only, crew); run TWS 6, 8, 10, 12, 14, 16, 20 × TWA 30–180. Write the polar CSV in the same columns as `orc_polar.csv`, and a short comparison of its upwind speeds against the ORC certificate.
  - verify: Script reproduces the CSV; upwind speeds are within ~10% of the certificate (a sanity check that the inputs are right — not the calibration gate).
  - size: M
- [ ] M2.P3.T2 — Dossier README
  - files: data/boats/j80/README.md
  - approach: Summarise every file in the dossier, the certificate chosen and why, the DSYHS parent hull and scale factors, the downwind calibration target (non-spi certificate or Python-VPP — whichever M2.P1.T1 found), the stability targets (RM at 1°, LPS) for M3, and every value marked estimated.
  - verify: Every file in `data/boats/j80/` is listed; the downwind target is stated.
  - size: M

**Verification gate:** `python -I tools/hull/morph.py --check` reports displacement, LWL and BWL within 2% of the certificate; `orc_polar.csv` and `pyvpp_polar_nospi.csv` exist and load; the README names the downwind target and every source.

## Decisions

- 2026-11-27 — J/80 as the sole reference boat: the user chose it over YD-41/Alberg 30; it is well researched (IRENav full-scale trials) with many ORC certificates and a hull inside the DSYHS range.
- 2026-11-27 — Hull mesh is the closest DSYHS hull morphed to J/80 particulars: no open J/80 lines exist (ORC offset files are proprietary); this gives real, fair, tank-tested geometry that hits J/80 hydrostatics.
- 2026-11-27 — Reference certificate is ORC's 2019 J/80 one-design certificate (data.orc.org/public/od/2019/j80.od.pdf): current certificates sit behind a human-verification gate agents cannot pass; the user chose the OD certificate, which is class-standard by construction. Current certificates the user later supplies are added for spread only.
- 2026-11-27 — The OD certificate has no non-spinnaker speed table, so the downwind target is the Python-VPP main+jib polar. Its beat/run rows give VMG; `orc_polar.csv` stores boat speed as VMG/|cos TWA| and `sources.md` keeps the raw VMGs.
