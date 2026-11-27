# J/80 reference dossier

Geometry, mass, rig, sail and performance data for a generic (class-standard) J/80, with the tools that produce or check it. `sources.md` has the full provenance, conflict resolutions and method notes; this file is the index and the working summary.

## File index

| File | Holds | Units / frame | Produced by |
|---|---|---|---|
| `sources.md` | Provenance, conflicts, method notes for every file | prose | by hand |
| `particulars.json` | ORC 2019 certificate hull, stability, crew, rig and sail particulars | SI; certificate units kept in `note` | transcribed by hand |
| `orc_polar.csv` | Certificate velocity prediction table (`tws_kn,twa_deg,bsp_kn,kind`; `fixed`, `beat`, `run` rows) | kn, deg | transcribed by hand; `beat`/`run` boat speed derived from VMG |
| `mass.json` | Mass items, CG positions, keel ballast and bulb split, displacement basis | kg, m; provisional frame | transcribed by hand (with estimates) |
| `rig.json` | Mast, forestay, spreaders, shroud chainplates | m, deg; provisional frame | transcribed by hand |
| `sails.json` | Main and jib measurements, corner points, tracks | m, m^2; provisional frame | transcribed by hand |
| `appendages.json` | H.1 keel and rudder offsets, root and ballast positions, keel root fairing; plus a `foils` block | offsets and positions in the provisional frame; `foils` in the body frame | offsets by hand; `foils` block written by `tools/hull/appendages.py` |
| `hull_hydro.json` | Hull targets, parent hull, scale factors, sheer, flare, hydrostatic results, frame transform | m, m^2, m^3; body frame | `tools/hull/morph.py` |
| `hull_visual.obj` | Render mesh of the morphed hull (normals split at creases, not watertight) | m; body frame | `tools/hull/morph.py` |
| `hull_physics.obj` | Watertight, outward-wound decimated hull mesh for hydrostatics | m; body frame | `tools/hull/morph.py` |
| `appendages.obj` | Keel and rudder solids (objects `keel`, `rudder`), each watertight | m; body frame | `tools/hull/appendages.py` |
| `pyvpp_polar_nospi.csv` | Python-VPP main + jib polar, same columns as `orc_polar.csv` | kn, deg | `tools/vpp/run_pyvpp.py` |
| `pyvpp_polar_upwind_check.md` | Upwind comparison with the certificate and the full VPP input list | prose | `tools/vpp/run_pyvpp.py` |

The DSYHS hull table the morph reads is `data/dsyhs/table.csv`; see `data/dsyhs/README.md`.

## Frames

Body frame (meshes and the `foils` block of `appendages.json`): x forward, y to port, z up, metres. Origin on the centreline at the midpoint of LWL on the DWL. The DWL is the certificate's measurement-trim flotation plane, taken as level.

Provisional frame (`rig.json`, `sails.json`, `mass.json` and everything in `appendages.json` outside `foils`): x forward, y to port, z up, metres. x = 0 at the stem (forward end of LOA, which is also the forestay deck point since SFJ = 0), so points aft of the stem have negative x. z = 0 on the same flotation plane. Positions assume a vertical mast; the 3.2 deg rake is recorded separately as `mast_rake`.

Transform, from `hull_hydro.json` `transform`: `p_body = p_provisional + (stem_x_in_body, 0, flotation_z_in_body)` with `stem_x_in_body` = 3.7871 m and `flotation_z_in_body` = 0.0 m.

## Reference certificate

The reference is ORC's 2019 J/80 International One Design certificate (https://data.orc.org/public/od/2019/j80.od.pdf), chosen because it is class-standard by construction: one-design hull, rig and sail inventory, so it describes a generic J/80 rather than one boat's setup. Current J/80 certificates sit behind a human-verification gate and were not used. The PDF has no text layer, so values were transcribed from the rendered pages. In `orc_polar.csv` the `beat` and `run` rows give boat speed derived from the certificate's VMG: `VMG / cos(beat angle)` and `Run VMG / -cos(gybe angle)`.

## Hull

- Parent: DSYHS model 61 (series 6, distance 0.1535); runner-up model 30 (0.202).
- Scale factors about the parent's DWL: sx 3.22802, sy 3.36839, sz 2.90137, iterated on the final mesh until LWL, BWL and canoe volume match.
- LWL 6.71 m, from the J/Boats tech specs (the certificate has only IMSL 7.237 m, RL 7.210 m, LSM0 6.967 m).
- BWL 1.8814 m, derived from the certificate sink (8.99 kg/mm gives a waterplane area of 8.771 m^2) divided by LWL and the parent's Cw 0.6947. Tc 0.2425 m, derived from canoe volume and the parent's Cb 0.4288.
- Displacement 1438 kg at 1025 kg/m^3 is 1.40293 m^3: canoe body 1.30675, keel 0.0805, keel root fairing 0.00612, rudder 0.00956 m^3.
- Overhangs are stretched beyond the waterline ends only, so the stem and transom land at LOA 8.032 m with the certificate LCF from stem (4.276 m). The sheer is a quadratic through the certificate freeboards (0.877 m forward, 0.744 m at the mast, 0.627 m aft); topsides are extended up to 0.2548 m forward and trimmed up to 5.7 mm aft; the deck is flat at the sheer.
- Topside flare to the certificate beam 2.488 m (from 2.212 m), k0 = 1.12505, peaking 4.712 m from the stem, by straightening each section above its DWL vertex. The immersed body is untouched: canoe volume, BWL and LWL are unchanged, waterplane area changes by +2e-5 relative. No section gains an inflection and the DWL kink is 0 deg.
- Keel root fairing preserving the 1.501 m draft: the keel keeps its H.1 root depth (-0.331 m) and span (1.170 m); the 0.089-0.103 m gap to the morphed canoe body is filled by extruding Section 1 vertically (0.00612 m^3 in the morph, 0.00603 m^3 in the mesh).

## Downwind calibration target

The certificate has no non-spinnaker velocity table (only non-spinnaker time allowances), so the downwind main + jib target is the Python-VPP polar `pyvpp_polar_nospi.csv` (Python-VPP commit 9b9fe8f, 1809 kg sailing condition with 350 kg crew, TWS 6-20 kn, TWA 20-180 deg). Known limitations:

- Beat angles are 3-9 deg lower than the certificate from 8 kn up (29-34 deg against 37-39 deg at 10-20 kn); VMG is +2% to +6% against the certificate there.
- The jib is never blanketed behind the main, so the best run angles come out at 176-179 deg.
- The crew moment in the vendored code is replaced in `run_pyvpp.py` by ORC VPP Documentation 2023 eq. 4.30, in SI. Half the maximum beam as the rail arm slightly overstates it.
- The keel volume formula ignores thickness and the rudder aspect ratio is doubled.

Upwind sanity: every upwind point is within 10% of the certificate, and beat VMG is within 6.4% (+1.7% to +6.4%). The table is in `pyvpp_polar_upwind_check.md`.

## Stability targets for the hydrostatics work

From `particulars.json` (certificate p.2):

- RM at 1 deg of heel: 27.0 kg m measured, which is 264.8 N m (the default RM is 27.7 kg m, 271.6 N m).
- Limit of positive stability (LPS) 111.7 deg.
- Stability index 104.9.
- VCGM -0.076 m (VCGD -0.15 m).

Open issue: matching the certificate RM at 1 deg on the morphed mesh at the sailing displacement needs VCG +0.067 m above the flotation plane, against the certificate's VCGM of -0.076 m. The 0.143 m difference suggests the morphed hull has too little transverse waterplane inertia, that is, low form stability.

## Estimated and derived values

Every entry marked `estimated` or `derived` in the JSON files, 74 in total, generated from the `confidence` fields. Methods are shortened to their first clause; the files carry the full text. `particulars.json` has none; the `beat` and `run` rows of `orc_polar.csv` are also derived (see above).

`appendages.json`

- `keel.material_density` (estimated): lead 11 340 and antimony 6 690 kg/m^3 mixed at about 4% Sb
- `keel.volume_from_offsets` (estimated): section area 0.6851 * chord * thickness (NACA 4-digit), integrated trapezoidally over depth, plus an elliptical tip cap
- `keel.lead_top_depth_below_root` (estimated): depth below Section 1 at which the lead volume from the tip up equals 0.0577 m^3 (635 kg / 11 000 kg/m^3)
- `keel.ballast_centroid` (estimated): centroid of the lead portion, from the lead top depth to the tip
- `keel.root_z` (estimated): z of Section 1 = -draft (1.501 m) + span (1.170 m)
- `keel.root_fairing` (estimated): fills the gap between H.1 Section 1 and the morphed canoe body
- `keel.root_leading_edge` (estimated): No public drawing locates the keel fore and aft
- `keel.centroid` (estimated): volume centroid of the offsets: 0.645 m below the root and 0.618 m aft of the root LE
- `rudder.stock_axis` (estimated): pivot on the pintle line at the vertical leading edge, assumed 0.05 m aft of the transom (LOA 8.032 m). z is the root (Section 1), assumed at the waterplane where the ...
- `rudder.tip_z` (estimated): root_z minus Section 5 depth
- `foils.keel.span` (estimated): from the canoe body on the centreline (mean over the root chord) down to the keel tip
- `foils.keel.quarter_chord_sweep` (derived): least-squares slope of the mesh quarter-chord line over the fin, Section 1 to Section 4 (above the bulb)
- `foils.keel.planform_area` (estimated): chord of horizontal mesh sections integrated from the canoe body to the tip
- `foils.keel.geometric_aspect_ratio` (estimated): span^2 / planform_area
- `foils.keel.effective_aspect_ratio` (estimated): 2x geometric: the fin is attached to the hull with no gap, so the hull bottom acts as a reflection plane
- `foils.keel.quarter_chord_centre` (estimated): chord-weighted mean of the quarter-chord points of horizontal mesh sections over the span, i.e. the quarter-chord point at the planform area centroid's depth
- `foils.keel.wetted_area` (estimated): area of the mesh triangles whose centroid is below hull_visual.obj
- `foils.keel.volume_below_hull` (estimated): mesh volume minus its intersection with hull_physics.obj (manifold3d boolean): fin, bulb, tip and the root fairing below the canoe body
- `foils.keel.root_overlap` (derived): the root fairing extends 0.020 m into the canoe body at every root vertex, so the overlap is 0.020 m times the Section 1 area (0.0657 m^2)
- `foils.keel.root_fairing_height` (derived): range over the Section 1 outline of the height from root_z up to hull_visual.obj
- `foils.rudder.span` (estimated): from the root at the flotation plane (appendages.json stock_axis z) down to Section 5
- `foils.rudder.quarter_chord_sweep` (derived): least-squares slope of the mesh quarter-chord line over the whole span
- `foils.rudder.planform_area` (estimated): chord of horizontal mesh sections integrated from the root to Section 5
- `foils.rudder.geometric_aspect_ratio` (estimated): span^2 / planform_area
- `foils.rudder.effective_aspect_ratio` (estimated): 1x geometric: transom-hung, so the root sits clear of the hull with no end plate
- `foils.rudder.quarter_chord_centre` (estimated): chord-weighted mean of the quarter-chord points of horizontal mesh sections over the span, i.e. the quarter-chord point at the planform area centroid's depth
- `foils.rudder.wetted_area` (estimated): area of the mesh triangles below the flotation plane (the root face at z = 0 is excluded)
- `foils.rudder.volume_below_hull` (estimated): mesh volume
- `foils.rudder.stock_position` (estimated): appendages.json rudder stock_axis converted to the body frame: on the vertical leading edge at the root
- `foils.rudder.stock_axis_direction` (estimated): vertical, along the leading edge, which H.1 gives as vertical

`hull_hydro.json`

- `transform` (derived): Stem at the morphed hull's LCF plus the certificate LCF from stem on CL (4.276 m)
- `flare` (estimated): k0 solved by bisection for max beam 2.488 m (ORC J/80 OD certificate 2019, p.1)
- `targets.bwl` (derived): Aw / (Cw Lwl): waterplane area from the certificate sink, 8.99 kg/mm / rho = 8.771 m^2, with the parent's Cw 0.6947
- `targets.tc` (derived): canoe volume / (Cb Lwl Bwl) with the parent's Cb 0.4288
- `targets.keel_volume` (estimated): appendages.json keel volume_from_offsets
- `targets.keel_fairing_volume` (estimated): appendages.json keel root_fairing: section area 0.0658 m^2 times the mean height from root_z up to the canoe body on the centreline over the root chord (0.0930 m)
- `targets.rudder_volume` (estimated): NACA 4-digit section area 0.6851 c t integrated trapezoidally over Sections 1-5
- `targets.canoe_volume` (derived): displacement volume - keel - keel fairing - rudder
- `targets.waterplane_area` (derived): sink kg/mm x 1000 / rho
- `targets.lcb` (derived): no method recorded (value read from the parent hull table)
- `targets.cp` (derived): no method recorded (value read from the parent hull table)

`mass.json`

- `items.hull_deck_fittings` (estimated): residual: builder's weight mid-range (1437.5 kg) minus keel, rudder, mast, boom, standing and running rigging below
- `items.standing_rigging_and_furler` (estimated): wire lengths from rig geometry times 1x19 stainless mass per metre (4.7 mm 0.113, 4.0 mm 0.080, 3.0 mm 0.045 kg/m), about 6 kg, plus turnbuckles and the furler, about 5 ...
- `items.battery` (estimated): midpoint of the class range, 2 to 25 kg
- `items.outboard_engine` (estimated): class minimum (empty of fuel)
- `items.running_rigging` (estimated): sheets, halyards and control lines
- `keel_lead_volume` (estimated): keel_ballast 635 kg / 11 000 kg/m^3
- `keel_sump_volume` (estimated): offsets volume (0.0805 m^3) minus keel_lead_volume
- `bulb_mass` (estimated): offsets volume below Section 4 (0.0292 m^3) x 11 000 kg/m^3, assuming the lead fills from the tip up
- `ballast_ratio` (estimated): keel_ballast / certificate displacement (1438 kg), which hydrostatics use
- `crew_count` (estimated): the class sets no crew number

`rig.json`

- `forestay_deck_point` (estimated): SFJ = 0 puts the forestay at the stem. z extrapolated linearly along the sheer from FFM (0.877 m at SFFP 0.177 m) to the freeboard at mast (0.744 m at J)
- `forestay_length` (estimated): distance from forestay_deck_point to hounds_forestay_on_mast
- `forestay_angle_from_vertical` (estimated): from forestay_deck_point and hounds_forestay_on_mast
- `mast_top_above_sheer` (estimated): scaled from both drawings: J/Boats 10.61 m, ORC p.3 10.59 m
- `mast_top_above_water` (estimated): mast_top_above_sheer + freeboard at mast, measurement trim
- `mast_top_above_deck` (estimated): mast_top_above_sheer minus the cabin-top height at the mast partners (0.29 m above sheer), both scaled from the J/Boats drawing
- `mast_rake` (estimated): slope of the mast aft face between 300 and 1000 px on the J/Boats profile drawing, relative to its waterline
- `spreader_lower_height_above_sheer` (estimated): scaled from the J/Boats rig profile using P = 9.144 m between the bands
- `spreader_upper_height_above_sheer` (estimated): as spreader_lower_height_above_sheer
- `shroud_chainplate_base` (estimated): x: cap shroud meets the deck about 0.14 m aft of the mast aft face on the J/Boats profile. y: assumed just inboard of the 1.24 m half-beam at sheer

`sails.json`

- `main.tack` (estimated): aft face of mast (x = -(J + MDL1)) at boom band height (freeboard at mast + BAS)
- `main.head` (estimated): tack + P straight up the aft face
- `main.clew` (estimated): tack + E aft along a horizontal boom
- `main.sheet_point_on_boom` (estimated): mainsheet strop position scaled from the J/Boats profile (3.03 m aft of the mast aft face), on the underside of the boom
- `main.traveler_centre` (estimated): x: mean of the plan drawing (-6.25 m) and the vertical mainsheet on the profile (-6.05 m). z: lower mainsheet block on the profile
- `main.traveler_span` (estimated): track length across the cockpit scaled from the J/Boats deck plan
- `jib.tack` (estimated): at the forestay deck point
- `jib.head` (estimated): tack + HLU along the line from the forestay deck point to the hounds
- `jib.clew` (estimated): scaled from the ORC p.3 sail plan at 110.9 px/m
- `jib.foot` (estimated): distance tack to clew from the positions above
- `jib.leech` (estimated): distance head to clew from the positions above
- `jib.track_fwd_end` (estimated): the slot drawn on the cabin top spans x -3.87 to -3.34 m (0.53 m) in the J/Boats plan
- `jib.track_aft_end` (estimated): as track_fwd_end

## Open questions

- Whether the real J/80 canoe body reaches the class keel-root depth. The morphed canoe body is 0.09-0.10 m shallower than the H.1 root (no DSYHS hull is fine enough to reach it at this LWL, BWL and volume), and the fairing bridges it. A hull drawing would settle it.
- The canoe-body wetted area is 9.43 m^2 against 9.13 m^2 from the certificate wetted surface less twice the keel and rudder lateral areas; some DSYHS hulls show a similar wetted-surface gap.
- The bow sheer-extension corner: the sheer extension forward (up to 0.2548 m) is a modelling choice, not a measured line.
- Keel fore-and-aft position, spreader length and sweep, rudder tip and root height, and a measured bulb mass are in no public source.

## Sources

See `sources.md` for the full list: the ORC certificate and rules, the 2024 and 2006 class rules (Appendix H.1 offsets), the building specification, the J/Boats pages and drawings, the DSYHS datasets and Python-VPP.

## Regenerating

From the repository root, with `.venv` set up (`tools/bootstrap.sh`):

1. `tools/fetch_dsyhs.sh` downloads and checks the DSYHS geometries into `data/dsyhs/raw/`.
2. `.venv/bin/python -I tools/hull/morph.py` writes `hull_visual.obj`, `hull_physics.obj` and `hull_hydro.json`.
3. `.venv/bin/python -I tools/hull/appendages.py` writes `appendages.obj` and the `foils` block of `appendages.json`.
4. `tools/vpp/fetch_pyvpp.sh`, then `.venv/bin/python -I tools/vpp/run_pyvpp.py` (about 3 minutes) writes `pyvpp_polar_nospi.csv` and `pyvpp_polar_upwind_check.md`.
5. `.venv/bin/python -I tools/dossier_checks.py` checks the transcribed data for consistency.

Verification: `.venv/bin/python -I tools/hull/morph.py --check` and `.venv/bin/python -I tools/hull/appendages.py --check` recompute from the written meshes and fail on any miss. `data/dsyhs/table.csv` is rebuilt by `tools/hull/dsyhs_table.py`.
