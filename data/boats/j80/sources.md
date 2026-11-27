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

## Class rules, builder specifications and manufacturer data

### International J/80 Class Rules 2024

- URL: https://j80.org/images/pdf/J80_CR_2024-06June-01-2.pdf (linked from https://j80.org/class-rules)
- Type: International J/80 Class Association class rules, effective 1 June 2024. This is the current version.
- Used for: mast and boom weights and band distances (C.8.1, C.8.2), crew weight limit (C.4.1), complete-boat minimum weight (C.6.1), builder's weight range (D.5.1), rudder weight (E.2.6), keel and rudder material and fittings (E.2), and mainsail and jib maximum dimensions and minimum weights (G.3, G.4).
- Appendix H.1, "Official J/80 Keel Plan & Offsets" and "Official J/80 Rudder Plan & Offsets", is the source of every keel and rudder dimension in `appendages.json`. These are minimum offsets.
- Some digits in the PDF use a font with shifted glyph codes (each glyph code is the character code minus 29). They were decoded and checked against the same rules in the 2006 version.

### International J/80 Class Rules 2006

- URL: https://www.sailing.org/tools/documents/J80_2006_CR_060208-[701].pdf
- Confirms the Appendix H.1 offsets table has nine thickness columns: 15.8 four times, then 16.4, 18.9, 25.2, 31.6, and the tip at 15.8. That matches the column assignment used in the 2024 table (see the keel offset table entry below).
- Also used to record values that changed: crew weight limit 338.6 kg or 3 people (now 350 kg, any number), and boom minimum weight 10.9 kg (now 9.6 kg). The current 2024 values are used.

### J/80 Standard Building Specifications (1 December 2020)

- URL: https://j80.org/images/pdf/J80_Standard_Building_Specifications.pdf
- Used for: two pairs of spreaders (item 45), standing rigging wire sizes (59-65), jib furler (69), 1 m jib T-tracks (7), and sheet, traveler and control purchases (70-78).

### J/Boats J/80 pages and drawings

- Tech specs: https://www.jboats.com/j80-tech-specs. LOA 8.00 m, LWL 6.71 m, beam 2.51 m, draft 1.49 m, ballast 635 kg (1,400 lb), displacement 1315 kg (2,900 lb), I 9.60, ISP 10.06, J 2.90, P 9.14, E 3.81 m, 100% SA 31.40 m^2. https://www.jboats.com/j80 states the 1,400 lb lead keel is a 48% ballast ratio.
- Rig profile drawing: https://www.jboats.com/images/slides/j80/700x350/J80-Rig-700.png
- Deck plan drawing: https://www.jboats.com/images/slides/j80/700x350/J80-Plan-700.png
- The drawings have no scale bar. The profile was scaled from P (9.144 m between the mast bands, 114.6 px/m). The plan was scaled from LOA and maximum beam (about 76 px/m). Every value taken from them is marked `estimated`, and its `method` says how it was scaled.

### IMS Rule 2020 and ORC Rating Systems 2019

- IMS Rule 2020: https://orc2.orc.org/rules/IMS%202020.pdf. Used for B4 Measurement Trim, which defines what the certificate displacement includes.
- ORC Rating Systems 2019: https://orc2.orc.org/rules/ORC%20Rating%20Systems%202019.pdf. Used for the measured-area formulas: rule 109.1 (mainsail) and 111.1/111.2 (headsail, with default widths where a width is not measured).

## Frame used in rig.json, sails.json, appendages.json and mass.json

The DWL midship is not fixed yet, so every position uses a provisional frame, stated in each file's `frame` field:

- x forward, y to port, z up, in metres.
- x = 0 at the stem: the forward end of LOA, which is also the forestay deck point because SFJ = 0. Points aft of the stem have negative x.
- z = 0 on the certificate's measurement-trim flotation plane, the plane its freeboards are measured from.

In sailing trim, with crew aboard, the boat sinks about 39 mm (350 kg at 8.99 kg/mm). A later hull-morph task moves the origin to DWL midship.

Positions assume a vertical mast, as P, IG and J are measured. The J/Boats profile shows about 3.2 deg of aft rake, recorded as `mast_rake`.

## Displacement and crew

ORC/IMS measured displacement does not include crew. IMS B4.1 requires the boat to be "dry in light ship condition without crew". B4.2 also removes sails, sheets and running rigging. Battery and the outboard as carried while racing stay in.

So the mass check compares the sum of the items marked `in_displacement: true` with the full 1438 kg, and does not subtract crew. Crew (class maximum 350 kg), sails and running rigging are listed separately.

The class builder's weight, 1425-1450 kg (D.5.1), covers hull, keel, rudder, deck, spars, and standing and running rigging. It excludes battery, sails and portable equipment. It brackets the certificate's 1438 kg, which is an independent confirmation.

J/Boats' 1315 kg displacement is a marketing figure that disagrees with both. It is recorded but not used.

## Conflicts and how each was resolved

- Keel ballast: J/Boats publishes 635 kg (1,400 lb) and `mass.json` uses it (confidence medium). The H.1 minimum offsets integrate to 0.0805 m^3 (centroid 0.653 m below Section 1), which would be about 885 kg at 11,000 kg/m^3, but that volume is the keel's displaced volume, not its lead volume. H.1 note 1 says Section 1 is the intersection of the hull fairbody with the moulded keel sump, so the offsets shape includes a GRP sump above the lead.
  - Material: lead strengthened with antimony (Class Rules 2024 E.2.1, same text in 2006; J/Boats "cast from lead and antimony"), about 11,000 kg/m^3 (lead 11,340 and antimony 6,690 kg/m^3 at about 4% Sb).
  - There is one casting and no separate bulb: a flared foot (Sections 5-8 widen to 200 mm, plus an elliptical tip, note 3).
  - Derived in `mass.json`: `keel_lead_volume` 0.0577 m^3 (635 / 11,000), `keel_sump_volume` 0.0228 m^3 (offsets minus lead), `bulb_mass` 321 kg (0.0292 m^3 below Section 4, lead filling from the tip up). The sump's mass is part of the hull/deck residual. In `appendages.json`, `lead_top_depth_below_root` and `ballast_centroid` locate the lead within the shape.
  - VCG sanity check: with VCGM -0.076 m from the certificate, 635 kg puts the rest-of-boat CG at about +0.64 to +0.79 m, which is plausible. 885 kg would put it at +1.36 m, above the deck.
  - Ballast ratio: 635/1438 = 0.442 against the certificate displacement, which hydrostatics use. J/Boats' 48% is 635/1315, using its lighter displacement figure.
- Keel offset table: the extracted Appendix H.1 table has eight half-width columns for nine headings (Sections 1-8 plus "Tip CL Profile below 8"). Columns were assigned by checking each section's listed leading-edge radius against the NACA 4-digit relation r = 1.1019 (t/c)^2 c. Under that check, Sections 1-4 are 100 mm thick, and Sections 5-8 are 104, 120, 160 and 200 mm. The last column is the tip profile. The radii reproduce to within 0.2 mm for Sections 4-8 and within 0.9 mm for Sections 1-3.
- Mainsail top width: the certificate's MHB is 0.16 m; the class maximum is 152 mm. The certificate value is used because it reproduces the certificate area. The class-maximum set is also checked.
- Jib half width: the certificate gives 1.48 m and the class maximum is 1475 mm. The difference is rounding.
- Spreader heights above sheer at mast: the J/Boats profile scales to 3.74 m and 6.94 m. The symbols on the ORC p.3 sail plan scale to 4.35 m and 7.39 m. The J/Boats drawing is the designer's, and the ORC sail plan is generated and schematic, so the J/Boats values are used.
- Jib track: the J/Boats plan draws a slot about 0.53 m long. The building specification says 1 m. A 1 m track centred on the drawn slot is used.

## Estimated items (not sourced)

All of the following are marked `"confidence": "estimated"` with a `method`:

- Rig:
  - forestay deck point height, forestay length and angle
  - mast top height and rake
  - spreader heights
  - shroud chainplate base
- Main:
  - tack, head and clew positions
  - sheet point on the boom
  - traveler centre and span
- Jib:
  - tack, head and clew positions; the clew is scaled from the ORC p.3 drawing and reproduces HLP within 1 mm
  - foot and leech lengths
  - track end positions
- Keel:
  - fore-and-aft position: root LE placed so the keel centroid is at the certificate LCF
  - root z, centroid
  - volume from offsets, lead top depth and ballast centroid
- Rudder:
  - stock axis position and root/tip z: root assumed at the waterplane at the transom
- Mass:
  - hull/deck/fittings, a residual from the builder's weight mid-range
  - standing rigging and furler, from wire lengths
  - battery
  - outboard, at the class minimum
  - running rigging
  - keel lead and sump volumes, bulb mass
  - ballast ratio
  - crew count

## Not found in any public source

- Spreader length and sweep angle.
- Keel fore-and-aft position on the hull.
- Rudder tip shape below Section 5, and the rudder root height.
- A measured bulb mass separate from the whole lead keel.

## Hull morph

`tools/hull/morph.py` builds `hull_visual.obj`, `hull_physics.obj` and `hull_hydro.json` from the closest Delft Systematic Yacht Hull Series hull (`data/dsyhs/README.md`). Run it with `.venv/bin/python -I tools/hull/morph.py`; `--check` recomputes hydrostatics from the written OBJs and fails on any miss. `hull_physics.obj` is checked to be watertight with outward winding; `hull_visual.obj` is for rendering only (normals split at creases) and need not be watertight.

### Targets

- **LWL** 6.71 m: J/Boats J/80 Tech Specs page (https://www.jboats.com/j80-tech-specs), confidence medium. It is the only plain LWL found; it is stored in `hull_hydro.json` `inputs`, which the script reads. The certificate LSM0 (6.967 m) and IMS L (7.237 m) are effective sailing lengths, 4-8% longer, which is consistent. J/Boats pairs it with its 1315 kg marketing displacement, so at 1438 kg the static waterline is probably a little longer.
- **Displacement** 1438 kg / 1025 kg/m^3 = 1.4029 m^3. The density is the certificate flotation SG (1.025, p.2). Keel 0.0805 m^3 (appendages.json `volume_from_offsets`, including the GRP sump). Keel root fairing 0.0061 m^3 (see below). Rudder 0.0096 m^3: NACA 4-digit section areas (0.6851 c t) integrated over Sections 1-5, with the root at the flotation plane as in appendages.json, and no tip below Section 5. Canoe body 1.3067 m^3.
- **BWL** 1.881 m, derived. The certificate sink, 8.99 kg/mm, is a waterplane area of 8.771 m^2. Dividing by LWL and the parent's Cw (0.695) gives BWL. The certificate beam (2.488 m) is 1.32 BWL, a strongly flared hull; the parent's own flare is 1.18.
- **Tc** 0.2425 m, derived from the canoe volume before the keel fairing (1.3129 m^3) and the parent's Cb (0.429); it seeds the vertical scale, which is then iterated on volume, and the morphed Tc is 0.242 m. It disagrees with the keel root depth in appendages.json (0.331 m below the flotation plane): no DSYHS hull is fine enough (Cb >= 0.336) to reach that depth at this LWL, BWL and volume. The keel root fairing fills the difference, so the keel keeps its H.1 root depth and the certificate draft.
- **LCB and Cp** are not on the certificate. They come from the parent (LCB 0.5455 Lwl aft of the FP, Cp 0.542) and are marked `derived`.

### Parent hull

The distance is sqrt(sum ln(p_parent / p_J80)^2) over Lwl/∇c^(1/3), Bwl/Tc and Lwl/Bwl. The J/80's Bwl and Tc are evaluated with each candidate's own Cw and Cb, so the distance measures how far the morph must distort that candidate. Cp and LCB are left out because the J/80's are unknown. The top five are model 61 (0.154), 30 (0.202), 49 (0.208), 50 (0.216) and 47 (0.227). The parent is **DSYHS model 61** (series 6), and the runner-up is model 30.

### Scaling, overhangs and sheer

- x, y and z are scaled by 3.228, 3.370 and 2.900 about the parent's DWL, iterated on the final mesh until LWL, BWL and canoe volume match.
- The stem is placed at the morphed LCF plus the certificate LCF from stem on CL (4.276 m), and the transom at LOA (8.032 m) aft of it. The overhangs are stretched beyond the waterline ends only, by a C1 quadratic in x, from 0.381 to 0.432 m forward and from 0.724 to 0.890 m aft. The immersed hull stays affine.
- The sheer is a quadratic through the certificate freeboards (p.2): FFM 0.877 m at SFFP 0.177 m, 0.744 m at the mast (J = 2.895 m aft of the stem, since SFJ = 0), and FAM 0.627 m at SAFP 8.015 m. The scaled DSYHS surface stops at a flat 0.635 m freeboard, so the topsides are extended vertically by up to 0.252 m forward and trimmed by up to 8 mm aft. The deck is flat at the sheer.

### Topside flare

The section points are flared before the surface is closed, so topology, deck and watertightness are unchanged. The girth point nearest the DWL is first moved onto the section's DWL crossing. Each half section is then straightened above that vertex (above its lowest point where the section is clear of the water): the turning angle at every girth vertex above it is multiplied by (1 - beta), and the straightened part is scaled uniformly about the DWL vertex back to the sheer height. Points at and below the DWL do not move. beta is solved per station so the sheer half-breadth becomes y_sheer (1 + (k0 - 1) w(x)). w(x) is a cos^2 taper peaking at the certificate max-beam station, 4.712 m from the stem (p.2), with half widths 4.712 m forward (0 at the stem) and 7.616 m aft (0.6 at the transom, keeping the wide stern). k0 = 1.1251 is solved for the certificate beam, 2.488 m. beta runs from 0 to 0.653 (0.635 at the peak), under a cap of 0.9 that keeps the topsides from going flat. Max beam goes from 2.212 to 2.488 m. It falls at the station 4.781 m from the stem both before and after the flare; the neighbouring stations are at 4.673 and 4.889 m. The flare leaves the canoe volume and BWL unchanged to the 6 decimals stored, and the waterplane area changes by +2e-5 relative (`hull_hydro.json` `flare`).

Why straightening: the parent's topsides turn one way from the bilge to the sheer, rounding in towards the sheer. An outward offset added as a function of height cannot keep that. If it grows faster than linearly (the earlier (z / z_sheer)^2) its outward curvature beats the topsides' own curvature, and about 100 of the 112 sections turned concave above 0.35-0.92 of the freeboard. If it starts with a slope (sin, 1 - (1 - t)^2, linear), it puts a reverse turn and a 7-13 degree slope step at the DWL; that gave 21-35 sections with a new inflection. A C1 smoothstep ramp over the lowest 10-25% of the freeboard, joined to a linear or concave-down profile, still left 3-32. Straightening only reduces how much each vertex turns. Every vertex keeps its turn direction, so no section gains an inflection, and the segment leaving the DWL vertex keeps its direction, so there is no slope step at the waterline.

Fairness (turning-direction sign changes along each half section, keel to sheer, before and after the flare): 0 of the 112 half sections gain an inflection. The DWL kink (the change in turning angle at the DWL vertex) is 0 deg. `--check` fails if more than 5% of sections gain an inflection. The flare is a modelling choice to reach the certificate beam, not a measured line.

### Keel root fairing

The keel keeps its H.1 `root_z` (-0.331 m), `span` (1.170 m) and `volume_from_offsets`, so its tip is at -1.501 m, the certificate draft. Between Section 1 and the morphed canoe body there is a gap on the centreline: 0.090 m at the root leading edge, 0.103 m at the trailing edge (0.089-0.103 m along the chord). appendages.json keel `root_fairing` fills it with Section 1 extruded vertically: area 0.6851 c t = 0.0658 m^2 times the mean gap over the root chord gives 0.0061 m^3. The gap depends on the vertical scale, so it is re-measured in every scale iteration and the fairing volume is part of the displacement sum. Heights and volume are in `hull_hydro.json` `result` (`keel_fairing_*`, `displacement_breakdown`). The keel root position fore and aft is itself estimated (appendages.json `root_leading_edge`).

### Frame

`hull_hydro.json` `transform` gives `stem_x_in_body` (3.787 m) and `flotation_z_in_body` (0). Add these to a point in the provisional stem/flotation frame of appendages.json, rig.json, sails.json and mass.json to get body coordinates. The DWL is the flotation plane because the displacement target is the measurement-trim displacement, with level trim assumed.

### Estimated, and open issues

- The overhang split, sheer extension, topside flare, keel root fairing and flat deck are modelling choices, not measured lines.
- The canoe-body wetted area is 9.43 m^2. The certificate WS minus twice the keel and rudder lateral areas is 9.13 m^2.
- The keel root (z = -0.331 m) sits 0.09-0.10 m below the morphed canoe-body bottom (Tc 0.242 m); the fairing bridges it. Either the J/80 canoe body has much slacker sections than any DSYHS hull, or the keel root depth needs another look.

## Appendage meshes and foils

`tools/hull/appendages.py` builds `appendages.obj` (objects `keel` and `rudder`, body frame) and the `foils` block of `appendages.json` from the H.1 offsets in `appendages.json`, converted to the body frame with the `hull_hydro.json` transform. Run it with `.venv/bin/python -I tools/hull/appendages.py`; `--check` recomputes everything from the written OBJ and the hull meshes and fails on any miss. Each object is a single watertight solid with outward winding (keel 3076 triangles, rudder 1686).

### Sections

- Every table section is a horizontal NACA 4-digit thickness form with the open trailing edge (half width 0.0105 t at x/c = 1, about the 1 mm H.1 gives), 41 cosine-spaced stations a side on the keel and 33 on the rudder. Between table sections the leading edge, chord and thickness are linear in depth, as in the trapezoidal `volume_from_offsets`.
- The keel's leading edge follows `leading_edge_x_aft_of_root_le_m` (a constant 19.29 deg sweep). Below Section 8 the tip is 9 rings plus an apex over 0.050 m, with chord and thickness both scaled by cos(theta) at depth 0.050 sin(theta) about the Section 8 x/c = 0.3 station: a quarter-ellipse in profile and in front view.
- The rudder's leading edge is vertical on the stock axis (0.05 m aft of the transom), from its root at the flotation plane to a flat foot at Section 5, with nothing below Section 5, as in the morph's rudder volume.

### How the roots meet the hull

- **Keel:** the Section 1 outline is carried straight up from `root_z`. Every outline vertex gets a ring vertex on `hull_visual.obj` (the hull bottom found straight above it) and a top vertex 0.020 m higher, inside the canoe body, closed by a flat-strip cap. There is no gap at any root vertex; against `hull_physics.obj`, whose bottom is up to 3 mm below the visual surface here, the surface ring is within 3 mm. The fairing height is 0.089-0.103 m, as in `hull_hydro.json`. The overlap inside the hull is 0.020 m x the Section 1 area, 0.00131 m^3, stored as `foils.keel.root_overlap`. `volume_below_hull` is the mesh minus its manifold3d intersection with `hull_physics.obj`, so it excludes the overlap.
- **Rudder:** `type` is transom-hung, not a spade. The blade hangs on the transom pintles, and its root at the flotation plane is 0.05 m aft of the transom and 0.106 m below its bottom edge (the transom is clear of the water). So the rudder does not touch the hull, and `--check` reports that root-to-hull test as N/A instead of failing it. It checks that the leading edge lies on `stock_axis` instead.

### Foils block

Body-frame values in `appendages.json` `foils`, recomputed from the OBJ by `--check`:

| | keel | rudder |
|---|---|---|
| span | 1.263 m (canoe body to tip, including the 0.093 m fairing; H.1 alone 1.170 m) | 1.145 m |
| root / tip chord | 0.960 / 1.230 m (Section 8, the bulb) | 0.340 / 0.260 m |
| LE / quarter-chord sweep | 19.29 / 16.04 deg (Sections 1-4) | 0.00 / -1.00 deg |
| t/c root / tip | 0.1042 / 0.1626 | 0.120 / 0.120 |
| planform area | 1.107 m^2 (H.1 part 1.018 m^2) | 0.364 m^2 |
| geometric / effective AR | 1.44 / 2.88 | 3.61 / 3.61 |
| quarter-chord centre | (-0.284, 0, -0.864) m | (-4.375, 0, -0.549) m |
| wetted area | 2.403 m^2 | 0.748 m^2 |
| volume below the hull | 0.08523 m^3 | 0.00954 m^3 |

- **Effective aspect ratio** is an assumption: 2x geometric for the keel, because a fin fixed to the hull with no gap treats the hull bottom as a reflection plane (an upper bound, lowered by heel and leeway), and 1x for the rudder, which hangs clear of the hull with no end plate. The physics may replace both with its own end-plate model.
- **Rudder stock:** (-4.2949, 0, 0) m with a vertical axis. H.1 gives the leading edge as vertical, and no source gives the rake of the transom pintle line.

### Volumes against the displacement accounting

- Keel below the hull 0.08523 m^3 vs `volume_from_offsets` + fairing (0.0805 + 0.00612 = 0.08662), -1.6%. The fin and tip alone are 0.07920 m^3 vs 0.0805: about 0.0010 of the gap is the tip, an ellipsoidal cap (2/3 A8 h) where the offsets estimate used pi/4 A8 h, and 0.0003 is linear chord and thickness (a section area quadratic in depth) where the estimate interpolated area linearly. The mesh fairing is 0.00603 m^3 vs the morph's 0.00612; the morph used the centreline gap over the chord unweighted, and the gap is largest at the thin trailing edge.
- Rudder 0.00954 m^3 vs the morph's 0.00956, -0.2%.
- The total difference, -0.0014 m^3, is 0.1% of the 1.4029 m^3 displacement. It is within tolerance, so `morph.py` and the hull are unchanged.
