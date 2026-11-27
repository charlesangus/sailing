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
