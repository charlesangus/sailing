# M5 - Sails and Polar Calibration

The calibration milestone the survey calls make-or-break: a data-driven sail layer with the
J/80's main and jib, ORC-style lift/drag coefficients, sheet mechanics for the core racing
control set, rig windage, auto-hiking crew, and a steady-state harness that runs the dynamic
sim to equilibrium. Tuned until steady boat speed matches the reference polar within 5% at
TWS 6/10/16 kn.

## Scope

**In:**
- Apparent wind (uniform TWS here), N-sail aero layer with main and jib, jib-on-main interaction, controls: mainsheet, traveler, backstay, jib sheet, jib car, one reef.
- Rig/hull/crew windage, auto-hiking crew controller.
- Harness autopilot + auto-trimmer, `sailsim polar`, comparison report, calibration file, regression test.

**Out:**
- Asymmetric spinnaker (out of the demo).
- Vang, cunningham, outhaul, halyard tension, barber hauler (not in the control set).
- Wind gradient, gusts, sail lag and luffing flutter (M7 - Wind Field and Tacking).
- Tacking behaviour and jib sheet changeover (M7 - Wind Field and Tacking).

**Edge cases:**
- Sheet eased past the luffing point → sail weathervanes, lift ~0, drag small.
- Sheet overtrimmed → AoA past stall, lift drops, drag rises.
- Wind from astern on the jib → jib blanketed by the main (interaction term), little force.
- Head to wind → both sails luff, no drive; backed sails produce reverse force.
- Reef taken while loaded → area and CE height step down over ~10 s, not instantly.

**Assumptions:**
- ORC's published per-sail coefficient tables and flat/reef de-powering scheme are reproducible from the 2022/2023 VPP documentation.
- The non-spinnaker downwind target was settled by M2 - J/80 Reference Dossier (see its README).

## Phase 5.1: Aerodynamics

- [x] M5.P1.T1 — Apparent wind at the rig
  - files: core/include/sail/wind.hpp, core/src/wind.cpp, core/tests/test_wind.cpp
  - approach: A `WindField` interface (`sample(world_pos, t) → world velocity`) with a `UniformWind` implementation (TWS, direction). A helper computes apparent wind at a body-frame point from the boat state (velocity + ω × r), and the effective AWA/AWS in the heeled sail plane (heel reduces the effective angle per standard cos-heel projection). M7 adds gradient and gusts behind the same interface.
  - verify: doctest: boat at 5 kn into a 10 kn TWS at TWA 90° gives the analytic AWA/AWS; heel 20° reduces effective AWA as expected; ω contributes at the masthead.
  - size: M
- [x] M5.P1.T2 — Data-driven sail aero model
  - files: core/include/sail/sails.hpp, core/src/sails.cpp, core/tests/test_sails.cpp, data/sails/orc_main.csv, data/sails/orc_jib.csv
  - approach: Load any number of sails from `sails.json` (type, area, luff/foot/roach, CE height, attachment points). Per-sail Cl/Cd tables vs AWA from ORC documentation (main, jib) extended to ±180° for backed sails; ORC flat and reef parameters scale Cl and area/CE; effective height drives induced drag; a jib-on-main interaction term per ORC's combined-rig method. Force applied at each sail's CE (heel moment emerges). Output per-sail force, CE, and state (AoA, luffing/stalled flags) for visuals.
  - verify: doctest: combined Cl/Cd at sample AWAs reproduce ORC documentation values within 2% at flat = 1, reef = 1; flat < 1 reduces Cl monotonically; force is zero with zero apparent wind.
  - size: L
- [x] M5.P1.T3 — Sheet mechanics and trim controls
  - files: core/include/sail/sheets.hpp, core/src/sheets.cpp, core/include/sail/controls.hpp, core/tests/test_sheets.cpp
  - approach: Extend `Controls` with mainsheet, traveler, backstay, jib sheet (port and starboard), jib car, reef (0/1). Each sail has a free angle about its luff: it weathervanes to leeward until the sheet length limits it; traveler moves the main's sheeting point; backstay maps to ORC flat and twist; jib car to jib twist (and slightly to flat); reef steps area/CE over a 10 s transition. AoA = sheeted angle vs apparent wind; a luff state when AoA is below a per-sail luff threshold. Feeds M5.P1.T2's model.
  - verify: doctest: easing the mainsheet past luffing drops main Cl to <0.1; overtrimming past stall lowers Cl and raises Cd; moving traveler to windward raises AoA at constant sheet; reef halves no faster than its transition time.
  - size: L
- [x] M5.P1.T4 — Rig, hull and crew windage
  - files: core/include/sail/windage.hpp, core/src/windage.cpp, core/tests/test_windage.cpp
  - approach: `ForceModule` with drag from mast (cylinder by height strips), boom, rigging wire, hull topsides above the heeled waterline (projected area), and crew (area by position), each Cd from ORC docs. Applied at each element's centre so it contributes heel and yaw moments.
  - verify: doctest: with sails fully luffed, windage drag at AWS 20 kn head-to-wind matches a hand-computed total within 5%; windage is nonzero with sails furled.
  - size: M
- [x] M5.P1.T5 — Auto-hiking crew controller
  - files: core/include/sail/crew.hpp, core/src/crew.cpp, core/tests/test_crew.cpp
  - approach: Moves the crew point mass (M3 `set_crew_position`) toward the windward rail in proportion to heel above a threshold (full rail-out by ~15°), centreline in light air (< ~6 kn AWS) and downwind, with a 2 s transit lag when the windward side changes. Parameters in `data/boats/j80/crew.json`. Drive the crew through `MassModel::set_crew_position` and push the result into the running sim with `Simulation::set_body` plus each module's `set_reference_point` (the M3/M4 mid-run CG update path; `BoatSim` owns the modules).
  - verify: doctest: crew is at the windward rail at 20° heel, centred at 0°; switching tack moves it across in ~2 s.
  - size: M

## Phase 5.2: Steady-state harness

- [ ] M5.P2.T1 — Autopilot, auto-trimmer and `sailsim polar`
  - files: cli/cmd_polar.cpp, core/include/sail/harness/autopilot.hpp, core/src/harness/autopilot.cpp, core/include/sail/harness/autotrim.hpp, core/src/harness/autotrim.cpp
  - approach: PID autopilot on tiller holding a TWA. Auto-trimmer: coordinate-wise golden-section search over sheets, traveler, backstay, jib car and reef maximising time-averaged steady BSP (optionally VMG for beat/run), each evaluation running the dynamic sim to steady state (settle detection on BSP, heel, leeway). Build on M4's `BoatSim` (core/include/sail/boat_sim.hpp), which already assembles the full J/80 and runs turns and tows — and make `BoatSim` own the sail set, sheet controller, windage and crew controller too (today the crew update reaches sails/sheets/windage only through a listener), so one object assembles the sailing boat; settle detection must recognise a non-settling run (M4 saw a roll–yaw limit cycle in a hard-over 7 kn turn) and report it rather than averaging an oscillation. `sailsim polar --boat … --tws 6,10,16 --twa …` (with `--beat` / `--run` modes searching optimal VMG angles) writes CSV: tws, twa, bsp, heel, leeway, rudder, trim settings.
  - verify: Running TWS 10, TWA 90 converges to a steady BSP repeatably (two runs within 0.5%); output CSV loads.
  - size: L
- [ ] M5.P2.T2 — Polar comparison report
  - files: tools/calib/compare_polar.py, tools/calib/requirements.txt
  - approach: Reads `sailsim polar` CSV, `orc_polar.csv` and the downwind target from the dossier README; computes per-point error, beat/run angle error, and a pass/fail against 5% (BSP) and 2° (optimal beat angle); writes `docs/calibration/j80-polar.md` with a table and a PNG polar plot.
  - verify: Running it on a hand-made CSV that matches the target exactly reports all-pass; a 10% low CSV reports fails.
  - size: M

## Phase 5.3: Calibration

- [ ] M5.P3.T1 — Calibrate to the reference polar
  - files: data/boats/j80/calibration.json, data/boats/j80/sim_polar.csv, core/src/sails.cpp, docs/calibration/j80-polar.md
  - approach: Load a small set of documented multipliers from `calibration.json` (main/jib Cl max, Cd0, effective-height factor, interaction factor, hull resistance factor, windage factor) and tune them, fewest changes first, until `compare_polar.py` passes at TWS 6/10/16 across beat angle to 150° (and the downwind target to 180°). Heel at each point must be plausible (≤ ~25° upwind at 16 kn with reef/flat as the trimmer chooses). Commit the final report and the final `sailsim polar` output as `data/boats/j80/sim_polar.csv` (TWS 6–20 × TWA beat–180, the sim's own polar). Record each multiplier's final value and rationale in `calibration.json`.
  - verify: `compare_polar.py` reports all points within 5% and beat angle within 2° at TWS 6/10/16.
  - size: L
- [ ] M5.P3.T2 — Polar regression test
  - files: core/tests/test_j80_polar.cpp
  - approach: ctest (Release only, labelled `slow`, run with the `bench` preset — Release, tests on, no GDExtension; the `release` preset has tests off) runs a reduced polar — TWS 10, TWA 45/90/135 with stored optimal trim from M5.P3.T1 — and asserts BSP within 5% of the reference.
  - verify: `ctest --preset bench -L slow` passes.
  - size: M

**Verification gate:** `sailsim polar` at TWS 6/10/16 fed to `compare_polar.py` passes (BSP within 5%, beat angle within 2°) and `docs/calibration/j80-polar.md` is committed; `ctest` (dev, and bench `-L slow`) passes.

## Decisions

- 2026-11-27 — Main + jib only, no asymmetric spinnaker: user's choice; downwind calibrated to a non-spi ORC certificate if found, else Python-VPP main+jib.
- 2026-11-28 — Freshness check at claim: the slow regression runs under the `bench` preset (the `release` preset has tests off); the harness builds on M4's `BoatSim`; calibration starts from a hull ~13% stiffer than the certificate at small heel (M3 decision) and the `orc` hull resistance default (M4 decision), both of which the sail multipliers will partly absorb — record that in calibration.json.
- 2026-11-28 — Sails follow ORC VPP Documentation 2023 (§5.2 Tables 5.1/5.4, §5.4 combination with the 2023 kheff, §5.6 blanketing), high coefficient sets by default as Python-VPP does. ORC blanketing only shades headsail area outside the foretriangle (~1% for the J/80 jib) and Python-VPP has none, so the plan's "jib blanketed downwind" edge case is intentionally not modelled: the jib carries ~30% of the force at 180°, matching both calibration targets. The AoA override wraps the ORC optimum in a provisional luff/stall envelope (parameters are estimates, marked in data/sails/README.md).
- 2026-11-28 — Sheets: each sail rotates about its luff and is held by a geometric sheet (closed-form taut-sheet swing); a separate `SheetController` force module (registered before `SailSet`, adds no force) feeds AoA/flat/reef into `SailSet::set_trim`. All mapping parameters are low-confidence estimates in sheets.json; main reef area fraction 0.75 (one slab reef), 10 s transition. Sheet travel is linear in length, so the useful band is ~0.8–1.0 — an input-feel question for M10.
- 2026-11-28 — Windage: mast strips (bare-mast Cd 0.8 always — ORC uses 0.4–0.6 with the main set, so mast drag is overstated with sails up; left for the calibration windage factor), boom, nine wires, topsides panels clipped at the water, crew box; Cds from ORC 2023 §5 where given, otherwise standard values marked low confidence.
- 2026-11-27 — Control set: mainsheet, traveler, backstay, jib sheet, jib car, one reef — user chose the core racing set.
