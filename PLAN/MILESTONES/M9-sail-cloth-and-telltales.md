# M9 - Sail Cloth and Telltales

Make trim readable: XPBD cloth for the main and jib, shaped one-way by the computed aero load,
with visible luffing and flutter; telltales on the jib luff and main leech driven by local
angle of attack; standing rigging that responds to backstay tension; and sheets/traveler drawn
as taut or slack lines. Visual only — nothing here feeds back into the physics.

## Scope

**In:**
- `sailvis` visual library (XPBD cloth, telltale state), sail meshes, Godot cloth node, luffing/flutter/back-wind visuals, telltales, standing rigging with mast bend, sheets and traveler as catenaries, sail material with draft stripes.

**Out:**
- Halyards, vang, cunningham, outhaul and cockpit tails (not drawn).
- Any cloth-to-physics feedback (Context one-way rule).
- Spinnaker (out of the demo).

**Edge cases:**
- Sail fully eased and flogging in 20 kn → cloth stable (substep cap), no explosion.
- Tack/gybe → jib cloth passes through the foretriangle; self-collision ignored, but the cloth must not tunnel through the mast (simple capsule collider).
- Reef → main cloth swaps to a reefed mesh over the reef transition.

**Assumptions:**
- ~2–3k particles per sail on the CPU fits < 2 ms for both sails.

## Phase 9.1: Cloth

- [ ] M9.P1.T1 — XPBD cloth solver library
  - files: core/visual/CMakeLists.txt, core/visual/include/sailvis/cloth.hpp, core/visual/src/cloth.cpp, core/visual/tests/test_cloth.cpp
  - approach: New static library `sailvis` (depends on sailcore headers read-only). XPBD with small-steps substepping: distance and dihedral bending constraints, pin constraints to moving attachment points, capsule colliders, and an external per-triangle pressure load. API: build from a triangle mesh + pin set, `set_pins`, `set_pressure(per-triangle)`, `step(dt)`.
  - verify: doctest: a pinned square drapes to a stable state; a uniform pressure inflates it to a bounded camber; 1000 steps in a 20 kn-equivalent flog produce no NaN or particle escaping a bounding box.
  - size: L
- [ ] M9.P1.T2 — Sail meshes from sail geometry
  - files: core/visual/include/sailvis/sail_mesh.hpp, core/visual/src/sail_mesh.cpp, core/visual/tests/test_sail_mesh.cpp
  - approach: Generate a triangulated sail per `sails.json` (luff, foot, leech, roach, head), ~2–3k particles, UVs aligned to chord/span for draft stripes, and pin sets: main luff to mast, foot to boom, clew to sheet point; jib luff to forestay, tack, clew to the jib sheet. A reefed variant for the main.
  - verify: doctest: mesh area within 2% of the sail's published area; pins lie on the luff/foot edges.
  - size: M
- [ ] M9.P1.T3 — Aero-driven cloth in Godot
  - files: gdext/src/sail_cloth.hpp, gdext/src/sail_cloth.cpp, gdext/src/register_types.cpp, game/scenes/boat.tscn, game/scripts/boat_visual.gd
  - approach: `SailCloth` node per sail reads the sim's per-strip AoA, Cl and luff/flutter state each frame, maps them to a chordwise pressure distribution per triangle, steps `sailvis`, and writes an ArrayMesh. Pin positions follow mast, boom and sheet points from sim state. Replaces M6's placeholder sails. Budget < 2 ms total for both sails.
  - verify: `test_sail_cloth.gd` headless: cloth updates for 300 frames without errors and reports per-frame cost under budget; UAT that sail shape follows trim.
  - size: M
- [ ] M9.P1.T4 — Luffing, flutter and back-winding visuals
  - files: gdext/src/sail_cloth.cpp, core/visual/src/cloth.cpp
  - approach: When a strip is luffing, apply oscillating pressure near the luff at the M7 flutter frequency (luff bubble); when backed, reverse the pressure; leech flutter when the leech strip is near stall. Amplitude scales with AWS.
  - verify: UAT: easing the jib past luffing shows a luff bubble then full flogging; backing the jib in a tack bellies it the wrong way.
  - size: M

## Phase 9.2: Telltales and rigging

- [ ] M9.P2.T1 — Telltales
  - files: core/visual/include/sailvis/telltale.hpp, core/visual/src/telltale.cpp, core/visual/tests/test_telltale.cpp, gdext/src/sail_cloth.cpp
  - approach: Jib luff windward/leeward pairs at 3 heights and main leech telltales at the batten ends, attached to cloth particles. A state function from local strip AoA (streaming, windward lifting, leeward stalling, leech stalled/curling) drives a short verlet ribbon animation.
  - verify: doctest: the state function maps AoA bands to the expected states for both sails; UAT that telltales read correctly as you trim.
  - size: M
- [ ] M9.P2.T2 — Standing rigging and mast bend
  - files: game/scripts/rigging.gd, game/scenes/boat.tscn
  - approach: Shrouds, forestay and backstay as thin meshes between rig points from `rig.json`; mast bends (simple bend curve) and forestay sag decreases with backstay tension from sim state.
  - verify: `test_rigging.gd` headless: mast bend parameter changes with backstay control; UAT for looks.
  - size: M
- [ ] M9.P2.T3 — Sheets and traveler as lines
  - files: game/scripts/running_rigging.gd, game/scenes/boat.tscn
  - approach: Mainsheet, traveler car position and both jib sheets as tube-rendered catenary splines between their fixed points; sag from slack (sheet length minus straight-line distance) and straight when loaded (sim sheet load > threshold); the lazy jib sheet hangs slack.
  - verify: `test_running_rigging.gd` headless: easing the sheet increases the spline sag; UAT for looks.
  - size: M
- [ ] M9.P2.T4 — Sail material and draft stripes
  - files: game/assets/sails/sail_material.tres, game/assets/sails/draft_stripes.png
  - approach: Semi-translucent Dacron-like material with backlight transmission and two-sided lighting; draft stripes at 3 heights using the chord/span UVs so camber and draft position can be read.
  - verify: UAT: stripes visible from the helm camera on both tacks.
  - size: M

**Verification gate:** `ctest --preset dev` (incl. `sailvis` tests) and `tools/run_godot_tests.sh` pass; cloth cost < 2 ms per frame for both sails; user UAT: sail shape, luffing, back-winding, telltales and sheets read correctly while trimming through a beat, reach and tack.

## Decisions

- 2026-11-27 — Rigging drawn: standing rigging plus mainsheet, traveler and jib sheets; user chose the key lines over standing-only or everything.
