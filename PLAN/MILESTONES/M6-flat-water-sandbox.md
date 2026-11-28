# M6 - Flat-Water Sandbox

The first playable build: the calibrated J/80 in a Godot scene on flat water, in a procedural
bay with a coastline, headlands and a harbour mouth, with keyboard+mouse and gamepad controls
equally supported, chase/orbit/helm cameras, placeable marks, simple grounding, a settings
panel scaffold and a debug HUD. Constant uniform wind.

## Scope

**In:**
- `SailBoat` GDExtension node, input map (keyboard+mouse and gamepad, both tested), boat visuals with placeholder rigid sails, flat water shader, sky, procedural coastline from a heightmap, bathymetry grounding in the core, placeable buoys, cameras, debug HUD, settings panel scaffold.

**Out:**
- Wind gradient, gusts, presets (M7 - Wind Field and Tacking).
- Waves and ocean rendering (M8 - FFT Ocean and Waves).
- Cloth sails, telltales, rigging lines (M9 - Sail Cloth and Telltales).
- Instrument display and helm feel (M10 - Helm Feel and Instruments).
- Land wind shadow (out of the demo).
- Input rebinding UI (out of the demo).

**Edge cases:**
- Boat driven onto the shore → stiff penalty contact + friction stops it; the boat can heel and be sailed or drifted free.
- Keel touches the bottom in shallows before the hull does → contact acts on keel vertices too.
- Gamepad unplugged mid-session → keyboard keeps working; no crash.
- Frame-time spike → core accumulator drops time (M3), visuals interpolate.

**Assumptions:**
- A procedurally generated heightmap is acceptable art for the coastline.

## Phase 6.1: Bridge and input

- [x] M6.P1.T1 — `SailBoat` GDExtension node
  - files: gdext/src/sail_boat.hpp, gdext/src/sail_boat.cpp, gdext/src/frames.hpp, gdext/src/register_types.cpp, game/tests/test_sail_boat.gd
  - approach: `SailBoat` (Node3D) owns a core `BoatSim` (M5: it already assembles the whole sailing boat — sails, sheets, windage, crew, appendages — and exposes `set_controls`, `set_wind`, `step()`) built from a `boat_dir` property (`res://` path resolved to `data/boats/j80`, copied or symlinked into the game at build). `_process(delta)` calls `advance(delta)` and sets its transform from the interpolated state via `frames.hpp` (core Z-up → Godot Y-up, the only conversion point). Exposes control setters (tiller, mainsheet, traveler, backstay, jib sheets, jib car, reef) and read-only state (BSP, heel, leeway, AWA/AWS, TWA/TWS, rudder angle, hinge moment, per-sail angle/AoA/luff state, boom angle), and wind setters (TWS, direction). The wind is a `LogProfileWind` (TWS at 10 m, ORC log profile), the same field the polar was calibrated in, so sim speeds match `sim_polar.csv`.
  - verify: `game/tests/test_sail_boat.gd` headless: with trimmed sails at TWA 90°, after 30 s simulated the boat's speed is within 10% of the calibrated polar (`data/boats/j80/sim_polar.csv`, which also stores the trims) at TWS 10.
  - size: L
- [x] M6.P1.T2 — Input map for keyboard+mouse and gamepad
  - files: game/project.godot, game/scripts/boat_input.gd, game/tests/test_input.gd
  - approach: Godot input actions: helm (analog axis + keys with ramp), mainsheet in/out, jib sheet in/out (active side), traveler up/down, backstay on/off, jib car fwd/aft, reef toggle, camera switch, drop mark. Keyboard+mouse (mouse drag on helm optional) and gamepad (left stick helm, triggers sheets, bumpers/d-pad for the rest) mapped with equal coverage. `boat_input.gd` converts actions into `SailBoat` controls with sensible rates.
  - verify: `test_input.gd` headless injects key and joypad events with `Input.parse_input_event` and asserts each control moves in the right direction for both devices.
  - size: M

## Phase 6.2: Scene

- [x] M6.P2.T1 — Boat visuals with placeholder sails
  - files: game/scenes/boat.tscn, game/scripts/boat_visual.gd, game/assets/boat/
  - approach: Import `hull_visual.obj` and `appendages.obj`; simple procedural mast and boom from `rig.json`; boom and tiller rotate from sim state; main and jib as flat triangle-fan meshes from `sails.json` geometry, rotated to sheeted angle and tinted when luffing (replaced in M9). Simple materials.
  - verify: `game/tests/test_boat_visual.gd` headless checks boom rotation follows the sim boom angle; user UAT at the gate for looks.
  - size: M
- [x] M6.P2.T2 — Flat water, sky and atmosphere
  - files: game/scenes/environment.tscn, game/shaders/flat_water.gdshader
  - approach: Large camera-following water plane at y = 0 with scrolling normal-map ripples, Fresnel sky reflection and depth tint near shore (from the terrain depth); Godot procedural sky with sun; distance fog to the horizon.
  - verify: Scene loads headless without errors (`test_scene_load.gd`); looks checked at UAT.
  - size: M
- [x] M6.P2.T3 — Procedural bay heightmap and terrain
  - files: tools/scene/make_bay.py, data/scenes/bay/heightmap.png, data/scenes/bay/meta.json, game/scenes/bay_terrain.tscn, game/scripts/terrain_builder.gd
  - approach: Python generates a 16-bit heightmap (~4 × 4 km, 4 m/px) of a bay: headlands, a harbour mouth, a beach, open water to one side; `meta.json` gives scale and sea level. Godot builds a terrain mesh from it at load with slope/height colouring. The same heightmap is the bathymetry for M6.P2.T4.
  - verify: `make_bay.py` is deterministic (same seed → same hash); the terrain scene loads headless; UAT for looks.
  - size: L
- [x] M6.P2.T4 — Bathymetry grounding in the core
  - files: core/include/sail/grounding.hpp, core/src/grounding.cpp, core/tests/test_grounding.cpp
  - approach: Load the bathymetry heightmap (stb_image via FetchContent) with bilinear sampling. A `ForceModule` checks physics-mesh and appendage vertices against seabed height: penetrating vertices get a stiff spring + damper normal force and Coulomb friction, capped to keep the 200 Hz step stable. Add it to `BoatSim`'s module set (with `set_reference_point` for crew-driven CG moves) so the game and harnesses get it.
  - verify: doctest: boat driven at 5 kn onto a 10° slope stops within a boat length without NaNs or bounce-explosion; with the sail drive reversed (or the boat heeled by crew), it moves off again.
  - size: M
- [x] M6.P2.T5 — Placeable marks
  - files: game/scenes/buoy.tscn, game/scripts/marks.gd
  - approach: "Drop mark" action places a windward mark 0.5 nm upwind and a leeward mark at the boat's position (relative to current wind), replacing any previous pair. Buoys bob slightly; no collision.
  - verify: `test_marks.gd` headless: after the action, two buoys exist at the expected positions relative to wind.
  - size: M
- [ ] M6.P2.T6 — Cameras
  - files: game/scripts/camera_rig.gd, game/scenes/boat.tscn
  - approach: Chase (behind and above, heading-smoothed), orbit (mouse drag / right stick, zoom), helm POV (in the cockpit at the helmsman's eye, rolls and pitches with the boat). Camera-switch action cycles them.
  - verify: `test_cameras.gd` headless cycles all three without errors; UAT for feel.
  - size: M
- [ ] M6.P2.T7 — Debug HUD and settings panel scaffold
  - files: game/scenes/hud.tscn, game/scripts/hud.gd, game/scenes/settings_panel.tscn, game/scripts/settings_panel.gd
  - approach: Debug HUD text: BSP, TWS/TWA, AWS/AWA, heel, leeway, rudder, every control position, sim step µs, a wind-direction arrow. Settings panel (toggle key) with a section-based layout that later milestones add sections to; first section: TWS and wind direction sliders wired to `SailBoat`.
  - verify: `test_hud.gd` headless: HUD values update from a running sim; changing the TWS slider changes the sim's TWS.
  - size: M

**Verification gate:** `ctest --preset dev` and `tools/run_godot_tests.sh` pass; user UAT: launch `game/`, sail a beat and a run with keyboard+mouse and again with a gamepad, switch all cameras, drop marks, and run aground and get off.

## Decisions

- 2026-11-28 — Freshness check at claim: `SailBoat` wraps M5's `BoatSim` rather than a bare `Simulation`; the sandbox wind is `LogProfileWind` (constant, but height-profiled as calibrated); grounding joins `BoatSim`'s modules.
- 2026-11-28 — `SailBoat` calls `BoatSim::advance`, which runs the crew controller before each fixed step through a new `Simulation::advance` hook (calling `simulation().advance()` directly would freeze the crew the polar was calibrated with). Core exceptions are caught at the GDExtension boundary; on error the node goes inert. Boat data reaches the game as configure-time symlinks game/data/{boats,sails,foils,hull} → data/* (gitignored), so the res:// path is res://data/boats/j80.
- 2026-11-28 — Bay heightmap: seed 7, 1024² at 4 m/px, deterministic sha256 in meta.json; written as a minimal 16-bit PNG (filter 0, no metadata) and decoded in GDScript because Godot 4.7.2's PNG loader reduces 16-bit to 8-bit. Start (0, −1600) 31.6 m deep, harbour mouth ~710 m wide at ≥3 m. Terrain triangle winding/normals not yet viewed rendered — UAT.
- 2026-11-28 — Grounding: stb_image (pinned) decodes the 16-bit bathymetry; 656 decimated contact points (dense on the keel); contact natural frequency 10 Hz (k 7.1e6 N/m shared across contacts, 0.5 critical damping), smoothed Coulomb friction μ 0.5; a footprint broad phase makes deep water free. Registered in BoatSim only when a bathymetry is set, so harness and polar results are unchanged (byte-identical). `reset()` keeps the bathymetry: it belongs to the scene.
- 2026-11-27 — Coastline scene with simple grounding: user chose a bay with coastline over open water, with bathymetry contact physics and no land wind shadow.
- 2026-11-27 — Keyboard+mouse and gamepad equally: user's choice; both are tested in headless input tests.
