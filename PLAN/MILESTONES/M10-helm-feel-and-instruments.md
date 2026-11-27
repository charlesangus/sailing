# M10 - Helm Feel and Instruments

Give the sailor the feedback the survey says realism is judged by: tiller load from the rudder
hinge moment (visible deflection, weather helm and gamepad rumble), heel and motion cues in the
helm camera, a racing-style instrument display with polar targets, and a compact trim panel.

## Scope

**In:**
- Tiller load model, rumble, weather-helm check, helm-camera motion cues, instrument display with damping and polar % target, masthead wind indicator, trim/crew-mode panel.

**Out:**
- Audio of any kind (out of the demo).
- Force-feedback wheels or tiller hardware (out of the demo).
- Racing rules, timers, start sequences (out of the demo).

**Edge cases:**
- No gamepad → rumble calls are no-ops.
- Instrument values during a tack (AWA crossing 0) → damping filters wrap angles correctly, no 359°→1° swing through 180°.
- Sternway → BSP shows negative or "—" consistently.

**Assumptions:**
- A 3–6° steady weather helm at ~20° heel on a close reach is the right target for a J/80 (practitioner figure, not measured).

## Phase 10.1: Helm

- [ ] M10.P1.T1 — Tiller load and rumble
  - files: core/include/sail/tiller.hpp, core/src/tiller.cpp, core/tests/test_tiller.cpp, game/scripts/boat_input.gd, game/scripts/boat_visual.gd
  - approach: Model the helmsman's arm as a spring-damper between the commanded tiller position and the tiller, loaded by the rudder hinge moment (M4) through the tiller length, so a loaded rudder lags and deflects visibly. Expose tiller load (N at the hand). Gamepad rumble strength proportional to load above a deadband (`Input.start_joy_vibration`); the visual tiller shows the actual angle.
  - verify: doctest: under a constant hinge moment the tiller settles offset from the command by moment/stiffness; zero load → no offset. `test_rumble.gd` headless checks rumble intensity increases with load (mocked).
  - size: M
- [ ] M10.P1.T2 — Weather-helm check and tuning
  - files: cli/cmd_polar.cpp, data/boats/j80/calibration.json, docs/calibration/helm.md
  - approach: Add steady rudder angle and tiller load to `sailsim polar` output. Check close-reach TWS 16 at ~20° heel gives 3–6° weather helm; if not, adjust the documented CE/CLR lead factor in `calibration.json` and re-run the polar regression. Record results in `docs/calibration/helm.md`.
  - verify: `docs/calibration/helm.md` shows steady rudder angle inside 3–6° at that point; `ctest --preset release -L slow` still passes.
  - size: M

## Phase 10.2: Motion cues

- [ ] M10.P2.T1 — Helm-camera motion cues
  - files: game/scripts/camera_rig.gd
  - approach: In helm POV the camera rides the boat (heel, pitch, heave) with a small head-stabilisation lag, horizon free to tilt; a subtle look-to-leeward offset when heeled past 15° so the rail-down feel reads; optional comfort toggle that keeps the horizon level.
  - verify: `test_cameras.gd` headless: camera roll follows boat heel in helm POV and stays level with the comfort toggle; UAT for feel.
  - size: M

## Phase 10.3: Instruments

- [ ] M10.P3.T1 — Instrument display with polar targets
  - files: game/scenes/instruments.tscn, game/scripts/instruments.gd, gdext/src/sail_boat.cpp, core/include/sail/polar_table.hpp, core/src/polar_table.cpp, core/tests/test_polar_table.cpp
  - approach: Load the sim's calibrated polar `data/boats/j80/sim_polar.csv` (committed by M5) into a `PolarTable` with bilinear interpolation. Display BSP, AWA/AWS, TWA/TWS, heel, VMG, target BSP and % of polar, with instrument-style damping (angle-aware) and a show/hide toggle. Add a masthead wind indicator (windex) to the boat visual.
  - verify: doctest: `PolarTable` interpolation reproduces table points exactly and is monotone between; `test_instruments.gd` headless: angle damping wraps through 0° correctly.
  - size: M
- [ ] M10.P3.T2 — Trim and crew panel
  - files: game/scenes/trim_panel.tscn, game/scripts/trim_panel.gd
  - approach: Compact panel showing mainsheet, traveler, backstay, jib sheet (active side), jib car and reef positions as bars, plus the auto/manual crew mode. Replaces the debug HUD by default (debug HUD stays behind a toggle).
  - verify: `test_trim_panel.gd` headless: each bar tracks its control.
  - size: M

**Verification gate:** `ctest --preset dev`, `ctest --preset release -L slow` and `tools/run_godot_tests.sh` pass; `docs/calibration/helm.md` shows 3–6° weather helm on a close reach at TWS 16; user UAT: tiller load and rumble track gusts and heel, instruments read sensibly through tacks, trim panel matches controls.

## Decisions

- 2026-11-27 — No audio in the demo: user's choice; feedback comes from helm, visuals and instruments only.
