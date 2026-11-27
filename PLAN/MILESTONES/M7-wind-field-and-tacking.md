# M7 - Wind Field and Tacking

Replace the uniform wind with a realistic one and make manoeuvres feel right: a log-law wind
gradient (sails sampled in spanwise strips so twist matters), an advected gust-and-shift
field shown on the water, sail-force lag and luffing flutter, a crew that handles the jib
changeover automatically or leaves it to the player (toggle), wind presets with live sliders,
and tack and gybe harnesses checked against documented targets.

## Scope

**In:**
- Wind gradient, gust/shift field, per-sail spanwise strips, sail force lag and flutter, presets and live sliders, puff darkening on the flat water shader.
- Tack/gybe crew with an auto/manual toggle, tack-target research, tack and gybe harnesses.

**Out:**
- Land wind shadow (out of the demo).
- Real weather import (out of the demo).
- Visual sail flutter and telltales (M9 - Sail Cloth and Telltales) — this milestone computes the luff/flutter state only.
- Puff rendering on the FFT ocean — M8 - FFT Ocean and Waves carries the puff input over to its shader.

**Edge cases:**
- Crash gybe at 16 kn → boom swings with a large moment, no numerical blowup.
- Tack stalls head to wind (in irons) → boat drifts backwards; sternway handled by M4's models; the player can back the jib to bear away.
- Manual mode, player never releases the old jib sheet → jib backs and the boat is pushed round (heave-to behaviour).
- Gust strength slider at 0 → steady wind identical to `UniformWind` at the same TWS.

**Assumptions:**
- Masuyama 2011 (Fujin tacking data) or equivalent is obtainable; otherwise practitioner ranges for a J/80 are documented and used as targets.

## Phase 7.1: Wind field

- [ ] M7.P1.T1 — Wind gradient and spanwise sail strips
  - files: core/include/sail/wind.hpp, core/src/wind.cpp, core/src/sails.cpp, core/tests/test_wind.cpp, core/tests/test_sails.cpp
  - approach: Add a log-law gradient (reference TWS at 10 m, sea roughness z0 configurable) to the `WindField` interface. Split each sail into 3 spanwise strips; compute apparent wind per strip at its height (twist emerges from gradient + boat motion), and sum strip forces. Keep M5 - Sails and Polar Calibration's calibration: re-run `sailsim polar` at TWS 10 and adjust only `calibration.json` multipliers if the polar moves more than 2%.
  - verify: doctest: AWS increases with height per the log law; strip AWA differs top to bottom at TWA 90°; `compare_polar.py` still passes at TWS 6/10/16.
  - size: M
- [ ] M7.P1.T2 — Advected gust and shift field
  - files: core/include/sail/gusts.hpp, core/src/gusts.cpp, core/tests/test_gusts.cpp
  - approach: A `WindField` decorator adding speed and direction perturbations from 2D simplex noise (puff scale ~100–300 m, evolving in time) advected with the mean wind, with a deterministic seed; parameters gust strength (σ of speed, kn) and shift amplitude (σ of direction, °). Exposes a sampling call for the renderer (gust intensity on a grid around the boat).
  - verify: doctest: over a long sample, mean TWS equals the setting within 2% and σ matches gust strength within 10%; gust strength 0 equals the undecorated field exactly; the pattern translates at the mean wind speed.
  - size: M
- [ ] M7.P1.T3 — Wind presets and live sliders
  - files: data/wind/presets.json, gdext/src/sail_boat.cpp, game/scripts/settings_panel.gd, game/scenes/settings_panel.tscn
  - approach: Presets: light 6 kn, medium 10 kn, fresh 16 kn, puffy offshore (12 kn, strong gusts and shifts). Settings panel "Wind" section: preset picker plus live sliders for mean TWS, direction, gust strength, shift amplitude, applied immediately through `SailBoat`.
  - verify: `test_wind_settings.gd` headless: selecting each preset sets the sim's wind parameters; moving a slider changes the sim value.
  - size: M
- [ ] M7.P1.T4 — Puffs visible on the water
  - files: game/shaders/flat_water.gdshader, game/scripts/puff_texture.gd, gdext/src/sail_boat.cpp
  - approach: Each frame (or at 10 Hz) fill a small texture (e.g. 128², ~1 km around the boat) from the core gust sampler and feed it to the water shader, which darkens and roughens the surface where wind is stronger. Expose the same texture as a reusable input so M8's ocean shader can take it over.
  - verify: `test_puff_texture.gd` headless: texture values correlate with core gust samples at the same points; UAT that puffs are readable from the helm camera.
  - size: M

## Phase 7.2: Unsteady sails and crew

- [ ] M7.P2.T1 — Sail force lag and luffing flutter
  - files: core/src/sails.cpp, core/include/sail/sails.hpp, core/tests/test_sails.cpp
  - approach: First-order lag on each strip's Cl and Cd toward the steady value with τ = k·chord/AWS (k configurable, start ~2–4). When AoA is below the luff threshold, apply a lift loss and a small oscillating drag (flutter) whose frequency scales with AWS/chord; expose a per-strip `flutter` state for M9. Backed sails use the ±180° tables already in place.
  - verify: doctest: a step in AoA reaches 63% of the new Cl after τ; flutter state is set only while luffing; polar regression (`ctest -L slow`) still passes.
  - size: M
- [ ] M7.P2.T2 — Tack and gybe crew with auto/manual toggle
  - files: core/include/sail/crew.hpp, core/src/crew.cpp, core/include/sail/controls.hpp, gdext/src/sail_boat.cpp, game/scripts/boat_input.gd, game/scripts/settings_panel.gd, core/tests/test_crew.cpp
  - approach: Auto mode: as the bow passes head to wind the crew releases the old jib sheet, then trims the new sheet to the previous setting over 2–4 s, and moves the hiking mass across with the 2 s lag (M5). Manual mode: the player has separate release and trim actions for each jib sheet (both input devices); the crew still moves across. Gybes: boom swings when the main fills on the new side, its angular rate limited only by physics. Toggle lives in the settings panel ("Crew" section).
  - verify: doctest: in auto mode a scripted tack ends with the new jib sheet at the old setting within 4 s; in manual mode the jib stays backed until released. `test_crew_toggle.gd` headless switches modes.
  - size: L

## Phase 7.3: Tacking validation

- [ ] M7.P3.T1 — Tack and gybe targets from the literature
  - files: docs/calibration/tack-targets.md
  - approach: Find Masuyama et al. 2011 (sail-dynamometer boat Fujin tacking measurements) and any J/80 or similar-size keelboat tacking data; extract tack duration (head-to-wind to new close-hauled), minimum speed as % of entry speed, time to recover 90% of speed, and distance lost to windward. Scale to J/80 size where needed and state the method. Mark any figure not from a source as an assumption.
  - verify: The doc lists each target with a value range and a source or "assumption".
  - size: M
- [ ] M7.P3.T2 — Tack harness and tuning
  - files: cli/cmd_tack.cpp, core/tests/test_j80_tack.cpp, data/boats/j80/calibration.json
  - approach: `sailsim tack --tws 10 --rate R` sails close-hauled to steady state (M5 autopilot), then follows a scripted tiller profile through the tack with auto crew, and measures the M7.P3.T1 metrics. Tune only the sail lag k, yaw damping and crew timing until within M7.P3.T1 ranges at TWS 6/10/16. ctest (`slow`) asserts the ranges at TWS 10.
  - verify: `sailsim tack` at TWS 6/10/16 reports metrics inside the targets; `ctest -L slow` passes.
  - size: L
- [ ] M7.P3.T3 — Gybe harness
  - files: cli/cmd_gybe.cpp, core/tests/test_j80_gybe.cpp
  - approach: `sailsim gybe --tws 16 --mode controlled|crash` runs a broad-reach to broad-reach gybe; reports boom peak angular rate, heel excursion, heading overshoot. ctest asserts no NaN/blowup in a crash gybe at 16 kn and that a controlled gybe completes with heel excursion below the crash case.
  - verify: `ctest -L slow -R gybe` passes.
  - size: M

**Verification gate:** `ctest --preset dev` and `ctest --preset release -L slow` pass (polar, tack, gybe); `tools/run_godot_tests.sh` passes; `sailsim tack` metrics sit within `docs/calibration/tack-targets.md` at TWS 6/10/16; user UAT: tack and gybe in auto and manual crew modes in the puffy preset, puffs readable on the water.

## Decisions

- 2026-11-27 — Jib changeover is toggleable auto/manual: user chose a settings toggle over either mode alone.
- 2026-11-27 — Wind control is presets plus live sliders: user's choice; presets match the calibration speeds.
