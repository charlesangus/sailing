# Developer guide

## Stack

C++20 physics core (`core/`, static library `sailcore`) with no Godot dependency, wrapped by a godot-cpp GDExtension (`gdext/`). A CLI (`cli/`, binary `sailsim`) serves headless harnesses. Python tooling lives in `tools/` and runs in the repo's `.venv`. Core tests are doctest, run through ctest. Linux only.

## Prerequisites

- Linux x86_64
- g++ 13 or newer (C++20)
- python3 with the `venv` module
- git, curl

No system CMake or Godot is needed. `tools/bootstrap.sh` installs pinned CMake and Ninja into `.venv` and downloads the pinned Godot editor and export templates into `tools/godot/`, verifying their SHA-512 checksums (versions are in `tools/godot.lock`). Network access is required for bootstrap and for the first configure, which fetches Eigen and doctest.

## Bootstrap, build and test

```sh
git clone --recursive <repo-url> sailing
cd sailing
# or, in an existing clone: git submodule update --init --recursive

tools/bootstrap.sh
export PATH="$PWD/.venv/bin:$PATH"   # or call .venv/bin/cmake and .venv/bin/ctest directly

cmake --preset dev
cmake --build --preset dev
ctest --preset dev
tools/run_godot_tests.sh
```

`tools/run_godot_tests.sh` expects the `dev` build to exist: it runs `build/dev/cli/sailsim` and the GDExtension library, then runs each `game/tests/*.gd` headless under the bootstrapped Godot.

The first build compiles godot-cpp from source, which takes about 15 minutes on 4 cores. Later builds are incremental, and CI relies on a ccache cache to avoid that cost on most runs.

The `release` preset (`cmake --preset release`, `cmake --build --preset release`) builds an optimised core, CLI and GDExtension with tests off. The Godot test script uses the `dev` build.

The `bench` preset (`cmake --preset bench`, `cmake --build --preset bench`, `ctest --preset bench`) builds the core, CLI and tests in Release without godot-cpp, in about a minute. Use it for `build/bench/cli/sailsim bench --boat data/boats/j80` and for the step-cost budget test. Other builds mark that test with doctest's skip decorator: ctest does not list it, and a direct run of `sailcore_tests` counts it as skipped.

The milestone gate is `ctest --preset bench -L slow`. It runs the polar regression (`core/tests/test_j80_polar.cpp`): TWS 10 kn at TWA 45, 90 and 135 deg, sailed with the stored trims in `data/boats/j80/polar_regression.json` and no trim search, each point required to settle and to land within 5% of the reference polar (ORC certificate up to 75 deg, Python-VPP above, linear in TWA). It is Release-only like the budget test, so `ctest --preset dev` neither lists nor runs it; plain `ctest --preset bench` runs it with everything else, which costs a few seconds, and `-L slow` runs it alone. Its TWA is the course's, heading plus leeway, like every polar TWA. Refresh the stored trims from `sim_polar.csv` when the calibrated polar is regenerated; the current ones come from a polar that held the heading TWA, so they are 3 to 5 deg off their course angle upwind and a little off it at 90 deg.

Build output goes to `build/<preset>/`.

## Layout

- `core/` - physics core (`sailcore`), headers in `core/include/sail/`, doctest tests in `core/tests/`
- `cli/` - `sailsim` command-line harness
- `gdext/` - godot-cpp GDExtension (`sailgd`) exposing the core to Godot
- `game/` - Godot project, with headless tests in `game/tests/`
- `tools/` - bootstrap, Godot pin (`godot.lock`) and test runner scripts
- `data/` - boat data; arrives later as `data/boats/j80/`
- `docs/` - documentation
- `extern/` - git submodules (`godot-cpp`)

## Conventions

### Physics rules

- SI units throughout the core.
- Fixed physics step of 200 Hz inside the core. The core owns its accumulator; Godot's physics engine is not used for the boat.
- Visual systems (cloth, rigging, ocean rendering, telltales) are one-way: they read sim state and never feed back into it.
- The physics never reads GPU data. The ocean shares its spectrum with the GPU, never the reverse.
- Determinism: given a seed and inputs, a run is reproducible on the same build.

### Frames

- Core world frame: right-handed, x east, y north, z up, water plane z = 0.
- Hull frame: dossier data and meshes use the hull frame: x forward, y to port, z up, origin at the hull's DWL midship on the centreline.
- Rigid-body state: the state's position is the CG's world position. Body-frame vectors (velocities, wrenches) are expressed in axes parallel to the hull frame, about the CG. The hydrostatics module maps mesh vertices with `HydrostaticParams::reference_point`, the CG in hull-frame coordinates.
- The GDExtension converts to Godot's Y-up, -Z-forward frame at the boundary, and nowhere else.

### Boat data

Everything boat-specific lives in `data/boats/j80/` (JSON/CSV/OBJ, with a source and confidence for every number) and is loaded at runtime. Nothing J/80-specific is hard-coded in the core; the sail layer is data-driven and supports any number of sails.

### Bay terrain

`data/scenes/bay/heightmap.png` (16-bit greyscale, 1024 x 1024 at 4 m/px) and its `meta.json` are generated by `tools/scene/make_bay.py`; `meta.json` documents the pixel-to-world mapping, the height encoding, the bilinear sampling the core uses for bathymetry, and the boat's start. The game reads them from `res://data/scenes/` through the configure-time link and builds the mesh in `game/scripts/terrain_builder.gd` (`game/scenes/bay_terrain.tscn`).

```
.venv/bin/pip install -r tools/scene/requirements.txt
.venv/bin/python -I tools/scene/make_bay.py --seed 7   # writes data/scenes/bay/
.venv/bin/python -I tools/scene/make_bay.py --check    # byte-compares a fresh generation, runs the sanity checks
```

The same seed gives a byte-identical PNG; its sha256 is recorded in `meta.json`. Bump `GENERATOR_VERSION` in the script whenever a change alters the output.

## Game input

Actions are declared in `game/project.godot` and read by `game/scripts/boat_input.gd`, which sets the controls of the `SailBoat` at its `boat_path` each frame. Every action is bound on both devices. Held controls move at a fixed rate, scaled by the trigger or stick strength on a pad.

| Action | Keyboard | Gamepad | Effect |
| --- | --- | --- | --- |
| steer left / right | A, D or Left, Right | left stick X | tiller; right turns the boat to starboard |
| mainsheet in / out | W / S | right trigger / right bumper | 0.25 of travel per s |
| jib sheet in / out | E / Q | left trigger / left bumper | leeward jib sheet, 0.25 of travel per s |
| traveler windward / leeward | R / F | d-pad up / down | 0.5 per s over -1..1 |
| backstay on / off | T / G | right stick up / down, except in orbit mode | 0.25 per s (an in/out pair, not a toggle) |
| jib car forward / aft | Z / X | d-pad left / right | 0.25 per s |
| reef toggle | B | A | shakes out or reefs |
| camera switch | C | Y | `camera_switch_requested` signal; the camera rig cycles chase, orbit, helm |
| orbit left / right | J / L, right mouse drag | right stick X (orbit mode) | 2 rad per s; orbit mode only |
| orbit up / down | I / K, right mouse drag | right stick Y (orbit mode) | 1 rad per s, 5 to 85 degrees; orbit mode only |
| zoom in / out | `=` / `-`, mouse wheel | left stick up / down | exponential, 6 to 80 m; orbit mode only |
| drop mark | Space | X | `mark_drop_requested` signal |
| settings panel | Esc | Start | `settings_toggled` signal; the panel sets `BoatInput.controls_enabled` false while open so the d-pad and keys act on the UI only |
| debug HUD | H | Back (Select) | `toggle_hud` action, handled by `hud.gd` |

Debug HUD: `game/scripts/hud.gd` shows BSP, TWS/TWD/TWA (and course TWA), AWS/AWA, heel, leeway, rudder, every control position, sail luff/stall/backed flags, crew, seabed contacts when aground, the camera mode and `SailBoat.get_step_time_us()` (wall time per fixed step, mean over the last simulated second). The text refreshes at 10 Hz. The dial shows the true wind (blue) and apparent wind (yellow) relative to the bow.

Settings panel: `game/scripts/settings_panel.gd` is a stack of collapsible sections; `register_section(title, control)` adds one. Wind (TWS 0 to 30 kn, direction 0 to 359) is the first. The sim is not paused while it is open, so wind changes show immediately and `BoatInput` keeps polling the action that closes it. Vertical focus neighbours form a ring; sliders change with left/right (keys or d-pad).

Helm: the stick sets the tiller directly (deadzone 0.15, rescaled). Keys ramp it to full in 0.5 s and it self-centres at 4 per s once they are released.

Jib sheet: it acts on the sheet of the side the jib is on. `awa_deg >= 0` (wind over the starboard side) is starboard tack with the jib to port, so the port sheet is driven; otherwise the starboard sheet.

Cameras: `game/scripts/camera_rig.gd` holds three modes. Chase sits 16 m behind and 5 m above the boat on a heading smoothed with a 1 s time constant, and ignores heel. Orbit circles the boat at 22 m and 20 degrees to start, with the limits above, and the camera stays above the water. Helm is a camera on the hull frame, 0.5 m to the windward side of the tiller and 1.65 m above the flotation plane (1 m over the deck), looking 8 degrees down; right mouse drag looks around (100 degrees to each side).

The right stick is the backstay on a pad, so the rig clears `BoatInput.pad_backstay_enabled` while in orbit mode and the stick orbits instead; the backstay keys work in every mode. The pad backstay is the separate actions `backstay_pad_on` and `backstay_pad_off`. Pad look-around in helm mode is not provided.

A pad unplugged while an input is held has its held actions released, so the keyboard keeps working and nothing sticks.
