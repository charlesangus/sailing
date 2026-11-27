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
- Body frame: x forward, y to port, z up, origin at the hull's DWL midship on the centreline.
- The GDExtension converts to Godot's Y-up, -Z-forward frame at the boundary, and nowhere else.

### Boat data

Everything boat-specific lives in `data/boats/j80/` (JSON/CSV/OBJ, with a source and confidence for every number) and is loaded at runtime. Nothing J/80-specific is hard-coded in the core; the sail layer is data-driven and supports any number of sails.
