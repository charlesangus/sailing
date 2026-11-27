# M1 - Build Skeleton and CI

Stand up the repo layout every later milestone builds into: a CMake-built C++20 core library
with doctest tests and no Godot dependency, a CLI harness binary, a godot-cpp GDExtension, a
Godot 4 project that loads it, a toolchain bootstrap script, and GitHub Actions CI. Nothing
sails yet — the deliverable is a green pipeline from core test to headless Godot.

## Scope

**In:**
- `tools/bootstrap.sh`, CMake presets, `core/`, `cli/`, `gdext/`, `game/`, CI workflow, dev docs.

**Out:**
- Any physics (M3 - Rigid Body and Hydrostatics onwards).
- Windows/macOS builds and export presets (out of the demo entirely).
- Release packaging (M11 - Playtest Build).

**Edge cases:**
- Godot download fails or checksum mismatches → bootstrap exits non-zero with the URL and expected hash; never runs an unverified binary.
- Re-running bootstrap on a populated `tools/godot/` → no re-download (idempotent).

**Assumptions:**
- godot-cpp has a tag matching the pinned Godot 4.x release and builds under its CMake support.

## Phase 1.1: Repo and toolchain

- [ ] ~~M1.P1.T1 — Copy the prior-art survey into the repo~~ (cancelled: the survey stays local; see Decisions)
  - files: docs/research/prior-art-survey.md
  - approach: Copy `.local/Realistic Sailing Sim — Prior Art & Feasibility Survey.md` (project root) verbatim to `docs/research/prior-art-survey.md`. No edits.
  - verify: `diff` against the source file shows no differences.
  - size: M
- [x] M1.P1.T2 — Write the toolchain bootstrap script
  - files: tools/bootstrap.sh, tools/godot.lock, .gitignore
  - approach: Bash, `set -euo pipefail`. Creates `.venv` (python3 -m venv) and pip-installs pinned `cmake` and `ninja`. Reads the Godot version, Linux x86_64 editor URL, export-templates URL and their SHA-512 sums from `tools/godot.lock` (pin the latest stable Godot 4.x); downloads into `tools/godot/` only if absent, verifies the hash, unzips, and symlinks `tools/godot/godot`. Idempotent. Add `.venv/`, `tools/godot/`, `build/`, `dist/` to `.gitignore`.
  - verify: Run it twice from a clean checkout: the first downloads and verifies, the second is a no-op; `tools/godot/godot --version` prints the pinned version; corrupting the lock hash makes it fail non-zero.
  - size: M
- [x] M1.P1.T3 — CMake project with core library, tests and CLI
  - files: CMakeLists.txt, CMakePresets.json, core/CMakeLists.txt, core/include/sail/version.hpp, core/src/version.cpp, core/tests/test_version.cpp, cli/CMakeLists.txt, cli/main.cpp, .clang-format
  - approach: Top-level CMake (3.24+, Ninja) with presets `dev` (Debug, tests on) and `release`. `sailcore` static lib, C++20, warnings as errors (`-Wall -Wextra -Wpedantic`), no Godot includes. doctest via FetchContent, test exe registered with ctest. `sailcore::version()` returns a string from a configure-time version. `sailsim` CLI links sailcore; `sailsim --version` prints it; use a tiny subcommand dispatcher later milestones extend. Eigen 3.4 via FetchContent made available to sailcore (used from M3).
  - verify: `cmake --preset dev && cmake --build --preset dev && ctest --preset dev` passes; `build/dev/cli/sailsim --version` prints the version.
  - size: M

## Phase 1.2: Godot extension

- [x] M1.P2.T1 — godot-cpp GDExtension library
  - files: .gitmodules, extern/godot-cpp (submodule), gdext/CMakeLists.txt, gdext/src/register_types.cpp, gdext/src/sail_sim.hpp, gdext/src/sail_sim.cpp
  - approach: Add godot-cpp as a submodule pinned to the tag matching `tools/godot.lock`. Build it through its CMake support as part of the top-level project. Shared library `sailgd` links sailcore and godot-cpp, outputs into `game/bin/`. Registers one class `SailSim` (extends Node) with a method `get_core_version()` returning `sailcore::version()`. All Godot types stay in `gdext/`.
  - verify: `cmake --build --preset dev` produces `game/bin/libsailgd.linux.*.so`; `nm -D` shows the GDExtension entry symbol.
  - size: M
- [x] M1.P2.T2 — Godot project and headless smoke test
  - files: game/project.godot, game/sailgd.gdextension, game/scenes/main.tscn, game/tests/smoke.gd, tools/run_godot_tests.sh
  - approach: Minimal Godot 4 project, main scene with a `SailSim` node. `sailgd.gdextension` points at `bin/` debug/release libs for linux.x86_64. `game/tests/smoke.gd` (extends SceneTree) instantiates `SailSim`, checks `get_core_version()` equals the CLI's version, and quits with exit code 0/1. `tools/run_godot_tests.sh` runs every `game/tests/*.gd` with `tools/godot/godot --headless --path game --script` and fails on any non-zero exit.
  - verify: `tools/run_godot_tests.sh` exits 0; breaking the version check makes it exit non-zero.
  - size: M

## Phase 1.3: CI and conventions

- [ ] M1.P3.T1 — GitHub Actions CI
  - files: .github/workflows/ci.yml
  - approach: ubuntu-latest, on push to master and on PRs. Checkout with submodules, cache `tools/godot/` and `.venv` keyed on `tools/godot.lock`, run `tools/bootstrap.sh`, configure/build/test the `dev` preset, then `tools/run_godot_tests.sh`. Keep it under ~10 minutes (cache the godot-cpp build dir).
  - verify: The workflow passes on this milestone's PR (`gh pr checks --watch`).
  - size: M
- [x] M1.P3.T2 — Developer docs
  - files: docs/dev.md, README.md
  - approach: `docs/dev.md` states: bootstrap/build/test commands; the repo layout (core, cli, gdext, game, tools, data, docs); the unit and frame conventions and the one-way visual rule, copied from the board's `# Context and constraints`; where boat data lives. README links to it.
  - verify: Following `docs/dev.md` from a clean clone reproduces a green build and test run.
  - size: M

**Verification gate:** from a clean clone, `tools/bootstrap.sh && cmake --preset dev && cmake --build --preset dev && ctest --preset dev && tools/run_godot_tests.sh` all pass, and CI is green on the PR.

## Decisions

- 2026-11-27 — CMake + Ninja rather than SCons for godot-cpp: one build system for core, CLI and extension, and presets give CI and worktrees identical commands.
- 2026-11-27 — Linux only: chosen by the user; the core stays portable C++20 so other platforms are additive later.
- 2026-11-27 — Prior-art survey stays in `.local/`, never committed: the repo is public and the user wants the survey kept private; M1.P1.T1 cancelled.
- 2026-11-27 — Pinned Godot 4.7.2-stable with godot-cpp `10.0.0-stable` (api_version 4.7): godot-cpp has no `godot-4.7-stable` tag; from v10 it is versioned independently and targets 4.3+ via `api_version`. Ninja pinned at 1.13.0 (1.12.1 is not on PyPI).
