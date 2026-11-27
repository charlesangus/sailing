---
title: Realistic Sailing Sim — J/80 Tech Demo
status: running
ship: pr-per-milestone
publish_decisions: docs/decisions/
---

# Goal

A playable Linux tech demo of a realistic sailing simulator for one boat, the J/80
(26' Bermuda-rigged sloop, main + jib). Done means: a system-based 6-DOF model
(hydrostatics, DSYHS hull resistance, keel/rudder foils, ORC-style sails, windage) whose
steady-state speeds match the J/80 reference polar within ~5% at TWS 6/10/16 kn; tacks and
gybes whose timing and speed loss match documented targets; wind gradient, gusts and shifts;
an FFT ocean up to ~3 m Hs coupled to wave-relative buoyancy; XPBD sail cloth with telltales,
helm feel and instruments; and a packaged build the user has playtested. This covers steps
1–6 of the survey's proof-of-concept plan; N-sail generalisation to other rigs and a second
boat are out.

# Context and constraints

- **Research basis.** The prior-art and feasibility survey is
  `.local/Realistic Sailing Sim — Prior Art & Feasibility Survey.md` in the project root
  (`../../.local/` from a lane worktree). It is **local only — never commit or copy it into
  the repo** (the repo is public). It names the sources each force model comes from: ORC VPP docs, DSYHS,
  Python-VPP, Kerner's boat-water model, Masuyama's tacking data, Tessendorf/Arc Blanc ocean.
- **Stack.** Godot 4 (latest stable 4.x, pinned in M1) front end; a C++20 physics core
  (`core/`, static library `sailcore`) with **no Godot dependency**, wrapped by a godot-cpp
  GDExtension (`gdext/`); a CLI (`cli/`, binary `sailsim`) for headless harnesses; Python
  tooling in `tools/` (run in the repo's `.venv`). Build is CMake + Ninja with presets; tests
  are doctest via ctest. Linux only — no Windows/macOS builds.
- **Toolchain bootstrap.** The dev box has no system cmake or Godot. `tools/bootstrap.sh`
  (M1) installs cmake/ninja into `.venv` and downloads the pinned Godot editor and export
  templates into `tools/godot/` (gitignored). Run it before building in a fresh checkout or
  worktree.
- **Physics rules.** SI units throughout the core. Fixed physics step 200 Hz inside the core
  (the core owns its accumulator; Godot's physics engine is not used for the boat). Visual
  systems (cloth, rigging, ocean rendering, telltales) are **one-way**: they read sim state
  and never feed back into it. The physics never reads GPU data; the ocean shares its
  spectrum with the GPU, never the reverse. Determinism: given seed + inputs, a run is
  reproducible on the same build.
- **Frames.** Core world frame: right-handed, x east, y north, z up, water plane z = 0. Body
  frame: x forward, y to port, z up, origin at the hull's DWL midship on the centreline. The
  GDExtension converts to Godot's Y-up, -Z-forward frame at the boundary — nowhere else.
- **Boat data.** Everything boat-specific lives in `data/boats/j80/` (JSON/CSV/OBJ with a
  source and confidence for every number, built by M2) and is loaded at runtime. Nothing
  J/80-specific is hard-coded in the core; the sail layer is data-driven (any number of
  sails) even though the demo has two.
- **Calibration targets.** Upwind and reaching: the J/80 ORC certificate polar. Downwind
  (main + jib only): a J/80 ORC non-spinnaker certificate if one exists, else Python-VPP run
  on the dossier with main + jib. Tacking: `docs/calibration/tack-targets.md` (M7).
- **Verification without a GPU.** Core and harness checks run headless in ctest and CI.
  Godot checks run `godot --headless` scripts under `game/tests/`. Anything that needs real
  rendering is verified by the user as UAT (the PM parks the milestone with `Needs: uat`),
  or by a screenshot run through the `cloud-gpu` skill when the PM needs one itself.
- **Out of scope for the whole demo.** Audio, multiplayer/networking, asymmetric
  spinnaker, Windows/macOS, a hand-modelled hull, interactive wake or spray, flooding or
  swamping, motor/outboard, real-weather import.

# Board

| ID | Milestone | Status | Deps | Owner | Epic | File |
|----|-----------|--------|------|-------|------|------|
| M1 | Build Skeleton and CI | doing | - | bosley | E1 | [M1-build-skeleton-and-ci.md](PLAN/MILESTONES/M1-build-skeleton-and-ci.md) |
| M2 | J/80 Reference Dossier | todo | - | | E1 | [M2-j80-reference-dossier.md](PLAN/MILESTONES/M2-j80-reference-dossier.md) |
| M3 | Rigid Body and Hydrostatics | todo | M1, M2 | | E1 | [M3-rigid-body-and-hydrostatics.md](PLAN/MILESTONES/M3-rigid-body-and-hydrostatics.md) |
| M4 | Hull Resistance and Foils | todo | M3 | | E1 | [M4-hull-resistance-and-foils.md](PLAN/MILESTONES/M4-hull-resistance-and-foils.md) |
| M5 | Sails and Polar Calibration | todo | M4 | | E1 | [M5-sails-and-polar-calibration.md](PLAN/MILESTONES/M5-sails-and-polar-calibration.md) |
| M6 | Flat-Water Sandbox | todo | M5 | | E3 | [M6-flat-water-sandbox.md](PLAN/MILESTONES/M6-flat-water-sandbox.md) |
| M7 | Wind Field and Tacking | todo | M6 | | E2 | [M7-wind-field-and-tacking.md](PLAN/MILESTONES/M7-wind-field-and-tacking.md) |
| M8 | FFT Ocean and Waves | todo | M7 | | E2 | [M8-fft-ocean-and-waves.md](PLAN/MILESTONES/M8-fft-ocean-and-waves.md) |
| M9 | Sail Cloth and Telltales | todo | M7 | | E3 | [M9-sail-cloth-and-telltales.md](PLAN/MILESTONES/M9-sail-cloth-and-telltales.md) |
| M10 | Helm Feel and Instruments | todo | M7 | | E3 | [M10-helm-feel-and-instruments.md](PLAN/MILESTONES/M10-helm-feel-and-instruments.md) |
| M11 | Playtest Build | todo | M8, M9, M10 | | E3 | [M11-playtest-build.md](PLAN/MILESTONES/M11-playtest-build.md) |
| M12 | Playtest Tuning Pass | todo | M11 | | E3 | [M12-playtest-tuning-pass.md](PLAN/MILESTONES/M12-playtest-tuning-pass.md) |

# Epics

| ID | Name | Goal |
|----|------|------|
| E1 | Physics Core and Calibration | A headless, tested 6-DOF J/80 model whose steady-state speeds match the reference polar. |
| E2 | Wind and Ocean | A gusty, sheared wind field and an FFT sea up to ~3 m Hs that the boat genuinely sails through. |
| E3 | Presentation and Feedback | A playable Godot demo whose visuals, helm and instruments let a sailor read and trim the boat. |

# Open questions

_None._
