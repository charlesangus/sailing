# M11 - Playtest Build

A packaged Linux build the user plays to judge realism, with a deterministic session recorder
so any odd moment can be replayed and inspected, a start menu for conditions, a playtest
checklist and a performance pass. The user is the only playtester; their notes feed
M12 - Playtest Tuning Pass.

## Scope

**In:**
- Session recorder + deterministic replay, start menu and settings persistence, Linux export and packaging script, playtest checklist, performance pass.

**Out:**
- Distribution to other testers, itch.io pages, in-game feedback forms (user is the only playtester).
- Tuning changes from playtest findings (M12 - Playtest Tuning Pass).
- Windows/macOS exports (out of the demo).

**Edge cases:**
- Replay of a recording from a different build → refuse with a clear message (build hash stored in the file).
- Settings file missing or corrupt → defaults, no crash.

**Assumptions:**
- The user's machine has a GPU able to run the demo at ≥ 60 fps at 1080p; the perf target is measured there at UAT.

## Phase 11.1: Recording and replay

- [ ] M11.P1.T1 — Deterministic session recorder and replay
  - files: core/include/sail/recorder.hpp, core/src/recorder.cpp, core/tests/test_recorder.cpp, cli/cmd_replay.cpp, gdext/src/sail_boat.cpp, game/scripts/recording.gd
  - approach: Record build hash, boat dir, all settings and seeds, and the per-physics-step `Controls` (delta-encoded) to a compact binary file; periodic state checksums. `sailsim replay <file>` re-runs headless and verifies checksums; in-game replay mode drives `SailBoat` from the file with free camera. Recording toggled from the menu (on by default in the playtest build).
  - verify: doctest: a 5-minute scripted session replays with identical final state and all checksums matching; a file with a mismatched build hash is refused.
  - size: L

## Phase 11.2: Build and menu

- [ ] M11.P2.T1 — Start menu and settings persistence
  - files: game/scenes/start_menu.tscn, game/scripts/start_menu.gd, game/scripts/settings_store.gd
  - approach: Start menu: wind preset, sea state, start position (harbour mouth, open bay), crew mode, start/replay buttons, a controls help overlay for both devices. Persist settings to `user://settings.cfg` with defaults on missing/corrupt file.
  - verify: `test_settings_store.gd` headless: save/load round-trips; a corrupt file loads defaults.
  - size: M
- [ ] M11.P2.T2 — Linux export and packaging
  - files: game/export_presets.cfg, tools/package.sh
  - approach: Linux x86_64 release export preset; `tools/package.sh` builds the release preset (core + gdext), runs `godot --headless --export-release`, and writes `dist/sailing-<version>-linux.tar.gz` with the dossier data and a README for launching.
  - verify: `tools/package.sh` produces the tarball; extracting it elsewhere and running the binary with `--headless --quit-after 300` exits 0.
  - size: M
- [ ] M11.P2.T3 — Playtest checklist
  - files: docs/playtest/checklist.md
  - approach: A checklist the user works through: beat to a windward mark and back in each preset; tack and gybe in auto and manual crew modes; reef in a puff; sail through Hs 1.5 m and 3 m upwind and down; run aground and get off; for each, what to note (feel of helm, pointing, acceleration, heel, sail/telltale readability) and where to write notes (`docs/playtest/notes-<date>.md`) and which recordings to keep.
  - verify: The checklist covers every scenario named here.
  - size: S

## Phase 11.3: Performance

- [ ] M11.P3.T1 — Performance pass
  - files: game/scripts/perf_overlay.gd, docs/playtest/perf.md
  - approach: A perf overlay (frame ms, physics µs per step, cloth ms, ocean GPU ms). Profile the worst case (Hs 3 m, puffy preset, helm and chase cameras) and fix whatever is over budget; record numbers in `docs/playtest/perf.md`.
  - verify: `docs/playtest/perf.md` records ≥ 60 fps at 1080p in the worst case on the user's machine (confirmed at UAT).
  - size: M

**Verification gate:** `ctest` (dev and release `slow`) and `tools/run_godot_tests.sh` pass; `tools/package.sh` produces a working tarball; user UAT: install the tarball, work through `docs/playtest/checklist.md`, and commit notes to `docs/playtest/notes-<date>.md`.

## Decisions

- 2026-11-27 — The user is the only playtester: no distribution or feedback tooling; a recorder and replay instead, so notes can point at exact moments.
