# M8 - FFT Ocean and Waves

A Tessendorf FFT ocean from calm to ~3 m Hs (wind-coupled JONSWAP sea plus an optional swell),
shared between physics and rendering by generating the spectrum once on the CPU: the core runs
a coarse CPU FFT for wave-relative buoyancy, orbital velocity and slamming; Godot runs a
high-resolution GPU FFT for visuals from the same amplitudes. Proven by stability in 3 m seas,
plausible head-sea speed loss and surfing downwind.

## Scope

**In:**
- Core spectrum and CPU heightfield, wave-relative hydrostatics, pressure drag and slamming, orbital velocity in foil/hull inflow, head-sea and surfing checks.
- Godot GPU FFT, ocean surface mesh and shading with Jacobian whitecaps, hull water mask, sea-state settings, puff input carried over from M7, physics/visual consistency overlay.

**Out:**
- Wake, bow spray, interactive Kelvin waves (out of the demo).
- Shoaling or breaking waves near the shore (out of the demo).
- Flooding from green water (out of the demo).

**Edge cases:**
- Hs 0 → physics identical to M3's flat-water hydrostatics (regression).
- Boat airborne off a wave crest (bow out) → triangle classification handles fully emerged parts; slamming on re-entry is bounded.
- Camera below the surface → ocean renders its underside without artefacts or is clamped above water.
- Terrain meets ocean → ocean fades out under terrain; no wave rendering on land.

**Assumptions:**
- Two physics cascades at 64² capture the wavelengths that move a 26' boat; the third, short cascade is visual-only.
- Godot 4's RenderingDevice compute is fast enough for 3 × 256² cascades at 60 fps on the user's GPU.

## Phase 8.1: Spectrum and CPU heightfield

- [ ] M8.P1.T1 — Ocean spectrum and shared amplitudes
  - files: core/include/sail/ocean/spectrum.hpp, core/src/ocean/spectrum.cpp, core/tests/test_spectrum.cpp
  - approach: JONSWAP wind sea from TWS and fetch (Hs up to ~3 m) plus an optional swell (Hs, Tp, direction, spreading); cos-2s directional spreading. Generate seeded h0(k) complex amplitudes for 3 cascades (e.g. 512 m, 128 m, 32 m patches) at 256²; the physics uses the central 64² of the first two cascades, so physics and visuals share the low-wavenumber content exactly. Export amplitudes as flat float arrays for the GPU.
  - verify: doctest: Hs from integrating the spectrum and from the generated amplitudes matches the requested Hs within 5% for 0.5, 1.5 and 3 m; the same seed reproduces identical arrays.
  - size: L
- [ ] M8.P1.T2 — CPU FFT heightfield sampling
  - files: core/include/sail/ocean/heightfield.hpp, core/src/ocean/heightfield.cpp, core/tests/test_heightfield.cpp, core/CMakeLists.txt
  - approach: pocketfft (header-only, FetchContent). Each update evolves h0 to h(k,t) for the two physics cascades at 64² and IFFTs height, horizontal (choppy) displacement and orbital velocity at the surface; update at 50 Hz and interpolate between updates for the 200 Hz step. `sample(x, y, t)` returns height and orbital velocity at a world point, inverting choppy displacement with 2–3 fixed-point iterations; orbital velocity decays with depth as exp(kz) per cascade's dominant k.
  - verify: doctest: for a single-component spectrum the sample matches the analytic Gerstner/Airy wave within 1%; benchmark update + 2000 samples < 0.3 ms (Release).
  - size: L

## Phase 8.2: Physics coupling

- [ ] M8.P2.T1 — Wave-relative hydrostatics
  - files: core/src/hydrostatics.cpp, core/include/sail/hydrostatics.hpp, core/tests/test_hydrostatics.cpp
  - approach: Replace the flat plane with heightfield samples at each physics-mesh vertex; clip triangles against per-vertex water heights (Kerner) with pressure ρ·g·(local water height − point height). Keep the flat-water path as a fast special case when Hs = 0.
  - verify: doctest: Hs 0 gives results identical to M3's (bitwise or within 1e-9); in a single long swell the boat's heave follows the wave elevation with the expected phase at a wavelength ≫ LWL.
  - size: L
- [ ] M8.P2.T2 — Pressure drag, slamming and orbital inflow
  - files: core/include/sail/hydrodynamics.hpp, core/src/hydrodynamics.cpp, core/src/appendages.cpp, core/src/hull_forces.cpp, core/tests/test_hydrodynamics.cpp
  - approach: Kerner part 2: per-submerged-triangle pressure drag from the normal component of velocity relative to the water (including orbital velocity) with linear+quadratic coefficients; slamming force on triangles whose submerged area rises fast, capped. Add orbital velocity to keel/rudder inflow and to hull relative velocity, so surfing and wave-induced yaw emerge.
  - verify: doctest: 10 simulated minutes at Hs 3 m with a fixed tiller has no NaN and bounded energy; slamming peaks are capped.
  - size: L
- [ ] M8.P2.T3 — Head-sea and surfing validation
  - files: cli/cmd_polar.cpp, docs/calibration/waves.md
  - approach: Add `--hs --tp --wave-dir` to `sailsim polar`. Measure speed loss close-hauled at TWS 16 in Hs 1 m head seas vs flat water, and peak/mean BSP running in Hs 2.5 m following seas; compare to published added-resistance ranges for small yachts and document. Tune pitch damping only if motion is visibly springy or speed loss is outside the range.
  - verify: `docs/calibration/waves.md` records the measurements: head-sea speed loss inside the documented range; downwind runs show surfing bursts above hull speed.
  - size: M

## Phase 8.3: Rendering

- [ ] M8.P3.T1 — GPU FFT cascades in Godot
  - files: game/shaders/ocean/*.glsl, gdext/src/ocean_gpu.hpp, gdext/src/ocean_gpu.cpp, game/scripts/ocean.gd
  - approach: RenderingDevice compute pipeline: upload h0 for the 3 cascades from the core once (and when sea state changes); per frame evolve to h(k,t), Stockham IFFT, and write displacement, normal and Jacobian textures. Use the open-source GodotOceanWaves (MIT) project's structure as a reference, not a dependency.
  - verify: Headless can't render: `test_ocean_init.gd` checks the pipeline creates without errors where a RenderingDevice exists, skipping otherwise; a debug readback of one cascade's displacement texture at 4 points matches the core's CPU heightfield for the same cascade within 2 cm (run where a GPU exists, or via the `cloud-gpu` skill).
  - size: L
- [ ] M8.P3.T2 — Ocean surface mesh and shading
  - files: game/shaders/ocean_surface.gdshader, game/scripts/ocean_mesh.gd, game/scenes/environment.tscn
  - approach: Camera-centred clipmap grid (nested rings) displaced by the cascades; shading with Fresnel sky reflection, sun specular, subsurface tint on crests, whitecap foam from the Jacobian, distance blending to a far plane; fade out under terrain; take over M7's puff texture input. Replaces the flat water plane.
  - verify: Scene loads headless; UAT at Hs 0.3, 1.5 and 3 m for looks and no seams.
  - size: L
- [ ] M8.P3.T3 — Hull water mask
  - files: game/shaders/hull_mask.gdshader, game/scenes/boat.tscn, game/shaders/ocean_surface.gdshader
  - approach: Render the hull interior volume into a mask (depth pre-pass or stencil) so the ocean surface does not draw inside the cockpit or through the hull when heeled.
  - verify: UAT: no water inside the boat at 25° heel in Hs 1.5 m.
  - size: M
- [ ] M8.P3.T4 — Sea-state settings
  - files: data/wind/presets.json, game/scripts/settings_panel.gd, game/scenes/settings_panel.tscn, gdext/src/sail_boat.cpp
  - approach: "Sea" section: wind-sea coupling (fetch slider, on/off), swell Hs, Tp, direction; presets gain sea settings. Changes regenerate h0 on the CPU and re-upload to the GPU.
  - verify: `test_sea_settings.gd` headless: changing swell Hs changes the core spectrum's Hs.
  - size: M
- [ ] M8.P3.T5 — Physics/visual consistency overlay
  - files: game/scripts/ocean_debug.gd, gdext/src/sail_boat.cpp
  - approach: Debug toggle draws small markers at the core's physics sample heights around the boat, so any mismatch between the GPU surface and the physics surface is visible.
  - verify: UAT: markers sit on the rendered surface within a few cm in the large swells (short-cascade detail aside).
  - size: M

**Verification gate:** `ctest --preset dev` and `ctest --preset release -L slow` pass (incl. Hs-0 regression and the 3 m stability test); `docs/calibration/waves.md` committed; `tools/run_godot_tests.sh` passes; user UAT at Hs 0.3, 1.5 and 3 m: ocean looks right, consistency markers sit on the surface, no water in the hull, ≥ 60 fps.

## Decisions

- 2026-11-27 — Sea states calm to ~3 m Hs with swell: user's choice; brings slamming and surfing into scope.
- 2026-11-27 — No wake or spray: user chose ocean only; whitecaps come from the FFT Jacobian.
- 2026-11-27 — Spectrum generated once on the CPU, shared with the GPU: keeps physics independent of the GPU (Context rule) while what you see matches what the hull feels.
