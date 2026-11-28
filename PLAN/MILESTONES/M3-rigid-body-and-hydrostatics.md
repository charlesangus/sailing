# M3 - Rigid Body and Hydrostatics

The 6-DOF rigid body and flat-water hydrostatics: integrator at a fixed 200 Hz, added mass,
a mass model with a movable crew point mass, per-triangle buoyancy against the physics hull
mesh valid at any heel up to 180°, and damping. Proven by the J/80 floating at the right draft
and trim and reproducing the certificate's righting-moment figures.

## Scope

**In:**
- Rigid body, simulation stepping, mass model, OBJ loading, per-triangle hydrostatics on flat water, heave/roll/pitch damping, `sailsim hydro|gz|bench`.
- Knockdowns and self-righting to 180° heel.

**Out:**
- Flooding, swamping, water ingress (out of the demo).
- Waves (M8 - FFT Ocean and Waves).
- Hull resistance and foils (M4 - Hull Resistance and Foils).
- Auto-hiking logic — this milestone only exposes the crew position (M5 - Sails and Polar Calibration).

**Edge cases:**
- Fully submerged or fully emerged triangles → whole-triangle fast paths; triangles cut by the plane → clipped (Kerner).
- Boat inverted (180°) → buoyancy still correct; whether it self-rights from there follows from the GZ curve (stable inverted only past the LPS).
- Large frame hitch (real dt > 0.1 s) → the accumulator clamps substeps and drops time rather than spiralling.

**Assumptions:**
- A constant diagonal added-mass matrix (Fossen-style estimates from hull dims) is adequate; no frequency dependence.
- The certificate gives RM at 1° and LPS; if not, the GZ targets come from the morphed hull's own computed curve cross-checked against J/80 class stability figures.

## Phase 3.1: Rigid body

- [x] M3.P1.T1 — Rigid-body state and integrator
  - files: core/include/sail/rigid_body.hpp, core/src/rigid_body.cpp, core/include/sail/simulation.hpp, core/src/simulation.cpp, core/tests/test_rigid_body.cpp
  - approach: Eigen types. State: world position, orientation quaternion, body-frame linear and angular velocity. Mass, body inertia tensor, 6×6 added-mass matrix (diagonal for now). Body-frame force/moment accumulator. Semi-implicit Euler at dt = 1/200 s with the full gyroscopic term (ω × Iω) and quaternion renormalisation. `Simulation::advance(real_dt)` runs fixed steps through an accumulator (max 20 substeps, excess dropped) and exposes an interpolation alpha. Force modules plug in through a `ForceModule` interface (`accumulate(const State&, double t, Wrench&)`), so later milestones add modules without touching the integrator.
  - verify: doctest: free fall matches g·t²/2 within 0.1%; torque-free spin about a principal axis conserves angular momentum within 0.1% over 60 s; an intermediate-axis spin tumbles (Dzhanibekov) rather than staying fixed; accumulator gives the same state for 100×0.01 s and 1000×0.001 s calls.
  - size: L
- [x] M3.P1.T2 — Mass model with movable crew
  - files: core/include/sail/mass_model.hpp, core/src/mass_model.cpp, core/include/sail/boat_data.hpp, core/src/boat_data.cpp, core/tests/test_mass_model.cpp
  - approach: Load `data/boats/j80/mass.json` (nlohmann/json via FetchContent). Give every item a body-frame CG (convert the provisional stem/flotation frame with `hull_hydro.json` `stem_x_in_body`/`flotation_z_in_body`); mast CG = heel z + MCG (IMS F8.2: MCG is measured from the mast's lower point, not the sheer — run_pyvpp.py currently adds it to the sheer); don't count standing rigging twice where MWT already includes it (IMS F8.1(a)). The `hull_deck_fittings` residual takes mass = 1438 − Σ(other in-displacement items) and its z is solved so the total VCG equals the certificate VCGM (−0.076 m, i.e. 0.076 m below the flotation plane, z up); fail loudly if the solved z leaves 0.3–1.1 m above the plane (expect about +0.67 m). Update the data files to match (mass.json residual value, cg, source, confidence `derived`, method; a corrected `vcgm` note in particulars.json saying it is below the plane, z up, and that the morphed mesh is ~13% too stiff at that VCG), and correct the dossier's wrong "low waterplane inertia / low form stability" wording — the mesh is too stiff — in sources.md, README.md and the generator text in tools/vpp/run_pyvpp.py with its check file. Combine items into total mass, CG and inertia (parallel-axis, each item as a point or box). Added-mass estimates from hull_hydro.json dims (surge ~5% of mass, sway/heave and rotational terms per Fossen's slender-body approximations) — expose the coefficients in config. `set_crew_position(y, z)` moves the crew point mass and recomputes CG and inertia; the crew mass is also settable, so the ORC stability-trim configuration (default crew 411 kg plus gear 65.8 kg on the centreline at 0.708 m above the flotation plane, ORC VPP Documentation 2023) can be built for the AVS test.
  - verify: doctest: total mass equals the dossier; moving crew to the rail shifts CG by crew_mass/total × offset; inertia stays symmetric positive definite.
  - size: M

## Phase 3.2: Hydrostatics

- [x] M3.P2.T1 — Per-triangle hydrostatics on flat water
  - files: core/include/sail/mesh.hpp, core/src/mesh.cpp, core/include/sail/hydrostatics.hpp, core/src/hydrostatics.cpp, core/tests/test_hydrostatics.cpp
  - approach: OBJ loader into flat preallocated arrays (vertices in body frame). Each step transform vertices to world, compute heights above z = 0, classify triangles (all under / cut / all over), split cut triangles into submerged sub-triangles (Kerner part 1), and apply hydrostatic pressure ρ·g·depth at each sub-triangle's centroid along its inward normal — but integrate with the exact centre-of-pressure depth so the force is right for coarse triangles. Sum force and moment about CG as a `ForceModule`. Loads `hull_physics.obj` + `appendages.obj`. ρ = 1025.
  - verify: doctest: a unit box and an icosphere give analytic buoyant force and centre of buoyancy within 0.5% at 0°, 30°, 90° and 150° heel and part-submerged; force is zero when fully emerged.
  - size: L
- [x] M3.P2.T2 — Hydrostatic damping
  - files: core/include/sail/damping.hpp, core/src/damping.cpp, core/tests/test_damping.cpp, data/boats/j80/damping.json
  - approach: A `ForceModule` with linear + quadratic damping on heave, roll and pitch (body frame), coefficients from `damping.json`. Start roll damping from a fraction of critical computed from roll inertia and initial GM, heave from waterplane area and added mass; document the formulas inline in the data file's comments field.
  - verify: doctest: J/80 dropped 0.1 m into flat water settles with a heave natural period within 10% of the analytic value from waterplane area and mass+added mass, and decays to <5% within 5 periods without overshoot growth.
  - size: M
- [ ] M3.P2.T3 — Hydrostatics CLI tools
  - files: cli/cmd_hydro.cpp, cli/cmd_gz.cpp, cli/main.cpp
  - approach: `sailsim hydro --boat data/boats/j80` settles the boat in flat water (heavy damping) and prints draft, trim, displaced volume, LCB as JSON. `sailsim gz --boat … --step 5` for heel 0–180°: hold heel, let sinkage and trim settle, record righting moment and GZ; output JSON (heel, rm_Nm, gz_m) and the angle of vanishing stability.
  - verify: Both commands run on the J/80 and emit valid JSON; `gz` GZ is ~0 at 0° and changes sign once at the reported AVS.
  - size: M

## Phase 3.3: Validation

- [ ] M3.P3.T1 — J/80 hydrostatic and stability regression tests
  - files: core/tests/test_j80_hydrostatics.cpp
  - approach: doctest cases loading the real dossier, at measurement trim (1438 kg, no crew). At rest: displaced mass equals dossier displacement within 1%, trim within 0.5° of level DWL, draft matches dossier within 2%. Stability: report RM at 1° against the certificate (expected about +13%: the morphed hull is too stiff, accepted by user decision) and assert it lies between 0% and +20%. AVS in the ORC stability-trim configuration (VCG at VCGM, crew and gear per the mass model's ORC configuration, trim held level) within 5° of the certificate LPS 111.7° (consultant measured 111.6°); report measurement-trim AVS and free-trim AVS for information. Knockdown: released from 100° heel in flat water with heavy roll damping, the boat returns within 10° of upright within 20 s.
  - verify: `ctest --preset dev -R j80_hydro` passes.
  - size: M
- [ ] M3.P3.T2 — Step-cost benchmark
  - files: cli/cmd_bench.cpp, core/tests/test_bench_budget.cpp
  - approach: `sailsim bench` runs 10 000 steps of the J/80 at a 30° heel and reports mean and p99 µs per step, broken down per force module. A ctest asserts mean hydrostatics cost < 1 ms in a Release build (skip in Debug).
  - verify: `sailsim bench` prints the breakdown; the Release ctest passes.
  - size: M

**Verification gate:** `ctest --preset dev` passes including the J/80 hydrostatics tests; `sailsim gz` on the J/80 reports RM at 1° (expected ~+13% vs the certificate, accepted) and AVS in ORC stability trim within 5° of the certificate LPS; `sailsim bench` (Release) shows hydrostatics < 1 ms per step.

## Decisions

- 2026-11-27 — Valid to 180° heel, no flooding: user chose knockdowns and self-righting without a water-ingress model.
- 2026-11-27 — Crew as a movable point mass: user chose auto-hiking; this milestone exposes `set_crew_position`, M5 drives it.
- 2026-11-27 — Semi-implicit Euler at 200 Hz: one hydrostatics evaluation per step keeps cost low; the survey's stability advice is 100–200 Hz with damping.
- 2026-11-27 — ORC VCGM is z up from the flotation plane, so −0.076 m is below it; at that VCG the morphed mesh gives RM at 1° about 13% above the certificate — the morphed hull is too stiff (the M2 dossier wording "low form stability" is backwards). User decision (revised from calibrating the CG): keep the physical CG at VCGM and accept a boat ~13% stiffer than the reference at small heel, rather than a hull/deck CG at ~0.96 m. The RM-at-1° check is report-only; AVS is tested in ORC stability trim. M5 calibrates sails against this stiffer boat. A hull-shape fix is filed as follow-up work.
- 2026-11-27 — The hull/deck residual (708.8 kg) is solved in both x and z: z so the total VCG equals VCGM (+0.712 m above flotation), x so measurement-trim LCG equals the total LCB, because the hull was morphed to float level at exactly that condition. Mast CG is from the mast heel (IMS F8.2); the standing rigging double-count with MWT was removed. The Python-VPP polar now uses VCGM; upwind stays within 7.8% of the certificate.
- 2026-11-27 — Damping is specified as fractions of critical (damping.json) and derived at load from dossier stiffnesses: roll from the certificate RM at 1° (not the stiffer mesh, so simulated roll ζ is ~6% below nominal), pitch from an estimated waterplane longitudinal inertia. sailcore and its tests build at -O1 in Debug (asserts kept): simulation tests were ~25 ms/step at -O0.
- 2026-11-27 — The integrator solves body-frame Kirchhoff equations on total momentum (M_rb + M_A)·ν, so added-mass Coriolis and Munk-moment terms are included; later hydrodynamic modules must not add Munk-type terms again. The accumulator counts integer nanoseconds so time slicing cannot defer a step.
