# M4 - Hull Resistance and Foils

Make the hull move through water believably: DSYHS 1998 residuary resistance plus ITTC-57
friction with heel corrections, hull side force and yaw damping, and the keel and rudder as
lifting foils with stall. The rudder is tiller-driven with a rate limit and reports its hinge
moment for later helm feel. Proven by tow tests against Python-VPP's hull model and plausible
turning behaviour.

## Scope

**In:**
- Hull resistance and hull lift/damping modules, a generic foil module, keel and rudder instances, rudder hinge moment, `sailsim tow` and `sailsim turn`.

**Out:**
- Sails and wind (M5 - Sails and Polar Calibration).
- Wave added resistance and orbital inflow (M8 - FFT Ocean and Waves).
- Tiller feel and rumble — only the hinge moment number is produced here (M10 - Helm Feel and Instruments).
- Motor/prop walk (out of the demo).

**Edge cases:**
- Sternway (boat going backwards) → resistance and foil models defined for inflow from astern; rudder lift reverses and is weaker.
- Zero speed → foil and resistance forces go to zero smoothly (no division by speed).
- Rudder past stall (>~20° AoA) → post-stall Cl/Cd table, lift drops, drag rises.

**Assumptions:**
- The J/80 sits inside the DSYHS 1998 regression's parameter range (checked in M4.P1.T1 against hull_hydro.json).

## Phase 4.1: Hull resistance

- [ ] M4.P1.T1 — DSYHS residuary and frictional resistance
  - files: core/include/sail/hull_resistance.hpp, core/src/hull_resistance.cpp, core/tests/test_hull_resistance.cpp
  - approach: Implement Keuning & Sonnenberg 1998 upright residuary resistance (coefficients per Principles of Yacht Design / ORC VPP docs) as a function of Fn and hull parameters from `hull_hydro.json`; ITTC-57 frictional resistance with a form factor; the DSYHS heeled-resistance delta. Pure function first (`resistance(speed, heel)`), with a range check that warns if hull parameters fall outside the regression range.
  - verify: doctest: resistance at Fn 0.2/0.35/0.45 matches Python-VPP's hull resistance for the same J/80 inputs within 5% (export reference numbers from `tools/vpp/` into the test).
  - size: M
- [ ] M4.P1.T2 — Hull force module: resistance, side force and yaw damping
  - files: core/include/sail/hull_forces.hpp, core/src/hull_forces.cpp, core/tests/test_hull_forces.cpp
  - approach: `ForceModule` applying M4.P1.T1's resistance opposite the hull's horizontal velocity at the centre of lateral resistance, plus a canoe-body side force from leeway (DSYHS heeled side-force terms, low aspect lifting body) and its induced drag, plus linear+quadratic sway and yaw damping. Smooth at zero speed and valid in sternway. The integrator already includes the added-mass Coriolis and Munk-moment terms (Kirchhoff formulation, M3 decision), so this module must not add a Munk moment or any other added-mass-derived term again.
  - verify: doctest: towed at constant velocity, the module's drag equals M4.P1.T1's function; a hull drifting sideways at 1 m/s decelerates monotonically; no NaN at zero speed.
  - size: M

- [ ] M4.P1.T3 — Decide and implement heel/immersion dependence of added mass
  - files: core/src/mass_model.cpp, core/include/sail/mass_model.hpp, core/tests/test_mass_model.cpp
  - approach: Added mass is a constant diagonal from upright strip estimates (`strip_added_mass`). Decide, with the hull forces in place, whether sway/heave/roll added mass should vary with heel and draft (e.g. re-evaluate the strip estimate from the immersed sections at the current pose, updated at a low rate), or stay constant with the limitation documented. If it varies, update `BodyProperties` through the existing mid-run update path (`Simulation::set_body`) and keep it consistent with the integrator's Kirchhoff added-mass terms. Record the decision in this milestone's Decisions.
  - verify: doctest: if implemented, added mass equals the upright value at 0° heel and changes smoothly and monotonically with heel; the J/80 drop and knockdown tests still pass. If kept constant, the decision and its rationale are recorded and `strip_added_mass` documents the limitation.
  - size: M

## Phase 4.2: Appendage foils

- [ ] M4.P2.T1 — Generic lifting-surface foil module
  - files: core/include/sail/foil.hpp, core/src/foil.cpp, core/tests/test_foil.cpp, data/foils/naca0012.csv
  - approach: A `Foil` with span, chord distribution, sweep, area, effective aspect ratio (doubled for hull/free-surface mirroring, configurable), and a section Cl/Cd table over ±180° AoA (NACA 00xx with post-stall flat-plate behaviour). Finite-wing lift slope by Helmbold, induced drag Cl²/(π·AR_eff·e). Local inflow = body velocity + ω × r at the foil's quarter-chord centre (flows from any direction). Returns force and centre of pressure; usable for keel, rudder and later sails' structure.
  - verify: doctest: low-AoA lift slope within 2% of Helmbold for AR 2 and 5; stall at the table's angle; force zero at zero inflow; symmetric section gives zero lift at zero AoA.
  - size: L
- [ ] M4.P2.T2 — Keel and rudder with tiller input
  - files: core/include/sail/appendages.hpp, core/src/appendages.cpp, core/include/sail/controls.hpp, core/tests/test_appendages.cpp
  - approach: Instantiate keel and rudder `Foil`s from `appendages.json`. Rudder angle from a `Controls` struct field `tiller` (−1..1 → ±35°) through a rate limit (~60°/s). Apply a downwash factor from the keel to the rudder's inflow angle (simple ε = k·α_keel, k configurable). Both are `ForceModule`s.
  - verify: doctest: at 5 kn with 3° leeway the keel produces side force of the expected sign and magnitude (hand-computed); the rudder angle follows a step tiller command at the rate limit.
  - size: M
- [ ] M4.P2.T3 — Rudder hinge moment output
  - files: core/src/appendages.cpp, core/include/sail/appendages.hpp, core/tests/test_appendages.cpp
  - approach: Compute the rudder's hinge moment about its stock from the rudder force and its centre of pressure (moves aft past stall). Expose it in the sim state as `rudder_hinge_moment_Nm` for M10.
  - verify: doctest: hinge moment is zero at zero AoA, grows with AoA, and changes sign behaviour past stall as the CP moves aft.
  - size: M

## Phase 4.3: Validation

- [ ] M4.P3.T1 — Tow and turning harnesses
  - files: cli/cmd_tow.cpp, cli/cmd_turn.cpp, core/tests/test_j80_tow.cpp
  - approach: `sailsim tow --speeds 1..7 --heel H --leeway L` holds the boat at the given attitude, applies a horizontal tow force solved to steady speed, and prints resistance and side force vs speed as JSON. `sailsim turn --speed 5 --tiller 1` gives the boat an initial speed with a constant thrust matching resistance and full tiller, and reports turning circle diameter and time to 90° heading change. ctest checks the J/80's upright resistance at 3/5/6 kn against Python-VPP within 5%.
  - verify: `ctest -R j80_tow` passes; `sailsim turn` diameter is 2–4 boat lengths at full rudder, 5 kn (record the value in the PR body).
  - size: M

**Verification gate:** `ctest --preset dev` passes including `j80_tow`; `sailsim tow` resistance at 3/5/6 kn upright is within 5% of Python-VPP; `sailsim turn` gives a 2–4 LOA turning circle with no NaNs in sternway or at zero speed.

## Decisions

- 2026-11-27 — Tiller steering, no wheel: the J/80 is tiller-steered.
- 2026-11-28 — Inbox change-request (from the M3 - Rigid Body and Hydrostatics review) applied: added M4.P1.T3 to decide heel/immersion-dependent added mass.
