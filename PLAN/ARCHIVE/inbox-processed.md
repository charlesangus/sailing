### 2026-11-28T00:10:00Z — change-request
- refs: M4
- Deferred from the M3 - Rigid Body and Hydrostatics review: added mass is a constant diagonal from upright strip estimates (`strip_added_mass` in core/src/mass_model.cpp), so it does not change with heel or immersion. When M4 - Hull Resistance and Foils adds hydrodynamic forces, decide whether added mass should depend on heel and draft, and keep it consistent with the integrator, which already includes added-mass Coriolis and Munk terms through its Kirchhoff formulation.
