## Pending

### 2026-11-27T17:30:00Z — change-request
- refs: M2
- New milestone, suggested name "Hull Form Stability Fix". The morphed J/80 hull (data/boats/j80/hull_physics.obj, from tools/hull/morph.py, DSYHS model 61) is about 13% too stiff at small heel: with the certificate's measured VCG (VCGM −0.076 m, below the flotation plane) its RM at 1° is 300 N·m against the certificate's 264.8 N·m. The waterplane area already matches the certificate (8.767 vs 8.771 m²), so the excess is waterplane transverse inertia — the morph's BWL/flare distribution. M3 - Rigid Body and Hydrostatics works around it by solving the hull/deck residual's CG height (~0.96 m, physically doubtful) to VCG +0.067 m. Goal: reshape the morph (e.g. a narrower waterline toward the ends at constant area, or a different parent) so the mesh reproduces RM at 1° with VCG near VCGM, then drop `vcg_rm_calibrated` and re-run the hydrostatics regression and the Python-VPP polar. Depends on M3.
