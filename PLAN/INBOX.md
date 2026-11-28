## Pending

### 2026-11-27T17:30:00Z — change-request
- refs: M2
- New milestone, suggested name "Hull Form Stability Fix". The morphed J/80 hull (data/boats/j80/hull_physics.obj, from tools/hull/morph.py, DSYHS model 61) is about 13% too stiff at small heel: with the certificate's measured VCG (VCGM −0.076 m, below the flotation plane) its RM at 1° is 300 N·m against the certificate's 264.8 N·m. The waterplane area already matches the certificate (8.767 vs 8.771 m²), so the excess is waterplane transverse inertia — the morph's BWL/flare distribution. M3 - Rigid Body and Hydrostatics keeps the physical VCG at VCGM and accepts the ~13% excess stiffness (user decision). Goal: reshape the morph (e.g. a narrower waterline toward the ends at constant area, or a different parent) so the mesh reproduces RM at 1° within 5% at VCGM, then tighten the M3 RM regression back to ±5% and re-run the Python-VPP polar and any sail calibration done against the stiffer hull. Depends on M3.

### 2026-11-28T01:00:00Z — change-request
- refs: none
- New work for M4 - Hull Resistance and Foils' follow-up (or a later milestone): compute the ORC VPP 2023 documented hull lengths and coefficients in tools/hull — LSM1/LSM4/LSM6 (depth-attenuated sectional-area second moments, eqs. 4.11/4.12, l=2, t=0.25, a=10) and the dynamic L of Table 6.1, effective B/T (eqs. 4.13–4.15), and heeled wetted area/BTR/LVR tables — and write them as the optional `orc` fields in data/boats/j80/hull_hydro.json that the `orc` hull resistance model already reads when present. The Appendix B neural-network residuary half stays out unless its output scaling (`CALC_CFxP_FN_Fz_LPP22t105c`) can be resolved. Re-run the hull resistance tests and the Python-VPP comparison afterwards.
