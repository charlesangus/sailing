# M12 - Playtest Tuning Pass

> Stub — elaborate into phases/tasks before starting (PLAN-FORMAT.md §5).

Act on the user's playtest notes: retune coefficients, lags, damping, helm and visual feedback
where the boat feels wrong, without breaking the calibration gates (polar within 5%, tack
targets, weather helm). Each fix is driven by a note and, where possible, a recording from
M11's replay system that demonstrates the problem before and after.

Blocked on: M11 - Playtest Build's UAT — the user's notes in `docs/playtest/notes-<date>.md`
decide what gets tuned.

Acceptance sketch:
- Every playtest note is either fixed (with a before/after replay or harness result) or recorded as a deliberate won't-fix.
- `compare_polar.py`, the tack harness and the weather-helm check still pass after tuning.
- The user re-plays the checklist items they flagged and accepts the result.
