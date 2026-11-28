# Polar calibration

`compare_polar.py` compares a `sailsim polar` CSV with the J/80 dossier polars and writes
`docs/calibration/j80-polar.md` and `j80-polar.png`. Exit code 0 means every requested TWS passes, 1 means a fail.

    .venv/bin/pip install -r tools/calib/requirements.txt
    .venv/bin/python -I tools/calib/compare_polar.py --sim polar.csv --boat data/boats/j80 \
        [--out docs/calibration/j80-polar.md] [--tws 6,10,16] [--bsp-tol 0.05] \
        [--downwind-from 75] [--gate-beat-angle DEG] [--accepted FILE]

Targets: the beat and fixed TWA up to `--downwind-from` (default 75 deg) come from the ORC certificate; fixed TWA
above it and the run optimum come from Python-VPP main + jib. The certificate's 90 deg speed appears to carry a
spinnaker, and the J/80 here sails main + jib only.

TWA and VMG: every TWA is the course's, between the true wind and the boat's track through the water (heading
plus leeway). That is the angle the ORC VPP and Python-VPP tabulate: Python-VPP's `AeroMod.wind_triangle` adds
the boat speed along the TWA axis and models leeway separately, and its VMG is `BSP * cos(TWA)`. The sim CSV's
`twa_deg` is the course TWA its autopilot held, `twa_heading_deg` the heading's, and `vmg_kn` its velocity along
the true wind. A CSV without `twa_heading_deg` predates that and is rejected; regenerate it.

Beat: BSP and VMG (`vmg_kn`) must each be within `--bsp-tol` of the certificate's beat BSP and VMG. The beat angle
and its error are reported but not gated unless `--gate-beat-angle DEG` is given, because the references disagree
on it: Python-VPP with the same ORC sail method sits 3 to 9 deg below the certificate from 8 kn up. Run VMG is
reported only.

Accepted residuals: `--accepted FILE` (default `<boat>/calibration_accepted.json` if it exists) is a JSON list of
`{tws, point, metric, max_error_pct | max_error_deg, reason}`. `point` is `"run"`, `"beat"` or a fixed TWA in deg;
`metric` is `"bsp"` (bound `max_error_pct`) or `"beat_angle"` (bound `max_error_deg`). A point that misses its
tolerance but is within the entry's bound is reported as accepted and does not fail; anything beyond the bound
still fails. An optional `min_error_pct` or `min_error_deg` makes the bound signed, `min <= error <= max`, so a
residual that changes sign is no longer accepted. The report lists every entry and whether it was used.

Missing beat or run rows, unsettled points, NaN BSP and points outside the target table are failures.

Test: `.venv/bin/python -I tools/calib/test_compare_polar.py`
