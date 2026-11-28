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

Beat: BSP and VMG must each be within `--bsp-tol` of the certificate's beat BSP and VMG. The beat angle and its
error are reported but not gated unless `--gate-beat-angle DEG` is given. Beat angles come out narrower than the
certificate's; Python-VPP with the same ORC sail method is also 3 to 9 deg narrower.

Accepted residuals: `--accepted FILE` (default `<boat>/calibration_accepted.json` if it exists) is a JSON list of
`{tws, point, metric, max_error_pct | max_error_deg, reason}`. `point` is `"run"`, `"beat"` or a fixed TWA in deg;
`metric` is `"bsp"` (bound `max_error_pct`) or `"beat_angle"` (bound `max_error_deg`). A point that misses its
tolerance but is within the entry's bound is reported as accepted and does not fail; anything beyond the bound
still fails. The report lists every entry and whether it was used.

Missing beat or run rows, unsettled points, NaN BSP and points outside the target table are failures.

Test: `.venv/bin/python -I tools/calib/test_compare_polar.py`
