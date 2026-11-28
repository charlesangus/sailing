# Polar calibration

`compare_polar.py` compares a `sailsim polar` CSV with the J/80 dossier polars and writes
`docs/calibration/j80-polar.md` and `j80-polar.png`. Exit code 0 means every requested TWS passes, 1 means a fail.

    .venv/bin/pip install -r tools/calib/requirements.txt
    .venv/bin/python -I tools/calib/compare_polar.py --sim polar.csv --boat data/boats/j80 \
        [--out docs/calibration/j80-polar.md] [--tws 6,10,16] [--bsp-tol 0.05] \
        [--beat-angle-tol 2] [--downwind-from 150]

Targets: beat and fixed TWA up to `--downwind-from` come from the ORC certificate; fixed TWA above it and the
run optimum come from Python-VPP (the certificate's downwind numbers are spinnaker numbers).

Missing beat or run rows, unsettled points, NaN BSP and points outside the target table are failures.

Test: `.venv/bin/python -I tools/calib/test_compare_polar.py`
