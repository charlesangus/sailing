import csv
import json
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import compare_polar as cp  # noqa: E402

BOAT = Path(__file__).resolve().parents[2] / "data" / "boats" / "j80"
HEADER = ["tws_kn", "twa_deg", "bsp_kn", "vmg_kn", "heel_deg", "leeway_deg", "rudder_deg", "settled",
          "mainsheet", "traveler", "backstay", "jib_sheet", "jib_car", "reef", "evaluations",
          "sim_time_s", "kind"]
TWS = (6.0, 10.0, 16.0)


def target_rows(scale=1.0):
    t = cp.Targets(BOAT, 75.0)
    rows = []
    for tws in TWS:
        for kind in ("beat", "run"):
            _, twa, bsp = t.optimum(kind, tws)
            rows.append((tws, twa, bsp * scale, kind))
        for twa, _ in t.orc["fixed"][tws]:
            if twa <= 75:
                rows.append((tws, twa, t.fixed_bsp(tws, twa)[1] * scale, "fixed"))
        for twa in (90.0, 110.0, 135.0, 160.0, 170.0):
            rows.append((tws, twa, t.fixed_bsp(tws, twa)[1] * scale, "fixed"))
    return rows


def write_sim(path, rows, unsettled=()):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for i, (tws, twa, bsp, kind) in enumerate(rows):
            settled = 0 if i in unsettled else 1
            w.writerow([tws, twa, bsp, 0, -10, 2, 1, settled, .7, 0, 0, .6, .5, 0, 10, 100, kind])


def run(tmp, name, rows, extra=(), unsettled=()):
    sim = Path(tmp) / f"{name}.csv"
    write_sim(sim, rows, unsettled)
    out = Path(tmp) / f"{name}.md"
    code = cp.main(["--sim", str(sim), "--boat", str(BOAT), "--out", str(out), *extra])
    return code, out.read_text()


def test_all():
    with tempfile.TemporaryDirectory() as tmp:
        code, md = run(tmp, "exact", target_rows())
        assert code == 0, md
        assert "Overall: **PASS**" in md
        assert (Path(tmp) / "exact.png").stat().st_size > 0
        code2, md2 = run(tmp, "exact", target_rows())
        assert md == md2

        code, md = run(tmp, "low", target_rows(0.9))
        assert code == 1 and "FAIL" in md

        rows = target_rows()
        code, md = run(tmp, "unsettled", rows, unsettled={3})
        assert code == 1 and "unsettled" in md

        rows = target_rows()
        rows[0] = (rows[0][0], rows[0][1] + 2.5, rows[0][2], "beat")
        code, md = run(tmp, "angle", rows)
        assert code == 0 and "error +2.5 deg" in md
        code, md = run(tmp, "angle_gated", rows, extra=("--gate-beat-angle", "2"))
        assert code == 1
        code, _ = run(tmp, "angle_gated_ok", rows, extra=("--gate-beat-angle", "3"))
        assert code == 0

        rows = target_rows()
        rows[0] = (rows[0][0], rows[0][1], rows[0][2] * 0.93, "beat")
        code, md = run(tmp, "beat_slow", rows)
        assert code == 1

        nan_rows = target_rows()
        nan_rows[2] = (*nan_rows[2][:2], math.nan, nan_rows[2][3])
        code, _ = run(tmp, "nan", nan_rows)
        assert code == 1

        t = cp.Targets(BOAT, 75.0)
        assert t.source_for_fixed(75.0) == cp.ORC and t.source_for_fixed(90.0) == cp.PYVPP and t.source_for_fixed(110.0) == cp.PYVPP
        code, md = run(tmp, "default_cut", target_rows())
        assert "up to 75 deg" in md
        assert "| fixed | 110.0 | Python-VPP |" in md and "| fixed | 90.0 | Python-VPP |" in md
        assert "| fixed | 75.0 | ORC certificate |" in md
        assert "## Accepted residuals" in md and "## Known differences" in md

        acc = Path(tmp) / "accepted.json"
        entry = {"tws": 6, "point": "run", "metric": "bsp", "max_error_pct": 6.0, "reason": "owner test"}
        acc.write_text(json.dumps([entry]))
        rows = target_rows()
        rows[1] = (rows[1][0], rows[1][1], rows[1][2] * 1.051, "run")
        none = Path(tmp) / "none.json"
        none.write_text("[]")
        code, md = run(tmp, "run_unaccepted", rows, extra=("--accepted", str(none)))
        assert code == 1
        code, md = run(tmp, "run_accepted", rows, extra=("--accepted", str(acc)))
        assert code == 0 and "| accepted |" in md and "| yes | owner test |" in md

        rows[1] = (rows[1][0], rows[1][1], rows[1][2] * 1.065 / 1.051, "run")
        code, md = run(tmp, "run_beyond", rows, extra=("--accepted", str(acc)))
        assert code == 1 and "| no | owner test |" in md

        one = [(10.0, 90.0, 6.4082, "fixed")]
        code, md = run(tmp, "one", one, extra=("--tws", "10"))
        assert code == 1 and "Missing sim rows: beat, run" in md


if __name__ == "__main__":
    test_all()
    print("ok")
