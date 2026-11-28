import csv
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
    t = cp.Targets(BOAT, 150.0)
    rows = []
    for tws in TWS:
        for kind in ("beat", "run"):
            _, twa, bsp = t.optimum(kind, tws)
            rows.append((tws, twa, bsp * scale, kind))
        for twa, _ in t.orc["fixed"][tws]:
            if twa <= 150:
                rows.append((tws, twa, t.fixed_bsp(tws, twa)[1] * scale, "fixed"))
        for twa in (160.0, 170.0):
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
        rows[0] = (rows[0][0], rows[0][1] + 3.0, rows[0][2], "beat")
        code, _ = run(tmp, "angle", rows)
        assert code == 1

        nan_rows = target_rows()
        nan_rows[2] = (*nan_rows[2][:2], math.nan, nan_rows[2][3])
        code, _ = run(tmp, "nan", nan_rows)
        assert code == 1

        one = [(10.0, 90.0, 6.4082, "fixed")]
        code, md = run(tmp, "one", one, extra=("--tws", "10"))
        assert code == 1 and "Missing sim rows: beat, run" in md


if __name__ == "__main__":
    test_all()
    print("ok")
