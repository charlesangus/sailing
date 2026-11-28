#!/usr/bin/env python3
"""Compare a `sailsim polar` CSV with the J80 dossier polar targets.

Target selection by true wind angle (see data/boats/j80/README.md):
  beat row, fixed TWA <= --downwind-from: ORC certificate (orc_polar.csv).
  fixed TWA > --downwind-from, run row:   Python-VPP main+jib (pyvpp_polar_nospi.csv),
      because the certificate has no non-spinnaker downwind table.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_TWS = (6.0, 10.0, 16.0)
ORC = "ORC certificate"
PYVPP = "Python-VPP"


def read_target(path):
    """Returns {"fixed": {tws: [(twa, bsp)]}, "beat": {tws: (twa, bsp)}, "run": ...}."""
    out = {"fixed": {}, "beat": {}, "run": {}}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            tws, twa, bsp = float(row["tws_kn"]), float(row["twa_deg"]), float(row["bsp_kn"])
            if row["kind"] == "fixed":
                out["fixed"].setdefault(tws, []).append((twa, bsp))
            else:
                out[row["kind"]][tws] = (twa, bsp)
    for rows in out["fixed"].values():
        rows.sort()
    return out


def read_sim(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rec = {k: row[k] for k in row}
            for k in ("tws_kn", "twa_deg", "bsp_kn", "heel_deg", "leeway_deg", "rudder_deg",
                      "mainsheet", "traveler", "backstay", "jib_sheet", "jib_car"):
                rec[k] = float(row[k])
            rec["settled"] = int(float(row["settled"]))
            rows.append(rec)
    return rows


def _interp_tws(table, tws, fn):
    """Linear in TWS between the bracketing table wind speeds; None outside the table."""
    keys = sorted(table)
    if not keys or tws < keys[0] - 1e-9 or tws > keys[-1] + 1e-9:
        return None
    for lo, hi in zip(keys, keys[1:]):
        if lo - 1e-9 <= tws <= hi + 1e-9:
            a, b = fn(table[lo]), fn(table[hi])
            if a is None or b is None:
                return None
            w = (tws - lo) / (hi - lo)
            return a + w * (b - a)
    return fn(table[keys[0]])


def _interp_twa(rows, twa):
    if not rows or twa < rows[0][0] - 1e-9 or twa > rows[-1][0] + 1e-9:
        return None
    return float(np.interp(twa, [r[0] for r in rows], [r[1] for r in rows]))


class Targets:
    def __init__(self, boat, downwind_from):
        boat = Path(boat)
        self.orc = read_target(boat / "orc_polar.csv")
        self.pyvpp = read_target(boat / "pyvpp_polar_nospi.csv")
        self.downwind_from = downwind_from

    def source_for_fixed(self, twa):
        return PYVPP if twa > self.downwind_from + 1e-9 else ORC

    def fixed_bsp(self, tws, twa):
        src = self.source_for_fixed(twa)
        table = (self.orc if src == ORC else self.pyvpp)["fixed"]
        return src, _interp_tws(table, tws, lambda rows: _interp_twa(rows, twa))

    def optimum(self, kind, tws):
        """(source, twa, bsp) of the beat or run optimum, or None outside the table."""
        src, table = (ORC, self.orc) if kind == "beat" else (PYVPP, self.pyvpp)
        t = table[kind]
        twa = _interp_tws(t, tws, lambda r: r[0])
        bsp = _interp_tws(t, tws, lambda r: r[1])
        return None if twa is None else (src, twa, bsp)


def _bad(v):
    return v is None or (isinstance(v, float) and math.isnan(v))


def evaluate(sim_rows, targets, tws_list, bsp_tol, beat_angle_tol):
    """Returns a list of per-TWS results; each has `points`, `beat`, `run`, `passed`."""
    results = []
    for tws in tws_list:
        rows = [r for r in sim_rows if abs(r["tws_kn"] - tws) < 1e-6]
        res = {"tws": tws, "points": [], "beat": None, "run": None, "missing": []}
        for r in sorted((r for r in rows if r["kind"] == "fixed"), key=lambda r: r["twa_deg"]):
            src, tgt = targets.fixed_bsp(tws, r["twa_deg"])
            res["points"].append(_point(r, src, tgt, bsp_tol))
        for kind in ("beat", "run"):
            found = [r for r in rows if r["kind"] == kind]
            opt = targets.optimum(kind, tws)
            if not found:
                res["missing"].append(kind)
                continue
            r = found[0]
            if opt is None:
                res[kind] = _point(r, PYVPP if kind == "run" else ORC, None, bsp_tol)
                res[kind]["twa_err"] = None
                continue
            src, ttwa, tbsp = opt
            p = _point(r, src, tbsp, bsp_tol)
            p["target_twa"] = ttwa
            p["twa_err"] = None if _bad(r["twa_deg"]) else r["twa_deg"] - ttwa
            if kind == "beat":
                p["pass"] = p["pass"] and p["twa_err"] is not None and abs(p["twa_err"]) <= beat_angle_tol
            res[kind] = p
        res["passed"] = (
            not res["missing"]
            and all(p["pass"] for p in res["points"])
            and all(res[k]["pass"] for k in ("beat", "run"))
        )
        results.append(res)
    return results


def _point(r, src, tgt, bsp_tol):
    sim = r["bsp_kn"]
    unsettled = r["settled"] != 1 or _bad(sim)
    err = None if (_bad(tgt) or _bad(sim) or tgt == 0) else (sim - tgt) / tgt
    ok = not unsettled and err is not None and abs(err) <= bsp_tol
    return {"twa": r["twa_deg"], "source": src, "target": tgt, "sim": sim, "err": err,
            "pass": ok, "unsettled": unsettled, "row": r}


def _f(v, fmt):
    return "n/a" if _bad(v) else format(v, fmt)


def _pct(v):
    return "n/a" if v is None else f"{100 * v:+.1f}"


def _status(p):
    if p["unsettled"]:
        return "FAIL (unsettled)"
    if p["target"] is None:
        return "FAIL (no target)"
    return "pass" if p["pass"] else "FAIL"


def _ctx(r):
    return (f"{_f(r['heel_deg'], '.1f')} | {_f(r['leeway_deg'], '.1f')} | {_f(r['rudder_deg'], '.1f')} | "
            f"{_f(r['mainsheet'], '.2f')} / {_f(r['traveler'], '.2f')} / {_f(r['backstay'], '.2f')} / "
            f"{_f(r['jib_sheet'], '.2f')} / {_f(r['jib_car'], '.2f')}")


def render(results, args, png_name):
    ok = all(r["passed"] for r in results)
    L = ["# J/80 polar calibration", "",
         f"Overall: **{'PASS' if ok else 'FAIL'}**", "",
         "## Targets", "",
         f"- Beat and fixed TWA up to {args.downwind_from:g} deg: {ORC} (`orc_polar.csv`, 2019 J/80 certificate).",
         f"- Fixed TWA above {args.downwind_from:g} deg and the run optimum: {PYVPP} main + jib "
         "(`pyvpp_polar_nospi.csv`). The certificate has no non-spinnaker velocity table and its "
         "downwind numbers are spinnaker numbers.",
         "- Targets are interpolated linearly in TWA, then in TWS.", "",
         "## Criteria", "",
         f"- BSP within {100 * args.bsp_tol:g}% of target at every point, including the beat and run optima.",
         f"- Beat angle within {args.beat_angle_tol:g} deg of the target beat angle.",
         "- Run angle error is reported only.",
         "- An unsettled sim point, a NaN BSP, a point with no target, or a missing beat or run row is a fail.", "",
         f"![polar]({png_name})", ""]
    L += ["## Summary", "", "| TWS kn | beat TWA err deg | beat BSP err % | run TWA err deg | run BSP err % | result |",
          "|---|---|---|---|---|---|"]
    for r in results:
        b, u = r["beat"], r["run"]
        L.append(f"| {r['tws']:g} | {_f(b and b['twa_err'], '+.1f')} | {_pct(b and b['err'])} | "
                 f"{_f(u and u['twa_err'], '+.1f')} | {_pct(u and u['err'])} | "
                 f"{'pass' if r['passed'] else 'FAIL'} |")
    L.append("")
    for r in results:
        L += [f"## TWS {r['tws']:g} kn", ""]
        if r["missing"]:
            L += [f"Missing sim rows: {', '.join(r['missing'])} (fail).", ""]
        L += ["| Point | TWA deg | Target | Target kn | Sim kn | Error % | Pass | Heel deg | Leeway deg | "
              "Rudder deg | Main / trav / backstay / jib sheet / jib car |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        rows = []
        if r["beat"]:
            rows.append(("beat", r["beat"]))
        rows += [("fixed", p) for p in r["points"]]
        if r["run"]:
            rows.append(("run", r["run"]))
        for kind, p in rows:
            L.append(f"| {kind} | {_f(p['twa'], '.1f')} | {p['source']} | {_f(p['target'], '.2f')} | "
                     f"{_f(p['sim'], '.2f')} | {_pct(p['err'])} | {_status(p)} | {_ctx(p['row'])} |")
        if r["beat"] and r["beat"].get("target_twa") is not None:
            L += ["", f"Beat angle: sim {_f(r['beat']['twa'], '.1f')}, target {r['beat']['target_twa']:.1f} deg."]
        if r["run"] and r["run"].get("target_twa") is not None:
            L += [f"Run angle: sim {_f(r['run']['twa'], '.1f')}, target {r['run']['target_twa']:.1f} deg."]
        L.append("")
    return "\n".join(L)


def plot(results, targets, sim_rows, path):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    colors = plt.cm.viridis(np.linspace(0.1, 0.85, max(len(results), 1)))
    for res, c in zip(results, colors):
        tws = res["tws"]
        beat = targets.optimum("beat", tws)
        lo = beat[1] if beat else 30.0
        twas = np.arange(math.ceil(lo), 181, 2.0)
        vals = [targets.fixed_bsp(tws, a)[1] for a in twas]
        pts = [(a, v) for a, v in zip(twas, vals) if v is not None]
        if pts:
            ax.plot(np.radians([p[0] for p in pts]), [p[1] for p in pts], "-", color=c, label=f"target {tws:g} kn")
        sim = [r for r in sim_rows if abs(r["tws_kn"] - tws) < 1e-6 and r["settled"] == 1 and not _bad(r["bsp_kn"])]
        sim.sort(key=lambda r: r["twa_deg"])
        if sim:
            ax.plot(np.radians([r["twa_deg"] for r in sim]), [r["bsp_kn"] for r in sim], "o--",
                    color=c, markersize=4, label=f"sim {tws:g} kn")
    ax.set_title("Boat speed (kn) vs TWA: sim and target")
    ax.legend(loc="lower left", bbox_to_anchor=(1.02, 0.0), fontsize=8)
    fig.savefig(path, dpi=110, bbox_inches="tight", metadata={"Software": None})
    plt.close(fig)


def run(args):
    sim = read_sim(args.sim)
    targets = Targets(args.boat, args.downwind_from)
    tws_list = [float(x) for x in args.tws.split(",")]
    results = evaluate(sim, targets, tws_list, args.bsp_tol, args.beat_angle_tol)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    png = out.with_suffix(".png")
    plot(results, targets, sim, png)
    out.write_text(render(results, args, png.name) + "\n")
    return 0 if all(r["passed"] for r in results) else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", required=True, help="sailsim polar CSV")
    ap.add_argument("--boat", default="data/boats/j80")
    ap.add_argument("--out", default="docs/calibration/j80-polar.md")
    ap.add_argument("--tws", default=",".join(f"{t:g}" for t in DEFAULT_TWS))
    ap.add_argument("--bsp-tol", type=float, default=0.05)
    ap.add_argument("--beat-angle-tol", type=float, default=2.0)
    ap.add_argument("--downwind-from", type=float, default=150.0,
                    help="fixed TWA above this uses Python-VPP instead of the ORC certificate")
    return run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
