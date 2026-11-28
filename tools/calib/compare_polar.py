#!/usr/bin/env python3
"""Compare a `sailsim polar` CSV with the J80 dossier polar targets.

Target selection by true wind angle (see data/boats/j80/README.md):
  beat row, fixed TWA <= --downwind-from: ORC certificate (orc_polar.csv).
  fixed TWA > --downwind-from, run row:   Python-VPP main+jib (pyvpp_polar_nospi.csv),
      because the certificate has no non-spinnaker velocity table and its 90 deg
      speed appears to carry a spinnaker.

Every TWA is the course's: the angle between the true wind and the boat's track through
the water, heading plus leeway. Both references tabulate that angle (Python-VPP's wind
triangle adds the boat speed along the TWA axis, AeroMod.wind_triangle), and their VMG is
BSP * cos(TWA). The sim CSV's twa_deg is the course TWA its autopilot held, twa_heading_deg
the heading's, and vmg_kn the CG velocity along the true wind, which the beat and run VMG
are scored on.
"""
import argparse
import csv
import json
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


SIM_FLOATS = ("tws_kn", "twa_deg", "twa_heading_deg", "bsp_kn", "vmg_kn", "heel_deg", "leeway_deg",
              "rudder_deg", "mainsheet", "traveler", "backstay", "jib_sheet", "jib_car")


def read_sim(path):
    """Raises ValueError when a column is missing: a CSV without twa_heading_deg gives the
    heading TWA in twa_deg, which cannot be compared with the course-TWA references."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [k for k in (*SIM_FLOATS, "settled", "kind") if k not in (reader.fieldnames or ())]
        if missing:
            raise ValueError(f"{path}: missing columns {', '.join(missing)}; regenerate it with sailsim polar")
        for row in reader:
            rec = {k: row[k] for k in row}
            for k in SIM_FLOATS:
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


def evaluate(sim_rows, targets, tws_list, bsp_tol, gate_beat_angle=None, accepted=()):
    """Returns a list of per-TWS results; each has `points`, `beat`, `run`, `passed`."""
    results = []
    for tws in tws_list:
        rows = [r for r in sim_rows if abs(r["tws_kn"] - tws) < 1e-6]
        res = {"tws": tws, "points": [], "beat": None, "run": None, "missing": []}
        for r in sorted((r for r in rows if r["kind"] == "fixed"), key=lambda r: r["twa_deg"]):
            src, tgt = targets.fixed_bsp(tws, r["twa_deg"])
            p = _point(r, src, tgt, bsp_tol)
            _accept(p, tws, r["twa_deg"], accepted)
            res["points"].append(p)
        for kind in ("beat", "run"):
            found = [r for r in rows if r["kind"] == kind]
            opt = targets.optimum(kind, tws)
            if not found:
                res["missing"].append(kind)
                continue
            r = found[0]
            if opt is None:
                p = _point(r, PYVPP if kind == "run" else ORC, None, bsp_tol)
                p["twa_err"] = None
                res[kind] = p
                continue
            src, ttwa, tbsp = opt
            p = _point(r, src, tbsp, bsp_tol)
            p["target_twa"] = ttwa
            p["twa_err"] = None if _bad(r["twa_deg"]) else r["twa_deg"] - ttwa
            _score_vmg(p, r, kind, ttwa, tbsp)
            if kind == "beat":
                _gate_beat(p, bsp_tol, gate_beat_angle)
            _accept(p, tws, kind, accepted)
            res[kind] = p
        res["passed"] = (
            not res["missing"]
            and all(p["pass"] for p in res["points"])
            and all(res[k]["pass"] for k in ("beat", "run"))
        )
        results.append(res)
    return results


def _score_vmg(p, r, kind, ttwa, tbsp):
    """VMG toward the wind on the beat and away from it on the run, both positive."""
    sign = 1.0 if kind == "beat" else -1.0
    p["target_vmg"] = sign * tbsp * math.cos(math.radians(ttwa))
    p["sim_vmg"] = None if _bad(r["vmg_kn"]) else sign * r["vmg_kn"]
    p["vmg_err"] = None
    if p["sim_vmg"] is not None and p["target_vmg"] > 0:
        p["vmg_err"] = (p["sim_vmg"] - p["target_vmg"]) / p["target_vmg"]


def _gate_beat(p, bsp_tol, gate_beat_angle):
    """Beat VMG is gated like BSP; the beat angle only when `gate_beat_angle` is given."""
    if not p["unsettled"] and (p["vmg_err"] is None or abs(p["vmg_err"]) > bsp_tol):
        p["fails"].append("vmg")
    if gate_beat_angle is not None and not p["unsettled"] and (
            p["twa_err"] is None or abs(p["twa_err"]) > gate_beat_angle):
        p["fails"].append("beat_angle")
    p["pass"] = not p["fails"]


def _point(r, src, tgt, bsp_tol):
    sim = r["bsp_kn"]
    unsettled = r["settled"] != 1 or _bad(sim)
    err = None if (_bad(tgt) or _bad(sim) or tgt == 0) else (sim - tgt) / tgt
    fails = []
    if unsettled:
        fails.append("unsettled")
    elif err is None:
        fails.append("no target")
    elif abs(err) > bsp_tol:
        fails.append("bsp")
    return {"twa": r["twa_deg"], "source": src, "target": tgt, "sim": sim, "err": err,
            "pass": not fails, "fails": fails, "unsettled": unsettled, "accepted": [], "row": r}


def _within(entry, p):
    """|error| up to the entry's max bound; with min_error_pct or min_error_deg, the signed
    error between min and max instead, so a residual cannot flip sign and stay accepted."""
    metric = entry["metric"]
    err = p["err"] if metric == "bsp" else p.get("twa_err")
    if err is None:
        return False
    if metric == "bsp":
        err, hi, lo = 100 * err, entry["max_error_pct"], entry.get("min_error_pct")
    else:
        hi, lo = entry["max_error_deg"], entry.get("min_error_deg")
    return abs(err) <= hi if lo is None else lo <= err <= hi


def _same_point(entry_point, point):
    if isinstance(point, str):
        return entry_point == point
    return not isinstance(entry_point, str) and abs(float(entry_point) - point) < 1e-6


def _accept(p, tws, point, accepted):
    """A point whose every failure is inside an accepted bound stops failing. Unsettled,
    no-target and VMG failures have no metric an entry can name, so they never qualify."""
    if not p["fails"]:
        return
    hits = []
    for metric in p["fails"]:
        match = [i for i, e in enumerate(accepted)
                 if e["metric"] == metric and abs(e["tws"] - tws) < 1e-6
                 and _same_point(e["point"], point) and _within(e, p)]
        if not match:
            return
        hits.append(match[0])
    p["accepted"] = hits
    p["pass"] = True


def load_accepted(path):
    """Reads the accepted-residual list; raises ValueError on a malformed entry."""
    entries = json.loads(Path(path).read_text())
    if not isinstance(entries, list):
        raise ValueError(f"{path}: expected a JSON list")
    for e in entries:
        bound = {"bsp": "max_error_pct", "beat_angle": "max_error_deg"}.get(e.get("metric"))
        if bound is None:
            raise ValueError(f"{path}: metric must be 'bsp' or 'beat_angle': {e}")
        for key in ("tws", "point", bound, "reason"):
            if key not in e:
                raise ValueError(f"{path}: entry missing '{key}': {e}")
        low = bound.replace("max_", "min_")
        if low in e and not e[low] <= e[bound]:
            raise ValueError(f"{path}: '{low}' above '{bound}': {e}")
    return entries


def _f(v, fmt):
    return "n/a" if _bad(v) else format(v, fmt)


def _pct(v):
    return "n/a" if v is None else f"{100 * v:+.1f}"


def _status(p):
    if p["unsettled"]:
        return "FAIL (unsettled)"
    if p["target"] is None:
        return "FAIL (no target)"
    if p["pass"] and p["accepted"]:
        return "accepted"
    return "pass" if p["pass"] else "FAIL"


def _ctx(r):
    return (f"{_f(r['twa_heading_deg'], '.1f')} | {_f(r['heel_deg'], '.1f')} | {_f(r['leeway_deg'], '.1f')} | "
            f"{_f(r['rudder_deg'], '.1f')} | "
            f"{_f(r['mainsheet'], '.2f')} / {_f(r['traveler'], '.2f')} / {_f(r['backstay'], '.2f')} / "
            f"{_f(r['jib_sheet'], '.2f')} / {_f(r['jib_car'], '.2f')}")


def _point_label(point):
    return point if isinstance(point, str) else f"{float(point):g} deg"


def _entry_bound(e):
    unit, hi, lo = (("%", e["max_error_pct"], e.get("min_error_pct")) if e["metric"] == "bsp"
                    else (" deg", e["max_error_deg"], e.get("min_error_deg")))
    return f"{hi:g}{unit}" if lo is None else f"{lo:+g} to {hi:+g}{unit}"


def render(results, args, png_name, accepted=()):
    ok = all(r["passed"] for r in results)
    used = {i for r in results for p in [*r["points"], r["beat"], r["run"]] if p for i in p["accepted"]}
    gate = ("not gated (reported only)" if args.gate_beat_angle is None
            else f"within {args.gate_beat_angle:g} deg of the target beat angle")
    L = ["# J/80 polar calibration", "",
         f"Overall: **{'PASS' if ok else 'FAIL'}**", "",
         "## Targets", "",
         "- Every TWA is the course's, between the true wind and the track through the water (heading plus "
         "leeway), as both references tabulate it; the sim's heading TWA is listed for reference. VMG is the "
         "sim's `vmg_kn`, its velocity along the true wind.",
         f"- Beat and fixed TWA up to {args.downwind_from:g} deg: {ORC} (`orc_polar.csv`, 2019 J/80 certificate).",
         f"- Fixed TWA above {args.downwind_from:g} deg and the run optimum: {PYVPP} main + jib "
         "(`pyvpp_polar_nospi.csv`). The certificate has no non-spinnaker velocity table and its "
         "90 deg speed appears to carry a spinnaker.",
         "- Targets are interpolated linearly in TWA, then in TWS.", "",
         "## Criteria", "",
         f"- BSP within {100 * args.bsp_tol:g}% of target at every point, including the beat and run optima.",
         f"- Beat VMG within {100 * args.bsp_tol:g}% of the certificate beat VMG.",
         f"- Beat angle: {gate}.",
         "- Run angle and run VMG errors are reported only.",
         "- An unsettled sim point, a NaN BSP, a point with no target, or a missing beat or run row is a fail.",
         "- A BSP or beat-angle miss inside an accepted residual is reported as accepted and does not fail.", "",
         "## Accepted residuals", ""]
    if accepted:
        L += ["| TWS kn | Point | Metric | Bound | Used | Reason |", "|---|---|---|---|---|---|"]
        for i, e in enumerate(accepted):
            L.append(f"| {e['tws']:g} | {_point_label(e['point'])} | {e['metric']} | {_entry_bound(e)} | "
                     f"{'yes' if i in used else 'no'} | {e['reason']} |")
    else:
        L.append("None.")
    L += ["", "## Known differences", "",
          "- The beat angle is reported, not gated. The sim's beat angles are 1 to 3 deg wider than the "
          "certificate's: both report TWA to the course over water (heading plus leeway), the Python-VPP and "
          "ORC convention.",
          "- Upwind VMG is about 4.5% below the certificate at 6 to 10 kn, from pointing rather than boat speed.",
          "- The certificate assumes ORC's default 411 kg crew; the sim sails the 350 kg class maximum, which "
          "costs about 2 to 3% at 16 to 20 kn upwind and reaching.",
          f"- The 90 deg and downwind target is {PYVPP} main + jib rather than the certificate, whose 90 deg "
          "speed appears to carry a spinnaker.", "",
          f"![polar]({png_name})", ""]
    L += ["## Summary", "",
          "| TWS kn | beat TWA err deg | beat BSP err % | beat VMG err % | run TWA err deg | run BSP err % | "
          "run VMG err % | result |",
          "|---|---|---|---|---|---|---|---|"]
    for r in results:
        b, u = r["beat"], r["run"]
        allp = [*r["points"], b, u]
        res = "FAIL" if not r["passed"] else ("pass (accepted)" if any(p and p["accepted"] for p in allp) else "pass")
        L.append(f"| {r['tws']:g} | {_f(b and b['twa_err'], '+.1f')} | {_pct(b and b['err'])} | "
                 f"{_pct(b and b.get('vmg_err'))} | {_f(u and u['twa_err'], '+.1f')} | {_pct(u and u['err'])} | "
                 f"{_pct(u and u.get('vmg_err'))} | {res} |")
    L.append("")
    for r in results:
        L += [f"## TWS {r['tws']:g} kn", ""]
        if r["missing"]:
            L += [f"Missing sim rows: {', '.join(r['missing'])} (fail).", ""]
        L += ["| Point | TWA deg | Target | Target kn | Sim kn | Error % | Pass | Heading TWA deg | Heel deg | "
              "Leeway deg | Rudder deg | Main / trav / backstay / jib sheet / jib car |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
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
            b = r["beat"]
            L += ["", f"Beat angle: sim {_f(b['twa'], '.1f')}, target {b['target_twa']:.1f} deg "
                      f"(error {_f(b['twa_err'], '+.1f')} deg). "
                      f"Beat VMG: sim {_f(b.get('sim_vmg'), '.2f')}, target {b['target_vmg']:.2f} kn "
                      f"(error {_pct(b.get('vmg_err'))} %)."]
        if r["run"] and r["run"].get("target_twa") is not None:
            u = r["run"]
            L += [f"Run angle: sim {_f(u['twa'], '.1f')}, target {u['target_twa']:.1f} deg. "
                  f"Run VMG: sim {_f(u.get('sim_vmg'), '.2f')}, target {u['target_vmg']:.2f} kn "
                  f"(error {_pct(u.get('vmg_err'))} %)."]
        for kind, p in rows:
            for i in p["accepted"]:
                e = accepted[i]
                L += ["", f"Accepted ({kind} {_f(p['twa'], '.1f')} deg, {e['metric']}): {e['reason']}."]
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
    accepted_path = args.accepted
    if accepted_path is None and (Path(args.boat) / "calibration_accepted.json").is_file():
        accepted_path = Path(args.boat) / "calibration_accepted.json"
    accepted = load_accepted(accepted_path) if accepted_path else []
    results = evaluate(sim, targets, tws_list, args.bsp_tol, args.gate_beat_angle, accepted)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    png = out.with_suffix(".png")
    plot(results, targets, sim, png)
    out.write_text(render(results, args, png.name, accepted) + "\n")
    return 0 if all(r["passed"] for r in results) else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", required=True, help="sailsim polar CSV")
    ap.add_argument("--boat", default="data/boats/j80")
    ap.add_argument("--out", default="docs/calibration/j80-polar.md")
    ap.add_argument("--tws", default=",".join(f"{t:g}" for t in DEFAULT_TWS))
    ap.add_argument("--bsp-tol", type=float, default=0.05)
    ap.add_argument("--gate-beat-angle", type=float, default=None, metavar="DEG",
                    help="also fail when the beat angle is more than DEG from the certificate")
    ap.add_argument("--accepted", default=None,
                    help="JSON list of accepted residuals (default: <boat>/calibration_accepted.json if present)")
    ap.add_argument("--downwind-from", type=float, default=75.0,
                    help="fixed TWA above this uses Python-VPP instead of the ORC certificate")
    return run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
