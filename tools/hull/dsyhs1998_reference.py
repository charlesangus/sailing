#!/usr/bin/env python3
"""Independent evaluation of the DSYHS 1998 canoe-body resistance, as a reference.

Reads the coefficient tables from data/hull/dsyhs1998_coefficients.csv (provenance in
data/hull/README.md) and evaluates the Keuning & Sonnenberg 1998 regressions for

- DSYHS models 1 (Series 1 parent) and 44 (Series 4 parent), from data/dsyhs/table.csv
  at model scale, with LCB/LCF from the published series parameter table, and
- the J/80 at its sailing displacement (hull_hydro.json `conditions.sailing.result`),

then writes data/hull/dsyhs1998_reference.json. The core's `dsyhs` hull resistance
model is tested against that file. Standard library only:
`.venv/bin/python -I tools/hull/dsyhs1998_reference.py`.
"""

import argparse
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COEFFICIENTS = ROOT / "data" / "hull" / "dsyhs1998_coefficients.csv"
DSYHS_TABLE = ROOT / "data" / "dsyhs" / "table.csv"
J80 = ROOT / "data" / "boats" / "j80"
OUT = ROOT / "data" / "hull" / "dsyhs1998_reference.json"

G = 9.81
FN = [0.20, 0.35, 0.45]
HEEL_DEG = [0.0, 20.0, 35.0]

# LCBfpp/LCFfpp of the series hulls as published in Keuning & Katgert 2008 (reproduced
# in Kleijweg 2015, Table B.1). data/dsyhs/table.csv has no LCF column.
LCB_OVER_LCF = {1: 0.980, 44: 0.947}

# Water for the model-scale series hulls: ITTC 7.5-02-01-03 fresh water at 15 deg C.
FRESH_15C = {"rho": 999.1, "nu": 1.1386e-6}


def read_tables(path):
    tables = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            coeffs = [float(row[f"c{i}"]) for i in range(9) if row[f"c{i}"] != ""]
            tables.setdefault(row["table"], []).append((float(row["x"]), coeffs))
    return tables


def interp_rows(rows, x, below, above):
    """Linear in x. below/above: 'zero' (a zero row at x = 0), 'hold' or 'extrapolate'."""
    xs = [r[0] for r in rows]
    if x <= xs[0]:
        if below == "zero":
            t = max(x, 0.0) / xs[0]
            return [t * c for c in rows[0][1]]
        return list(rows[0][1])
    if x >= xs[-1]:
        if above == "hold":
            return list(rows[-1][1])
        (x0, c0), (x1, c1) = rows[-2], rows[-1]
    else:
        i = max(k for k in range(len(xs)) if xs[k] <= x)
        (x0, c0), (x1, c1) = rows[i], rows[i + 1]
    t = (x - x0) / (x1 - x0)
    return [a + t * (b - a) for a, b in zip(c0, c1)]


def upright_terms(h):
    v13 = h["vol"] ** (1.0 / 3.0)
    v23 = h["vol"] ** (2.0 / 3.0)
    lcb_l = h["lcb_fpp"] / h["lwl"]
    s = v13 / h["lwl"]
    return [1.0,
            s * lcb_l, s * h["cp"], s * v23 / h["aw"], s * h["bwl"] / h["lwl"],
            s * v23 / h["sc"], s * h["lcb_fpp"] / h["lcf_fpp"], s * lcb_l ** 2,
            s * h["cp"] ** 2]


def heel20_terms(h):
    btr = h["bwl"] / h["tc"]
    lcb_pct = 100.0 * (0.5 - h["lcb_fpp"] / h["lwl"])
    return [1.0, h["lwl"] / h["bwl"], btr, btr ** 2, lcb_pct, lcb_pct ** 2]


def evaluate(tables, h, fn, heel_deg):
    v = fn * math.sqrt(G * h["lwl"])
    weight = h["vol"] * h["rho"] * G
    a = interp_rows(tables["upright"], fn, "zero", "extrapolate")
    rr_ratio = sum(c * t for c, t in zip(a, upright_terms(h)))
    phi = math.radians(abs(heel_deg))
    u = interp_rows(tables["heel20_x1000"], fn, "zero", "hold")
    drr20_ratio = 1e-3 * sum(c * t for c, t in zip(u, heel20_terms(h)))
    drr = 6.0 * phi ** 1.7 * drr20_ratio * weight
    btr = h["bwl"] / h["tc"]
    s = interp_rows([(0.0, [0.0] * 4)] + tables["wetted_heel"], abs(heel_deg), "hold", "hold")
    sc_heel = h["sc"] * (1.0 + (s[0] + s[1] * btr + s[2] * btr ** 2 + s[3] * h["cm"]) / 100.0)
    re = max(1e4, 0.7 * v * h["lwl"] / h["nu"])
    cf = 0.075 / (math.log10(re) - 2.0) ** 2
    rf = 0.5 * h["rho"] * v * v * sc_heel * cf * h["form_factor"]
    rr = rr_ratio * weight
    residuary = max(0.0, rr + drr)
    return {"fn": fn, "heel_deg": heel_deg, "speed_mps": v,
            "rr_over_weight": rr_ratio, "upright_residuary_N": rr,
            "heel_residuary_delta_N": drr, "residuary_N": residuary,
            "wetted_area_m2": sc_heel, "cf": cf, "viscous_N": rf,
            "total_N": residuary + rf}


def series_hull(model):
    with open(DSYHS_TABLE, newline="") as f:
        row = next(r for r in csv.DictReader(f)
                   if int(r["hull_id"]) == model and float(r["overhang"]) == 1.0)
    lwl = float(row["lwl_m"])
    lcb_fpp = float(row["lcb_frac_lwl"]) * lwl
    return {"name": f"DSYHS model {model}", "lwl": lwl, "bwl": float(row["bwl_m"]),
            "tc": float(row["tc_m"]), "vol": float(row["vol_m3"]),
            "aw": float(row["aw_m2"]), "sc": float(row["sc_m2"]),
            "cp": float(row["cp"]), "cm": float(row["cm"]),
            "lcb_fpp": lcb_fpp, "lcf_fpp": lcb_fpp / LCB_OVER_LCF[model],
            "form_factor": 1.0, **FRESH_15C}


def j80_hull():
    r = json.loads((J80 / "hull_hydro.json").read_text())["conditions"]["sailing"]["result"]
    res = json.loads((J80 / "hull_resistance.json").read_text())
    water = res["water"]
    rho = water["density"]["value"]
    form_factor = res["dsyhs"].get("form_factor", {"value": 1.0})["value"]
    lcb_fpp = r["lcb_frac_lwl_aft_fp"] * r["lwl"]
    x_fp = r["lcb_x_canoe"] + lcb_fpp
    return {"name": "J/80 sailing condition", "lwl": r["lwl"], "bwl": r["bwl"],
            "tc": r["tc"], "vol": r["canoe_volume"], "aw": r["waterplane_area"],
            "sc": r["wetted_area_canoe"], "cp": r["cp"], "cm": r["cm"],
            "lcb_fpp": lcb_fpp, "lcf_fpp": x_fp - r["lcf_x"], "form_factor": form_factor,
            "rho": rho, "nu": water["dynamic_viscosity"]["value"] / rho}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    tables = read_tables(COEFFICIENTS)
    hulls = []
    for h in [series_hull(1), series_hull(44), j80_hull()]:
        inputs = {k: v for k, v in h.items() if k != "name"}
        points = [evaluate(tables, h, fn, heel) for fn in FN for heel in HEEL_DEG]
        hulls.append({"name": h["name"], "inputs": inputs, "points": points})
    out = {
        "description": "DSYHS 1998 canoe-body resistance (Keuning & Sonnenberg 1998 "
                       "regressions) evaluated independently of the core, for the core's "
                       "dsyhs hull resistance model tests.",
        "written_by": "tools/hull/dsyhs1998_reference.py",
        "coefficients": "data/hull/dsyhs1998_coefficients.csv",
        "method": [
            "Rr / (vol rho g) = a0 + (a1 LCBfpp/Lwl + a2 Cp + a3 vol^(2/3)/Aw + a4 Bwl/Lwl "
            "+ a5 vol^(2/3)/Sc + a6 LCBfpp/LCFfpp + a7 (LCBfpp/Lwl)^2 + a8 Cp^2) "
            "vol^(1/3)/Lwl; LCBfpp and LCFfpp in m aft of the FP; coefficients linear in "
            "Fn between rows, zero at Fn 0, extrapolated linearly above Fn 0.60.",
            "dRr(20 deg) / (vol rho g) = 1e-3 (u0 + u1 Lwl/Bwl + u2 Bwl/Tc + u3 (Bwl/Tc)^2 "
            "+ u4 LCB + u5 LCB^2), LCB in % Lwl from Lwl/2, positive forward; linear in Fn, "
            "zero at Fn 0, held above Fn 0.55; dRr(phi) = 6.0 phi^1.7 dRr(20 deg), phi in "
            "rad.",
            "Sc(phi) = Sc (1 + (s0 + s1 Bwl/Tc + s2 (Bwl/Tc)^2 + s3 Cm) / 100), linear in "
            "heel between rows, zero at 0 deg, held above 35 deg.",
            "Rf = 0.5 rho V^2 Sc(phi) Cf (1 + k), Cf = 0.075 / (log10 Re - 2)^2, "
            "Re = 0.7 Lwl V / nu, 1 + k = 1.",
            "Residuary = max(0, Rr + dRr(phi)): the regression gives Rr slightly below zero "
            "around Fn 0.10.",
            "V = Fn sqrt(g Lwl), g = 9.81 m/s^2. All hull parameters are upright.",
        ],
        "hulls": hulls,
    }
    args.out.write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
