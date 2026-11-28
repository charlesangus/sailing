#!/usr/bin/env python3
"""Canoe-body hull resistance of a boat from the vendored Python-VPP, as a reference.

Evaluates Python-VPP's residuary (HydroMod._get_Rr) and viscous (HydroMod._get_Rv)
resistance of the bare canoe body for the hull measured at the sailing
displacement (hull_hydro.json `conditions.sailing.result`), at fixed Froude
numbers and heel angles, and writes data/boats/<boat>/hull_resistance_reference.json.
The core's ORC hull resistance model is tested against that file.

It also evaluates the upright, zero-leeway tow resistance at fixed speeds in knots,
of the canoe body alone and of the canoe body with the keel and rudder as
run_pyvpp.py gives them to Python-VPP, broken down by component. The core's towed
J/80 is tested against those.

Run tools/vpp/fetch_pyvpp.sh first, then
`.venv/bin/python -I tools/vpp/hull_reference.py`.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "tools" / "vpp" / "vendor" / "Python-VPP"
PYVPP_COMMIT = "9b9fe8f3f63d902a1df207691109e42932d0a9bb"

sys.path.insert(0, str(VENDOR / "src"))
from python_vpp import Keel, Rudder, Yacht  # noqa: E402
from python_vpp.HydroMod import HydroMod  # noqa: E402

FN = [0.20, 0.35, 0.45]
HEEL_DEG = [0.0, 20.0, 35.0]
TOW_KN = [3.0, 5.0, 6.0]
KNOT = 0.514444

DEVIATIONS = [
    {"item": "length",
     "orc_2023": "Fn on the dynamic length L, a Froude-number-dependent mix of RRLSM1, "
                 "RRLSM4 and RRLSM6 (section 6.3.1, Composite Length Calculation); "
                 "viscous Re on 0.85 LSM1 (eq. 6.2).",
     "here": "Lwl at the sailing displacement stands in for every one of them, for Fn, "
             "LVR and the viscous Reynolds number, unless the dossier supplies "
             "conditions.sailing.orc lsm1/lsm4/lsm6."},
    {"item": "BTR",
     "orc_2023": "Effective beam over effective hull depth, depth-attenuated integrals "
                 "(section 4, Beam Depth Ratio).",
     "here": "Bwl / Tc of the canoe body at the sailing displacement, unless the dossier "
             "supplies conditions.sailing.orc btr_effective or a heeled BTR table."},
    {"item": "neural-network half",
     "orc_2023": "Canoe-body Rr is the 50:50 mean of the drag surfaces and a 14-input "
                 "neural network (section 6.3).",
     "here": "Drag surfaces only; the neural-network half is not implemented."},
    {"item": "heel term",
     "orc_2023": "No immersed-rail multiplier on Rr; heel enters through heeled "
                 "hydrostatics.",
     "here": "Python-VPP's immersed-rail factor 1 + 0.0004 (phi_deg - 30)^2 for heel "
             "above 30 deg, 1 otherwise. Not in the ORC 2023 documentation; kept because "
             "the Python-VPP downwind target uses it. Upright hydrostatics otherwise."},
    {"item": "range",
     "orc_2023": "LVR outside 3..9 and BTR outside 2.5..9 default to the closest point of "
                 "the surface; Fn outside 0.125..0.7 is extrapolated.",
     "here": "Same in the core model (LVR and BTR clamped and flagged, Fn extrapolated "
             "linearly, with a zero row at Fn 0). Python-VPP instead extrapolates "
             "linearly in all three axes; the two agree whenever LVR and BTR are inside "
             "the grid, as they are for this boat."},
]


def v(d, *keys):
    for k in keys:
        d = d[k]
    return d["value"] if isinstance(d, dict) and "value" in d else d


def check_vendor():
    if not (VENDOR / "src" / "python_vpp").is_dir():
        sys.exit(f"hull_reference: {VENDOR} missing, run tools/vpp/fetch_pyvpp.sh")
    head = subprocess.run(["git", "-C", str(VENDOR), "rev-parse", "HEAD"],
                          capture_output=True, text=True, check=True).stdout.strip()
    if head != PYVPP_COMMIT:
        sys.exit(f"hull_reference: {VENDOR} is at {head}, expected {PYVPP_COMMIT}; "
                 "run tools/vpp/fetch_pyvpp.sh")


def appendage_inputs(boat):
    app = json.loads((boat / "appendages.json").read_text())
    keel, rudder = app["foils"]["keel"], app["foils"]["rudder"]
    ks = v(app, "keel", "sections")
    depth = [s["depth_below_root_m"] for s in ks]
    keel_tc = np.trapezoid([s["max_thickness_m"] for s in ks], depth) / np.trapezoid(
        [s["chord_m"] for s in ks], depth)

    # Python-VPP's trapezoid takes the root chord as given and the tip chord that makes
    # chord x span equal the real planform area, as run_pyvpp.py does.
    def trapezoid(foil, tc):
        return {"Cu": v(foil, "root_chord"),
                "Cl": 2 * v(foil, "planform_area") / v(foil, "span") - v(foil, "root_chord"),
                "Span": v(foil, "span"), "Tc_ratio": float(tc)}

    return {"keel": trapezoid(keel, keel_tc),
            "rudder": trapezoid(rudder, v(rudder, "thickness_chord_ratio_root"))}


def build_hydro(boat, appendages=None):
    part = json.loads((boat / "particulars.json").read_text())
    cond = json.loads((boat / "hull_hydro.json").read_text())["conditions"]["sailing"]["result"]
    zw = cond["sinkage_m"]
    inputs = {
        "Lwl": cond["lwl"], "Vol": cond["canoe_volume"], "Bwl": cond["bwl"],
        "Tc": cond["tc"], "WSA": cond["wetted_area_canoe"],
        "Tmax": v(part, "draft") + zw, "Amax": cond["max_section_area"],
        "Mass": cond["displacement_kg"], "Loa": v(part, "loa"), "Boa": v(part, "beam"),
        "Ff": v(part, "freeboard_fwd_ffm") - zw, "Fa": v(part, "freeboard_aft_fam") - zw,
    }
    # App=[] leaves the keel and bulb Rr and every appendage viscous term out, so the
    # result is the canoe body alone. GZ only feeds the righting moment, unused here.
    app = [] if appendages is None else [Keel(**appendages["keel"]),
                                         Rudder(**appendages["rudder"])]
    yacht = Yacht(Name=v(part, "name"), App=app, Sails=[],
                  GZ={"Heel": [0.0, 90.0], "GZ": [0.0, 0.0]}, **inputs)
    return HydroMod(yacht), inputs


def evaluate(hm, fn, heel_deg):
    hm.lsm, hm.lvr, hm.btr = hm.yacht.measureLSM()
    hm.vb = fn * np.sqrt(hm.g * hm.lsm)
    hm.phi = heel_deg
    hm.fn = hm.vb / np.sqrt(hm.g * hm.lsm)
    hm.leeway, hm.span_mult = 0.0, 1.0
    hm._get_Ri()
    rr, rv = float(hm._get_Rr()), float(hm._get_Rv())
    return {"fn": fn, "heel_deg": heel_deg, "speed_mps": float(hm.vb),
            "residuary_N": rr, "viscous_N": rv, "total_N": rr + rv}


def evaluate_tow(hm, speed_kn):
    hm.lsm, hm.lvr, hm.btr = hm.yacht.measureLSM()
    hm.vb = speed_kn * KNOT
    hm.phi = 0.0
    hm.fn = hm.vb / np.sqrt(hm.g * hm.lsm)
    hm.leeway, hm.span_mult = 0.0, 1.0
    induced = float(hm._get_Ri())
    rr, rv = float(hm._get_Rr()), float(hm._get_Rv())
    q = 0.5 * hm.rho * hm.vb ** 2
    parts = {}
    for appendage in hm.yacht.appendages:
        parts[appendage.type] = {
            "residuary_N": float(appendage._cr(hm.fn) * appendage.vol * hm.rho * hm.g * 1e-3),
            "viscous_N": float(q * hm._app_cd_area(appendage, 0.0)),
        }
        parts[appendage.type]["total_N"] = (parts[appendage.type]["residuary_N"]
                                            + parts[appendage.type]["viscous_N"])
    app_rr = sum(p["residuary_N"] for p in parts.values())
    app_rv = sum(p["viscous_N"] for p in parts.values())
    out = {"speed_kn": speed_kn, "speed_mps": float(hm.vb), "fn": float(hm.fn),
           "canoe": {"residuary_N": rr - app_rr, "viscous_N": rv - app_rv,
                     "total_N": rr + rv - app_rr - app_rv}}
    out.update(parts)
    out["induced_N"] = induced
    out["total_N"] = rr + rv + induced
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boat", default=str(ROOT / "data" / "boats" / "j80"))
    args = ap.parse_args()
    boat = Path(args.boat)
    check_vendor()
    hm, inputs = build_hydro(boat)
    points = [evaluate(hm, fn, h) for fn in FN for h in HEEL_DEG]
    app_inputs = appendage_inputs(boat)
    hm_app, _ = build_hydro(boat, app_inputs)
    tow = {
        "description": "Upright, zero-leeway tow resistance at fixed speeds of the canoe body "
                       "with Python-VPP's Keel and Rudder (the trapezoids run_pyvpp.py "
                       "builds), same hull inputs as `points`. total_N is Python-VPP's "
                       "_get_Rr + _get_Rv + _get_Ri at zero leeway, so induced_N is zero.",
        "method": [
            "Speed V = speed_kn * 0.514444 m/s; Fn = V / sqrt(g Lwl).",
            "Keel residuary: Python-VPP's rrk curve at Fn (clamped to 0..0.6) times the "
            "keel volume 0.666 c_mean 1.2 span, rho g 1e-3. The rudder has none.",
            "Appendage viscous: ORC 6.1.2 strips, 0.5 rho V^2 sum(wsa/5 (Cf_app(c) (1 + k) "
            "+ dcd)) with wsa = 2 c_mean span and dcd zero at zero side force.",
            "canoe is the remainder of _get_Rr and _get_Rv once the appendage terms are "
            "taken out; it equals the canoe-only evaluation.",
        ],
        "appendages": app_inputs,
        "points": [evaluate_tow(hm_app, kn) for kn in TOW_KN],
    }
    out = {
        "description": "Canoe-body hull resistance from the vendored Python-VPP "
                       "(HydroMod._get_Rr + HydroMod._get_Rv, Yacht built with App=[] so "
                       "no keel, bulb or rudder terms), for the hull at the sailing "
                       "displacement. Reference for the core ORC hull resistance model.",
        "written_by": "tools/vpp/hull_reference.py",
        "python_vpp": {"repository": "https://github.com/marinlauber/Python-VPP",
                       "commit": PYVPP_COMMIT, "licence": "MIT"},
        "method": [
            "Inputs from hull_hydro.json conditions.sailing.result: Lwl, canoe_volume, "
            "Bwl, Tc, wetted_area_canoe and displacement_kg (Python-VPP's Mass).",
            "Python-VPP constants: rho 1025 kg/m^3, mu 1.19e-3 Pa s (nu = mu / rho), "
            "g 9.81 m/s^2, roughness 0.",
            "Speed V = Fn sqrt(g Lwl), Python-VPP taking LSM = Lwl.",
            "Rr = ORCi drag surface (Fn, LVR = Lwl / canoe_volume^(1/3), BTR = Bwl / Tc), "
            "linear in each axis with a zero row at Fn 0, times Mass g 1e-3; times "
            "1 + 0.0004 (heel_deg - 30)^2 at heel of 30 deg or more.",
            "Rv = 0.5 rho V^2 wetted_area_canoe Cf 1.05, Cf = 0.066 / (log10 Re - 2.03)^2, "
            "Re = 0.85 V Lwl / nu.",
        ],
        "inputs": {
            "lwl_m": inputs["Lwl"], "canoe_volume_m3": inputs["Vol"],
            "bwl_m": inputs["Bwl"], "tc_m": inputs["Tc"],
            "wetted_area_canoe_m2": inputs["WSA"], "displacement_kg": inputs["Mass"],
            "lvr": float(hm.lvr), "btr": float(hm.btr),
            "rho": hm.rho, "mu": hm.mu, "g": hm.g,
        },
        "deviations_from_orc_2023": DEVIATIONS,
        "points": points,
        "tow": tow,
    }
    path = boat / "hull_resistance_reference.json"
    path.write_text(json.dumps(out, indent=2) + "\n")
    for p in points:
        print(f"Fn {p['fn']:.2f} heel {p['heel_deg']:4.0f}: Rr {p['residuary_N']:8.2f} "
              f"Rv {p['viscous_N']:7.2f} total {p['total_N']:8.2f} N")
    for p in tow["points"]:
        print(f"{p['speed_kn']:.0f} kn: canoe {p['canoe']['total_N']:7.2f} keel "
              f"{p['keel']['total_N']:6.2f} rudder {p['rudder']['total_N']:6.2f} "
              f"total {p['total_N']:7.2f} N")
    print(f"LVR {hm.lvr:.4f} BTR {hm.btr:.4f}; wrote {path}")


if __name__ == "__main__":
    main()
