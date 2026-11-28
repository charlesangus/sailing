#!/usr/bin/env python3
"""J/80 main + jib polar from Python-VPP, with inputs taken from the dossier.

Writes data/boats/j80/pyvpp_polar_nospi.csv (columns as orc_polar.csv) and
data/boats/j80/pyvpp_polar_upwind_check.md (upwind speeds against the ORC
certificate). Run tools/vpp/fetch_pyvpp.sh first, then
`.venv/bin/python -I tools/vpp/run_pyvpp.py`.

Python-VPP sails the boat at its sailing displacement, so the hull inputs are
measured on hull_physics.obj + appendages.obj floated at the certificate
displacement plus crew, sails and running rigging, not at the measurement trim
of hull_hydro.json. The righting-arm curve comes from the same meshes, with the
boat's VCG at the certificate VCGM.
"""

import argparse
import json
import logging
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "tools" / "vpp" / "vendor" / "Python-VPP"
PYVPP_COMMIT = "9b9fe8f3f63d902a1df207691109e42932d0a9bb"

# -I leaves the script directory off sys.path.
sys.path.insert(0, str(ROOT / "tools" / "hull"))
sys.path.insert(0, str(VENDOR / "src"))
from dsyhs_table import clip_below, hydrostatics  # noqa: E402
from python_vpp import YachtMod  # noqa: E402

TWS_KN = [6, 8, 10, 12, 14, 16, 20]
TWA_DEG = np.arange(20.0, 181.0, 1.0)
ORC_FIXED_TWA = [52, 60, 75, 90, 110, 120, 135, 150]
GZ_HEEL_DEG = np.arange(0.0, 60.1, 5.0)
G = 9.80665


def load(boat, name):
    with open(boat / name) as f:
        return json.load(f)


def v(d, *keys):
    for k in keys:
        d = d[k]
    return d["value"] if isinstance(d, dict) and "value" in d else d


def check_vendor():
    if not (VENDOR / "src" / "python_vpp").is_dir():
        sys.exit(f"run_pyvpp: {VENDOR} missing, run tools/vpp/fetch_pyvpp.sh")
    head = subprocess.run(["git", "-C", str(VENDOR), "rev-parse", "HEAD"],
                          capture_output=True, text=True, check=True).stdout.strip()
    if head != PYVPP_COMMIT:
        sys.exit(f"run_pyvpp: {VENDOR} is at {head}, expected {PYVPP_COMMIT}; "
                 "run tools/vpp/fetch_pyvpp.sh")


def heel(tris, phi):
    c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    return tris @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]).T


def immersed(tris, zw):
    """Volume and (y, z) centroid of a closed mesh below z = zw.

    Same divergence-theorem reduction as dsyhs_table.hydrostatics, with the
    edge-midpoint rule because y (z - zw) and z (z - zw) are quadratic.
    """
    wet, _ = clip_below(tris, zw)
    p0, p1, p2 = wet[:, 0], wet[:, 1], wet[:, 2]
    nz = 0.5 * np.cross(p1 - p0, p2 - p0)[:, 2]
    mids = np.stack([(p0 + p1) / 2, (p1 + p2) / 2, (p2 + p0) / 2], axis=1)
    hz = mids[:, :, 2] - zw
    vol = np.sum(hz.mean(axis=1) * nz)
    my = np.sum((mids[:, :, 1] * hz).mean(axis=1) * nz)
    mz = np.sum((mids[:, :, 2] * hz).mean(axis=1) * nz)
    return abs(vol), my / vol, mz / vol


def float_plane(tris, vol):
    lo, hi = float(tris[:, :, 2].min()), float(tris[:, :, 2].max())
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if immersed(tris, mid)[0] < vol:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def righting_arm(tris, vol, zg, phi):
    """GZ at heel phi, level trim, with G on the centreline at height zg."""
    t = heel(tris, phi)
    _, yb, _ = immersed(t, float_plane(t, vol))
    return -(yb + zg * math.sin(math.radians(phi)))


def sheer_z(points, x):
    xs, zs = zip(*points)
    return float(np.polyval(np.polyfit(xs, zs, 2), x))


def build_inputs(boat):
    part, hh = load(boat, "particulars.json"), load(boat, "hull_hydro.json")
    app, rig = load(boat, "appendages.json"), load(boat, "rig.json")
    sails, mass = load(boat, "sails.json"), load(boat, "mass.json")
    rho = 1000.0 * v(part, "flotation_sg")
    stem_x = hh["transform"]["stem_x_in_body"]

    hull = trimesh.load(boat / "hull_physics.obj", force="mesh", process=False)
    appm = trimesh.load(boat / "appendages.obj", force="mesh", process=False)
    # The keel top sits 0.020 m inside the canoe body; the union removes the overlap.
    boat = trimesh.boolean.union([hull, appm], engine="manifold")
    tris, hull_tris = boat.vertices[boat.faces], hull.vertices[hull.faces]

    m_boat = v(part, "displacement")
    m_crew = v(mass, "crew_weight_max_class")
    items = mass["items"]
    carried = {k: items[k]["value"] for k in ("mainsail", "jib", "running_rigging")}
    m_sail = m_boat + m_crew + sum(carried.values())

    # ORC's VCGM is z up from the flotation plane, so a negative value is below it.
    zg_boat = v(part, "vcgm")
    vol_meas = m_boat / rho
    gz1_at_zero = righting_arm(tris, vol_meas, 0.0, 1.0)
    rm1_mesh = m_boat * G * righting_arm(tris, vol_meas, zg_boat, 1.0)

    sheer = hh["sheer"]["points"]
    z_crew = sheer_z(sheer, v(mass, "crew_cg_x") + stem_x) + v(mass, "crew_cg_above_sheer")
    z_main = np.mean([v(sails, "main", k)[2] for k in ("tack", "head", "clew")])
    z_jib = np.mean([v(sails, "jib", k)[2] for k in ("tack", "head", "clew")])
    # mass.json puts running rigging at the mast CG, which is MCG above the mast heel
    # (IMS F8.2), not above the sheer.
    z_rr = items["running_rigging"]["cg"]["value"][2]
    zg_sail = (m_boat * zg_boat + m_crew * z_crew + carried["mainsail"] * z_main
               + carried["jib"] * z_jib + carried["running_rigging"] * z_rr) / m_sail

    vol_sail = m_sail / rho
    zw = float_plane(tris, vol_sail)
    canoe = hydrostatics(hull_tris, zw - float(hull_tris[:, :, 2].min()))
    gz = [righting_arm(tris, vol_sail, zg_sail, p) if p > 0 else 0.0 for p in GZ_HEEL_DEG]

    keel, rudder = app["foils"]["keel"], app["foils"]["rudder"]
    ks = v(app, "keel", "sections")
    depth = [s["depth_below_root_m"] for s in ks]
    keel_tc = np.trapezoid([s["max_thickness_m"] for s in ks], depth) / np.trapezoid(
        [s["chord_m"] for s in ks], depth)

    p, e = v(rig, "p"), v(rig, "e")
    return {
        "name": v(part, "name"),
        "cert": {"rm1_kgm": v(part, "righting_moment_1deg") / G,
                 "rm1_nm": v(part, "righting_moment_1deg"), "vcgm": v(part, "vcgm"),
                 "crew_default": v(part, "crew_weight_default")},
        "rho": rho,
        "sinkage": zw,
        "mass": {"boat": m_boat, "crew": m_crew, **carried, "sailing": m_sail},
        "zg": {"boat": zg_boat, "crew": z_crew, "mainsail": float(z_main),
               "jib": float(z_jib), "running_rigging": z_rr, "sailing": zg_sail},
        "gz1_mesh_at_zg0": gz1_at_zero,
        "rm1_mesh_nm": rm1_mesh,
        "yacht": {
            "Lwl": canoe["lwl_m"],
            "Vol": canoe["vol_m3"],
            "Bwl": canoe["bwl_m"],
            "Tc": canoe["tc_m"],
            "WSA": canoe["sc_m2"],
            "Tmax": v(part, "draft") + zw,
            "Amax": canoe["ax_m2"],
            "Mass": m_sail,
            "Loa": v(part, "loa"),
            "Boa": v(part, "beam"),
            "Ff": v(part, "freeboard_fwd_ffm") - zw,
            "Fa": v(part, "freeboard_aft_fam") - zw,
            "crew_weight": m_crew,
        },
        "gz": {"Heel": [float(x) for x in GZ_HEEL_DEG], "GZ": [float(x) for x in gz]},
        # Python-VPP's trapezoid takes the root chord as given and the tip chord
        # that makes chord x span equal the real planform area, because the keel
        # narrows to 0.74 m at the neck and flares to 1.23 m at the bulb.
        "keel": {"Cu": keel["root_chord"]["value"],
                 "Cl": 2 * keel["planform_area"]["value"] / keel["span"]["value"]
                 - keel["root_chord"]["value"],
                 "Span": keel["span"]["value"], "Tc_ratio": float(keel_tc)},
        "rudder": {"Cu": rudder["root_chord"]["value"],
                   "Cl": 2 * rudder["planform_area"]["value"] / rudder["span"]["value"]
                   - rudder["root_chord"]["value"],
                   "Span": rudder["span"]["value"],
                   "Tc_ratio": rudder["thickness_chord_ratio_root"]["value"]},
        "main": {"P": p, "E": e, "Roach": v(sails, "main", "area") / (0.5 * p * e) - 1.0,
                 "BAD": v(rig, "boom_height_above_sheer")},
        # ORC's headsail inputs: IG for the hoist, HLP for the luff perpendicular,
        # and the sheer at the mast as the base of I.
        "jib": {"I": v(rig, "i_jib"), "J": v(rig, "j"), "LPG": v(sails, "jib", "hlp"),
                "HBI": v(rig, "freeboard_at_mast") - zw},
        "areas_dossier": {"main": v(sails, "main", "area"), "jib": v(sails, "jib", "area")},
    }


def _get_rmc(self, phi):
    # ORC VPP Documentation 2023 eq. 4.30 (section 4.4.3), in SI. The ramp
    # between 2.5 and 7.5 deg of heel is the vendored one.
    n_aft = min(2.0, self.cw / self.bdwt)
    rail = self.cw - n_aft * self.bdwt
    rm = self.g * (rail * 0.5 * self.boa + n_aft * self.bdwt * 0.7 * 0.5 * self.boa)
    rm = rm * np.cos(np.radians(phi))
    return rm * np.where(
        phi <= 7.5, 0.5 * (1 - np.cos(np.maximum(0, phi - 2.5) / 5.0 * np.pi)), 1.0
    )


# Module level so that spawned worker processes apply it on import as well.
YachtMod.Yacht._get_RmC = _get_rmc


def run_vpp(inp, tws_kn, twa_deg):
    from python_vpp import VPP, Jib, Keel, Main, Rudder, Yacht

    yacht = Yacht(
        Name="J80",
        App=[Keel(**inp["keel"]), Rudder(**inp["rudder"])],
        Sails=[Main("Main", **inp["main"]), Jib("Jib", **inp["jib"])],
        GZ=inp["gz"],
        **inp["yacht"],
    )
    vpp = VPP(Yacht=yacht)
    vpp.set_analysis(tws_range=np.array(tws_kn, dtype=float), twa_range=np.array(twa_deg))
    vpp.run(method="parallel")
    return yacht, np.asarray(vpp.store)[:, :, 0, 0]


def optimum(twa, bsp, upwind):
    vmg = bsp * np.cos(np.radians(twa))
    if upwind:
        ok = (twa < 90) & (bsp > 0)
        j = np.flatnonzero(ok)[np.argmax(vmg[ok])]
    else:
        ok = (twa > 90) & (bsp > 0)
        j = np.flatnonzero(ok)[np.argmax(-vmg[ok])]
    return j


def fmt_twa(a):
    return f"{a:g}"


def write_csv(path, tws_kn, twa, bsp):
    lines = ["tws_kn,twa_deg,bsp_kn,kind"]
    for i, tws in enumerate(tws_kn):
        jb, jr = optimum(twa, bsp[i], True), optimum(twa, bsp[i], False)
        lines.append(f"{tws},{fmt_twa(twa[jb])},{bsp[i, jb]:.2f},beat")
        for j, a in enumerate(twa):
            if bsp[i, j] > 0:
                lines.append(f"{tws},{fmt_twa(a)},{bsp[i, j]:.2f},fixed")
        lines.append(f"{tws},{fmt_twa(twa[jr])},{bsp[i, jr]:.2f},run")
    path.write_text("\n".join(lines) + "\n")


def read_orc(boat):
    rows = {}
    for line in (boat / "orc_polar.csv").read_text().splitlines()[1:]:
        tws, twa, bsp, kind = line.split(",")
        rows.setdefault(int(float(tws)), {})[(kind, float(twa))] = float(bsp)
    return rows


def pct(a, b):
    return f"{100.0 * (a - b) / b:+.1f}%"


def write_check(path, boat, inp, yacht, tws_kn, twa, bsp):
    orc = read_orc(boat)
    y, cert = inp["yacht"], inp["cert"]
    out = [
        f"# {inp['name']} Python-VPP main + jib polar: upwind check against the ORC certificate",
        "",
        "Written by `tools/vpp/run_pyvpp.py`. ORC values are the 2019 OD certificate "
        "(`orc_polar.csv`), which is with spinnaker; upwind the spinnaker plays no part, "
        "so the beat and 52/60 deg speeds are comparable. The ORC beat boat speed is "
        "derived as Beat VMG / cos(beat angle) (see `sources.md`), so the VMG columns "
        "are the like-for-like comparison of the certificate's printed number.",
        "",
        "Differences are Python-VPP minus ORC, relative to ORC. The Python-VPP beat "
        f"angle is the VMG optimum on a {TWA_DEG[1] - TWA_DEG[0]:g} deg TWA grid.",
        "",
        "| TWS kn | ORC beat deg | VPP beat deg | ORC VMG kn | VPP VMG kn | VMG diff "
        "| ORC beat bsp | VPP beat bsp | bsp diff | ORC 52 | VPP 52 | 52 diff "
        "| ORC 60 | VPP 60 | 60 diff |",
        "|" + "---|" * 15,
    ]
    floor_hits, beat_diff, vmg_diff, run_twa = [], [], [], []
    for i, tws in enumerate(tws_kn):
        o = orc[tws]
        (obeat, obsp), = [(a, s) for (k, a), s in o.items() if k == "beat"]
        ovmg = obsp * math.cos(math.radians(obeat))
        jb = optimum(twa, bsp[i], True)
        vbeat, vbsp = twa[jb], bsp[i, jb]
        vvmg = vbsp * math.cos(math.radians(vbeat))
        if vbeat == twa[0]:
            vbeat_txt = f"{vbeat:g} (grid-limited)"
            floor_hits.append(tws)
        else:
            vbeat_txt = f"{vbeat:g}"
        beat_diff.append((tws, vbeat - obeat))
        vmg_diff.append(100.0 * (vvmg - ovmg) / ovmg)
        run_twa.append(twa[optimum(twa, bsp[i], False)])
        j52, j60 = int(np.flatnonzero(twa == 52)[0]), int(np.flatnonzero(twa == 60)[0])
        o52, o60 = o[("fixed", 52.0)], o[("fixed", 60.0)]
        out.append(
            f"| {tws} | {obeat:g} | {vbeat_txt} | {ovmg:.2f} | {vvmg:.2f} | {pct(vvmg, ovmg)} "
            f"| {obsp:.2f} | {vbsp:.2f} | {pct(vbsp, obsp)} "
            f"| {o52:.2f} | {bsp[i, j52]:.2f} | {pct(bsp[i, j52], o52)} "
            f"| {o60:.2f} | {bsp[i, j60]:.2f} | {pct(bsp[i, j60], o60)} |")
    m, z = inp["mass"], inp["zg"]
    sails_area = {s.type: s.area for s in yacht.sails}
    beat_txt = ", ".join(f"{d:+.1f} at {t} kn" for t, d in beat_diff)
    if floor_hits:
        grid_txt = (f"The beat optimum is at the {twa[0]:g} deg grid floor at "
                    f"{', '.join(str(t) for t in floor_hits)} kn, so those angles are grid-limited.")
    else:
        grid_txt = (f"The beat optimum is inside the {twa[0]:g}-{twa[-1]:g} deg grid at every "
                    "TWS, so none is grid-limited.")
    out += [
        "",
        "## Inputs",
        "",
        f"Python-VPP {PYVPP_COMMIT[:7]} (github.com/marinlauber/Python-VPP, MIT). "
        f"Sailing displacement {m['sailing']:.0f} kg = boat {m['boat']:.0f} + crew "
        f"{m['crew']:.0f} (class maximum) + mainsail {m['mainsail']:g} + jib {m['jib']:g} "
        f"+ running rigging {m['running_rigging']:g}; it floats {1000 * inp['sinkage']:.1f} mm "
        "below the certificate flotation plane.",
        "",
        "| Python-VPP input | value | from |",
        "|---|---|---|",
        f"| Lwl | {y['Lwl']:.3f} m | hull mesh at sailing displacement |",
        f"| Vol | {y['Vol']:.4f} m^3 | canoe body below that waterline |",
        f"| Bwl | {y['Bwl']:.3f} m | hull mesh |",
        f"| Tc | {y['Tc']:.3f} m | hull mesh |",
        f"| WSA | {y['WSA']:.3f} m^2 | canoe body wetted area, hull mesh |",
        f"| Amax | {y['Amax']:.4f} m^2 | max section area, hull mesh |",
        f"| Tmax | {y['Tmax']:.3f} m | certificate draft + sinkage |",
        f"| Mass | {y['Mass']:.0f} kg | sailing displacement |",
        f"| Loa, Boa | {y['Loa']:.3f}, {y['Boa']:.3f} m | certificate |",
        f"| Ff, Fa | {y['Ff']:.3f}, {y['Fa']:.3f} m | certificate FFM, FAM - sinkage |",
        f"| crew_weight | {y['crew_weight']:.0f} kg | class maximum (certificate default "
        f"{cert['crew_default']:.0f} kg exceeds it) |",
        f"| Keel Cu, Cl, Span, t/c | {inp['keel']['Cu']:.3f}, {inp['keel']['Cl']:.3f}, "
        f"{inp['keel']['Span']:.3f} m, {inp['keel']['Tc_ratio']:.3f} | appendages.json foils; "
        "Cl keeps the planform area; t/c is thickness over chord integrated over depth |",
        f"| Rudder Cu, Cl, Span, t/c | {inp['rudder']['Cu']:.3f}, {inp['rudder']['Cl']:.3f}, "
        f"{inp['rudder']['Span']:.3f} m, {inp['rudder']['Tc_ratio']:.3f} | appendages.json foils; "
        "Cl keeps the planform area |",
        f"| Main P, E, Roach, BAD | {inp['main']['P']:.3f}, {inp['main']['E']:.3f} m, "
        f"{inp['main']['Roach']:.4f}, {inp['main']['BAD']:.3f} m | rig.json; roach from the "
        f"measured area; Python-VPP area {sails_area['main']:.2f} m^2 "
        f"(dossier {inp['areas_dossier']['main']:.2f}) |",
        f"| Jib I, J, LPG, HBI | {inp['jib']['I']:.3f}, {inp['jib']['J']:.3f}, "
        f"{inp['jib']['LPG']:.3f}, {inp['jib']['HBI']:.3f} m | IG, J, HLP, freeboard at mast "
        f"- sinkage; Python-VPP area {sails_area['jib']:.2f} m^2 "
        f"(dossier {inp['areas_dossier']['jib']:.2f}) |",
        "",
        "Righting arm: the hull and appendage meshes heeled at level trim, at the sailing "
        f"displacement, with G at {z['sailing']:.3f} m. The boat's own VCG is the "
        f"certificate's VCGM, {z['boat']:+.3f} m, which ORC measures up from the "
        f"flotation plane, so it is {abs(z['boat']):.3f} m "
        f"{'below' if z['boat'] < 0 else 'above'} it (the meshes give "
        f"GZ(1 deg) = {inp['gz1_mesh_at_zg0']:.4f} m with G on the flotation plane). "
        f"Crew CG {z['crew']:.3f} m, mainsail {z['mainsail']:.3f} m, jib {z['jib']:.3f} m "
        f"and running rigging {z['running_rigging']:.3f} m above the flotation plane.",
        "",
        "| heel deg | " + " | ".join(f"{h:g}" for h in inp["gz"]["Heel"]) + " |",
        "|---|" + "---|" * len(inp["gz"]["Heel"]),
        "| GZ m | " + " | ".join(f"{g:.3f}" for g in inp["gz"]["GZ"]) + " |",
        "",
        "## Crew righting moment in Python-VPP",
        "",
        "Python-VPP's own crew moment is `0.8 bmax (cw + 0.7 bmax 89) cos(heel)` in kg m, "
        "summed with N m righting moments. `run_pyvpp.py` replaces it with ORC VPP "
        "Documentation 2023 eq. 4.30 (section 4.4.3): the crew count is CW / 89 kg, all but "
        "two crew sit on the rail at half the beam, and the helm and trimmer sit at 70% of "
        "that arm, times g and cos(heel). Beam is the certificate's "
        f"{yacht.boa:.3f} m. At 20 deg heel that is {float(yacht._get_RmC(20.0)):.0f} N m. "
        "The ramp over 2.5-7.5 deg of heel is Python-VPP's.",
        "",
        "## Known limitations",
        "",
        "- Crew moment: replaced with ORC eq. 4.30 in SI (see above). Half the maximum "
        "beam is used as the rail arm, which slightly overstates it, since the deck-edge "
        "half-beam at the crew station is a little less.",
        f"- Beat angle, Python-VPP minus the certificate, in deg: {beat_txt}. {grid_txt} "
        f"The beat VMG differs from the certificate by {min(vmg_diff):+.1f}% to "
        f"{max(vmg_diff):+.1f}% across the TWS range.",
        "- Python-VPP keeps the jib drawing at every TWA with no blanketing behind the "
        f"main, so the best run angles come out at {min(run_twa):g}-{max(run_twa):g} deg. "
        "This polar is the downwind calibration target.",
        "- Its keel volume formula ignores thickness.",
        "- It doubles the rudder aspect ratio.",
        f"- At the certificate's VCGM of {cert['vcgm']:+.3f} m the morphed meshes give "
        f"RM at 1 deg of {inp['rm1_mesh_nm']:.1f} N m at {m['boat']:.0f} kg, "
        f"{pct(inp['rm1_mesh_nm'], cert['rm1_nm'])} against the certificate's measured "
        f"{cert['rm1_nm']:.1f} N m ({cert['rm1_kgm']:.1f} kg m), so the morphed hull is "
        f"{'stiffer' if inp['rm1_mesh_nm'] > cert['rm1_nm'] else 'more tender'} than the "
        "real one. The VCGM is kept and the hull is to be corrected later.",
        "",
    ]
    path.write_text("\n".join(out))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--inputs-only", action="store_true",
                    help="print the mapped Python-VPP inputs and stop")
    ap.add_argument("--boat-dir", type=Path, default=ROOT / "data" / "boats" / "j80",
                    help="dossier directory to read and write (default data/boats/j80)")
    args = ap.parse_args(argv)
    boat = args.boat_dir
    logging.basicConfig(level=logging.WARNING)
    check_vendor()
    t0 = time.perf_counter()
    inp = build_inputs(boat)
    if args.inputs_only:
        print(json.dumps(inp, indent=1))
        return
    yacht, bsp = run_vpp(inp, TWS_KN, TWA_DEG)
    write_csv(boat / "pyvpp_polar_nospi.csv", TWS_KN, TWA_DEG, bsp)
    write_check(boat / "pyvpp_polar_upwind_check.md", boat, inp, yacht, TWS_KN, TWA_DEG, bsp)
    print(f"run_pyvpp: wrote pyvpp_polar_nospi.csv and pyvpp_polar_upwind_check.md "
          f"in {time.perf_counter() - t0:.1f} s")


if __name__ == "__main__":
    main()
