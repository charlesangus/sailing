#!/usr/bin/env python3
"""Write the sailing-condition hydrostatics into hull_hydro.json `conditions.sailing`.

hull_hydro.json `result` holds the hull at the ORC measurement trim. The
resistance models need it at the sailing displacement: boat plus crew, sails
and running rigging. This floats hull_physics.obj + appendages.obj at that
displacement, level trim, and measures the canoe body with
dsyhs_table.hydrostatics. It is a separate step from morph.py because it needs
appendages.obj, which tools/hull/appendages.py writes after morph.py; run it
last. morph.py rewrites hull_hydro.json from scratch and so drops the block.

Body frame: x forward, y to port, z up, metres; origin on the centreline at
the midpoint of Lwl on the measurement-trim DWL.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

# -I leaves the script directory off sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dsyhs_table import clip_below, hydrostatics  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CARRIED = ("mainsail", "jib", "running_rigging")


def v(d, *keys):
    for k in keys:
        d = d[k]
    return d["value"] if isinstance(d, dict) and "value" in d else d


def sailing_mass(mass):
    """Mass components in kg: certificate displacement, crew, carried items, and their sum."""
    comp = {"boat": v(mass, "displacement"), "crew": v(mass, "crew_weight_max_class")}
    comp.update({k: mass["items"][k]["value"] for k in CARRIED})
    comp["sailing"] = sum(comp.values())
    return comp


def load_meshes(boat):
    """Triangles of the hull + appendage union and of the hull alone."""
    hull = trimesh.load(boat / "hull_physics.obj", force="mesh", process=False)
    appm = trimesh.load(boat / "appendages.obj", force="mesh", process=False)
    # The keel top sits 0.020 m inside the canoe body; the union removes the overlap.
    union = trimesh.boolean.union([hull, appm], engine="manifold")
    return union.vertices[union.faces], hull.vertices[hull.faces]


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


def lcf_x(tris, zw):
    """x of the waterplane centroid: wet surface area projected on z, weighted by x."""
    wet, _ = clip_below(tris, zw)
    p0, p1, p2 = wet[:, 0], wet[:, 1], wet[:, 2]
    nz = 0.5 * np.cross(p1 - p0, p2 - p0)[:, 2]
    return float(np.sum(nz * wet[:, :, 0].mean(axis=1)) / np.sum(nz))


def measure(boat):
    part = json.loads((boat / "particulars.json").read_text())
    mass = json.loads((boat / "mass.json").read_text())
    rho = 1000.0 * v(part, "flotation_sg")
    m = sailing_mass(mass)
    tris, hull_tris = load_meshes(boat)

    zw = float_plane(tris, m["sailing"] / rho)
    canoe = hydrostatics(hull_tris, zw - float(hull_tris[:, :, 2].min()))
    total = hydrostatics(tris, zw - float(tris[:, :, 2].min()))
    return {
        "displacement_kg": m["sailing"],
        "displacement_components_kg": m,
        "flotation_sg": v(part, "flotation_sg"),
        "sinkage_m": zw,
        "lwl": canoe["lwl_m"],
        "bwl": canoe["bwl_m"],
        "tc": canoe["tc_m"],
        "canoe_volume": canoe["vol_m3"],
        "total_volume": total["vol_m3"],
        "lcb_x_canoe": canoe["x_fp_m"] - canoe["lcb_aft_fp_m"],
        "lcb_x_total": total["x_fp_m"] - total["lcb_aft_fp_m"],
        "lcb_frac_lwl_aft_fp": canoe["lcb_frac_lwl"],
        "lcf_x": lcf_x(hull_tris, zw),
        "cp": canoe["cp"],
        "cm": canoe["cm"],
        "cw": canoe["cw"],
        "waterplane_area": canoe["aw_m2"],
        "wetted_area_canoe": canoe["sc_m2"],
        "max_section_area": canoe["ax_m2"],
    }


def sailing_block(boat):
    res = {k: float(x) if isinstance(x, (float, np.floating)) else x
           for k, x in measure(boat).items()}
    return {
        "frame": "Body frame of hull_hydro.json (x forward, y to port, z up, metres; origin at DWL midship). sinkage_m is the height of the sailing flotation plane above the measurement-trim DWL (z = 0), so the boat floats that much deeper. Level trim. lcb_x_* and lcf_x are body x.",
        "method": "Sailing displacement (mass.json: certificate displacement + max class crew + mainsail, jib, running rigging) floated on hull_physics.obj united with appendages.obj by bisection on the union volume, at flotation_sg. The canoe body (hull_physics.obj alone) is measured below that plane with dsyhs_table.hydrostatics: lwl, bwl, tc, canoe_volume, lcb_x_canoe, cp, cm, cw, waterplane_area, wetted_area_canoe, max_section_area. total_volume and lcb_x_total are the union (canoe plus keel and rudder). lcf_x is the x centroid of the canoe waterplane. Written by tools/hull/condition_hydro.py; values are not rounded.",
        "confidence": "derived",
        "result": res,
    }


def run(boat):
    path = boat / "hull_hydro.json"
    doc = json.loads(path.read_text())
    doc.setdefault("conditions", {})["sailing"] = sailing_block(boat)
    path.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    return doc["conditions"]["sailing"]


def read(boat):
    """The written sailing block, checked against the current mass.json."""
    block = json.loads((boat / "hull_hydro.json").read_text()).get("conditions", {}).get("sailing")
    if block is None:
        sys.exit("condition_hydro: hull_hydro.json has no conditions.sailing, "
                 "run tools/hull/condition_hydro.py")
    want = sailing_mass(json.loads((boat / "mass.json").read_text()))["sailing"]
    if abs(block["result"]["displacement_kg"] - want) > 1e-9:
        sys.exit(f"condition_hydro: conditions.sailing is for {block['result']['displacement_kg']} kg, "
                 f"mass.json gives {want} kg; run tools/hull/condition_hydro.py")
    return block["result"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boat-dir", type=Path, default=ROOT / "data/boats/j80")
    args = ap.parse_args(argv)
    res = run(args.boat_dir)["result"]
    print(f"sailing {res['displacement_kg']:.1f} kg, sinkage {res['sinkage_m']:.4f} m, "
          f"Lwl {res['lwl']:.4f} Bwl {res['bwl']:.4f} Tc {res['tc']:.4f} m, "
          f"canoe {res['canoe_volume']:.4f} m^3, total {res['total_volume']:.4f} m^3")


if __name__ == "__main__":
    main()
