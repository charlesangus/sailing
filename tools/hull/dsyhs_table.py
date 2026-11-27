"""Upright canoe-body hydrostatics of the DSYHS IGES hulls at their design waterline.

The IGES files are starboard half hulls (y <= 0, model 43 excepted) at model
scale: x forward from the aft perpendicular, z up from the canoe-body bottom, in mm.
Each hull is tessellated, and the wetted part is the mesh below the plane
z = zmin + Tc, where Tc is the series' published canoe-body draft. The region
bounded by the wetted mesh, the centreplane y = 0 and the waterplane is closed,
or by the wetted mesh and the waterplane for a full hull,
so its integrals reduce, by the divergence theorem, to sums over the wetted mesh
alone: fields of the form (0, 0, f(x)(z - zw)) vanish on both planes.
"""

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.IGESControl import IGESControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

MM = 1e-3
STATIONS = 400


def read_mesh(path, deflection_mm):
    reader = IGESControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, 0.1, True)
    tris = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            nodes = np.array(
                [
                    (lambda p: (p.X(), p.Y(), p.Z()))(tri.Node(i).Transformed(trsf))
                    for i in range(1, tri.NbNodes() + 1)
                ]
            )
            idx = np.array(
                [tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]
            ) - 1
            if face.Orientation() == TopAbs_REVERSED:
                idx = idx[:, ::-1]
            tris.append(nodes[idx])
        exp.Next()
    return np.concatenate(tris) * MM


def clip_below(tris, zw):
    """Return the parts of the triangles below z = zw and the waterline points."""
    below = tris[:, :, 2] <= zw
    n_below = below.sum(axis=1)
    kept = [tris[n_below == 3]]
    wl = []
    for t, b in zip(tris[(n_below > 0) & (n_below < 3)], below[(n_below > 0) & (n_below < 3)]):
        poly = []
        for i in range(3):
            p, q = t[i], t[(i + 1) % 3]
            if b[i]:
                poly.append(p)
            if b[i] != b[(i + 1) % 3]:
                s = (zw - p[2]) / (q[2] - p[2])
                c = p + s * (q - p)
                poly.append(c)
                wl.append(c)
        for k in range(1, len(poly) - 1):
            kept.append(np.array([[poly[0], poly[k], poly[k + 1]]]))
    return np.concatenate(kept), np.array(wl)


def section_area(wet, x, zw):
    """Half-section area at station x of a mesh lying wholly below zw."""
    d = wet[:, :, 0] - x
    neg = d < 0
    hit = neg.any(axis=1) & ~neg.all(axis=1)
    t, d, neg = wet[hit], d[hit], neg[hit]
    if len(t) == 0:
        return 0.0
    i, j = np.array([0, 1, 2]), np.array([1, 2, 0])
    cross = neg[:, i] != neg[:, j]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = d[:, i] / (d[:, i] - d[:, j])
        pts = t[:, i] + s[:, :, None] * (t[:, j] - t[:, i])
    order = np.argsort(~cross, axis=1, kind="stable")[:, :2]
    rows = np.arange(len(t))[:, None]
    a, b = pts[rows, order].transpose(1, 0, 2)
    seg = b - a
    normal = np.stack([-seg[:, 2], seg[:, 1]], axis=1)
    tri_n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])[:, 1:]
    normal *= np.where(np.sum(normal * tri_n, axis=1) < 0, -1.0, 1.0)[:, None]
    return abs(np.sum((0.5 * (a[:, 2] + b[:, 2]) - zw) * normal[:, 1]))


def hydrostatics(tris, tc):
    zmin = tris[:, :, 2].min()
    zw = zmin + tc
    wet, wl = clip_below(tris, zw)
    p0, p1, p2 = wet[:, 0], wet[:, 1], wet[:, 2]
    n = 0.5 * np.cross(p1 - p0, p2 - p0)
    area = np.linalg.norm(n, axis=1)
    # Model 43 is modelled as a full hull; every other file is a port half.
    full = tris[:, :, 1].max() > 0.01 * -tris[:, :, 1].min()
    k = 1.0 if full else 2.0
    # Exact for linear integrands; (z - zw) * x is quadratic, so use the
    # three edge-midpoint rule, which is exact for quadratics on a triangle.
    mids = np.stack([(p0 + p1) / 2, (p1 + p2) / 2, (p2 + p0) / 2], axis=1)
    hz = mids[:, :, 2] - zw
    vol_h = np.sum(hz.mean(axis=1) * n[:, 2])
    mx_h = np.sum((mids[:, :, 0] * hz).mean(axis=1) * n[:, 2])
    sign = 1.0 if vol_h > 0 else -1.0
    vol_h, mx_h = sign * vol_h, sign * mx_h
    aw_h = abs(np.sum(n[:, 2]))
    closure = abs(np.sum(n[:, 0])) / max(np.sum(np.abs(n[:, 0])), 1e-12)

    x_fp, x_ap = wl[:, 0].max(), wl[:, 0].min()
    lwl = x_fp - x_ap
    bwl = wl[:, 1].max() - wl[:, 1].min() if full else 2.0 * np.abs(wl[:, 1]).max()
    xb = mx_h / vol_h
    xs = np.linspace(x_ap, x_fp, STATIONS + 2)[1:-1]
    ax = k * max(section_area(wet, x, zw) for x in xs)
    vol = k * vol_h
    return {
        "zmin_m": zmin,
        "half": not full,
        "z_dwl_m": zw,
        "x_fp_m": x_fp,
        "lwl_m": lwl,
        "bwl_m": bwl,
        "tc_m": tc,
        "vol_m3": vol,
        "lcb_aft_fp_m": x_fp - xb,
        "lcb_frac_lwl": (x_fp - xb) / lwl,
        "aw_m2": k * aw_h,
        "ax_m2": ax,
        "sc_m2": k * area.sum(),
        "cp": vol / (ax * lwl),
        "cm": ax / (bwl * tc),
        "cb": vol / (lwl * bwl * tc),
        "cw": k * aw_h / (lwl * bwl),
        "lwl_bwl": lwl / bwl,
        "bwl_tc": bwl / tc,
        "lwl_vol13": lwl / vol ** (1.0 / 3.0),
        "closure": closure,
    }


def load_published(xlsx):
    df = pd.read_excel(xlsx, sheet_name="Canoe body hydrostatics")
    df = df.rename(columns={"Sysser": "hull_id", "Overhang": "overhang"})
    df["hull_id"] = df["hull_id"].astype(int)
    return df


def parse_name(path):
    m = re.fullmatch(r"SYSSER(\d+)(?:_(0|05))?_surface\.igs", path.name)
    if not m:
        return None
    return int(m.group(1)), {None: 1.0, "0": 0.0, "05": 0.5}[m.group(2)]


def main(argv=None):
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--igs-dir", type=Path, default=root / "data/dsyhs/raw/IGS")
    ap.add_argument(
        "--hydrostatics",
        type=Path,
        default=root / "data/dsyhs/raw/DSYHS_hydrostatics_modelscale.xlsx",
    )
    ap.add_argument("--out", type=Path, default=root / "data/dsyhs/table.csv")
    ap.add_argument("--deflection-mm", type=float, default=0.05)
    ap.add_argument("--only", nargs="*", help="file stems to process, e.g. SYSSER01")
    ap.add_argument(
        "--compare",
        type=Path,
        help="write a per-hull comparison with the published table to this CSV",
    )
    args = ap.parse_args(argv)

    pub = load_published(args.hydrostatics)
    rows, cmp_rows = [], []
    for path in sorted(args.igs_dir.glob("SYSSER*_surface.igs")):
        key = parse_name(path)
        if key is None or (args.only and path.stem.split("_surface")[0] not in args.only):
            continue
        hull_id, overhang = key
        ref = pub[(pub.hull_id == hull_id) & np.isclose(pub.overhang, overhang)]
        if len(ref) != 1:
            raise RuntimeError(f"{path.name}: no unique published row")
        ref = ref.iloc[0]
        tris = read_mesh(path, args.deflection_mm)
        h = hydrostatics(tris, float(ref.tc0))
        if h["closure"] > 1e-3:
            print(f"{path.name}: wetted surface not closed ({h['closure']:.2e})", file=sys.stderr)
        rows.append({"hull_id": hull_id, "overhang": overhang, "file": path.name, **h})

        pub_v = {
            "lwl_bwl": ref.lwl0 / ref.bwl0,
            "bwl_tc": ref.bwl0 / ref.tc0,
            "lcb_frac_lwl": 0.5 - ref.lcb0 / ref.lwl0,
            "cp": ref.cp0,
            "cm": ref.cm0,
            "lwl_vol13": ref.lwl0 / ref.volc0 ** (1.0 / 3.0),
            "vol_m3": ref.volc0,
            "aw_m2": ref.aw0,
            "sc_m2": ref.sc0,
        }
        c = {"hull_id": hull_id, "overhang": overhang}
        for k, v in pub_v.items():
            c[k] = h[k]
            c[k + "_pub"] = v
            c[k + "_err_pct"] = 100.0 * (h[k] - v) / v
        cmp_rows.append(c)
        print(
            f"{path.name}: Lwl {h['lwl_m']:.4f} ({ref.lwl0:.4f}) "
            f"vol {h['vol_m3']:.6f} ({ref.volc0:.6f}) "
            f"Cp {h['cp']:.4f} ({ref.cp0:.4f})",
            file=sys.stderr,
        )

    table = pd.DataFrame(rows).drop(columns=["closure", "zmin_m", "half"])
    table = table.sort_values(["hull_id", "overhang"], ascending=[True, False])
    table.to_csv(args.out, index=False, float_format="%.6g")
    if args.compare:
        pd.DataFrame(cmp_rows).to_csv(args.compare, index=False, float_format="%.5g")
    errs = pd.DataFrame(cmp_rows).filter(like="_err_pct").abs()
    print(errs.max().round(3).to_string(), file=sys.stderr)


if __name__ == "__main__":
    main()
