"""Morph the closest DSYHS hull to a boat's particulars and write its meshes and hydrostatics.

The parent hull is the DSYHS model whose Lwl/∇c^(1/3), Bwl/Tc and Lwl/Bwl are
closest to the boat's. Its IGES surface is cut into stations, each half
section is resampled to a fixed number of girth points and mirrored, and the
stations are joined into a closed tube with a flat deck at the sheer, a
transom cap and a stem apex. The tube is scaled to the target Lwl and Bwl,
and its vertical scale is iterated until canoe body plus keel, keel root
fairing and rudder displace the target volume. The overhangs are then
stretched, above the waterline only, so the stem lands where the boat's LOA
and LCF-from-stem put it, and the topsides are trimmed or extended vertically
to the boat's sheer and flared, above the DWL only, to the boat's max beam.

Body frame: x forward, y to port, z up, metres; origin on the centreline at
the midpoint of Lwl on the DWL.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import fast_simplification
import numpy as np
import pandas as pd
import trimesh

# -I leaves the script directory off sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dsyhs_table import clip_below, hydrostatics, read_mesh  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]

# NACA 4-digit thickness polynomial, open trailing edge.
NACA_A = (0.2969, -0.1260, -0.3516, 0.2843, -0.1015)
NACA_P = (0.5, 1.0, 2.0, 3.0, 4.0)

# Straightening a section all the way would make its topsides a flat panel
# along the DWL tangent; the cap keeps at least a tenth of every vertex's turn.
FLARE_BETA_MAX = 0.9
FLARE_INFLECTION_MAX = 0.05


def naca_area_and_centroid():
    """Section area / (t c) and centroid x / c of a NACA 4-digit thickness form."""
    area = 10.0 * sum(a / (p + 1.0) for a, p in zip(NACA_A, NACA_P))
    moment = 10.0 * sum(a / (p + 2.0) for a, p in zip(NACA_A, NACA_P))
    return area, moment / area


def val(d, key):
    return d[key]["value"]


def item(value, unit, source, confidence, method=None, note=None):
    out = {"value": value, "unit": unit, "source": source, "confidence": confidence}
    if method:
        out["method"] = method
    if note:
        out["note"] = note
    return out


def rudder_volume(rudder):
    """Displaced volume and centroid of a rudder from its section table, provisional frame."""
    k_area, xc = naca_area_and_centroid()
    secs = val(rudder, "sections")
    depth = np.array([s["depth_below_root_m"] for s in secs])
    chord = np.array([s["chord_m"] for s in secs])
    thick = np.array([s["max_thickness_m"] for s in secs])
    area = k_area * chord * thick
    vol = np.trapezoid(area, depth)
    x_aft = np.trapezoid(area * xc * chord, depth) / vol
    z_down = np.trapezoid(area * depth, depth) / vol
    axis = val(rudder, "stock_axis")
    return vol, np.array([axis[0] - x_aft, 0.0, axis[2] - z_down]), k_area


def rank_parents(table, lwl, vol_c, aw):
    """Distance of each DSYHS hull from the boat in log ratio space.

    Bwl comes from the measured waterplane area with the candidate's own Cw,
    and Tc from the canoe volume with the candidate's own Cb, so each
    candidate is compared with the boat it would become.
    """
    t = table.copy()
    bwl = aw / (t.cw * lwl)
    tc = vol_c / (t.cb * lwl * bwl)
    j = {
        "lwl_vol13": np.full(len(t), lwl / vol_c ** (1.0 / 3.0)),
        "bwl_tc": bwl / tc,
        "lwl_bwl": lwl / bwl,
    }
    t["distance"] = np.sqrt(sum(np.log(t[k] / v) ** 2 for k, v in j.items()))
    for k, v in j.items():
        t["j80_" + k] = v
    t["bwl_target"] = bwl
    t["tc_target"] = tc
    return t.sort_values(["distance", "overhang"], ascending=[True, False])


def cut_section(tris, x):
    """Starboard half section of a triangle soup at plane x, ordered keel to sheer."""
    d = tris[:, :, 0] - x
    neg = d < 0
    hit = neg.any(axis=1) & ~neg.all(axis=1)
    t, d, neg = tris[hit], d[hit], neg[hit]
    i, j = np.array([0, 1, 2]), np.array([1, 2, 0])
    cross = neg[:, i] != neg[:, j]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = d[:, i] / (d[:, i] - d[:, j])
        pts = t[:, i] + s[:, :, None] * (t[:, j] - t[:, i])
    pts = pts[cross][:, 1:]
    if len(pts) < 3:
        raise RuntimeError(f"section at x={x:.4f} is empty")
    pts[:, 0] = np.minimum(pts[:, 0], 0.0)
    zc = 0.5 * (pts[:, 1].min() + pts[:, 1].max())
    # Sections are star-shaped about a centreline point at mid depth, so the
    # polar angle orders the unconnected segment ends along the girth.
    ang = np.arctan2(-pts[:, 0], zc - pts[:, 1])
    pts = pts[np.argsort(ang)]
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    pts = np.vstack([pts[:1], pts[1:][step > 1e-9]])
    return np.vstack([[0.0, pts[0, 1]], pts])


def resample(poly, n):
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))])
    u = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(u, s, poly[:, 0]), np.interp(u, s, poly[:, 1])], axis=1)


def snap_dwl(poly, half):
    """Move the girth point nearest the DWL onto the section's DWL crossing.

    The flare moves only points above z = 0; with a vertex on z = 0 the edge
    that crosses the waterline keeps its crossing, so the waterplane stays put.
    """
    k = np.nonzero((poly[:-1, 1] <= 0.0) & (poly[1:, 1] > 0.0))[0]
    if not len(k):
        return half
    p, q = poly[k[-1]], poly[k[-1] + 1]
    c = p + (0.0 - p[1]) / (q[1] - p[1]) * (q - p)
    j = 1 + np.argmin(np.linalg.norm(half[1:-1] - c, axis=1))
    out = half.copy()
    out[j] = [c[0], 0.0]
    return out


def to_sheer(poly, z_sheer):
    """Trim a half section at the sheer height, or extend its top vertically to it."""
    if poly[-1, 1] < z_sheer:
        return np.vstack([poly, [poly[-1, 0], z_sheer]])
    k = np.nonzero(poly[:, 1] <= z_sheer)[0].max()
    p, q = poly[k], poly[k + 1]
    s = (z_sheer - p[1]) / (q[1] - p[1])
    return np.vstack([poly[: k + 1], p + s * (q - p)])


class Morph:
    """Parent sections plus the maps that place them in the body frame."""

    def __init__(self, tris, stations, girth, deck):
        self.x_min = tris[:, :, 0].min()
        self.x_max = tris[:, :, 0].max()
        self.z_top = tris[:, :, 2].max()
        eps = 1e-5 * (self.x_max - self.x_min)
        s = np.linspace(0.0, 1.0, stations)
        self.xs = self.x_min + eps + (self.x_max - self.x_min - 2 * eps) * 0.5 * (1 - np.cos(np.pi * s))
        self.sections = [cut_section(tris, x) for x in self.xs]
        self.girth = girth
        self.deck = deck
        self.stem_z = None

    def x_map(self, x, sx, x0, fwd=None, aft=None):
        """Affine x scale, then a C1 quadratic stretch of each overhang beyond the waterline ends."""
        xb = sx * (x - x0)
        for o in (fwd, aft):
            if o is None:
                continue
            u = (xb - o["x_wl"]) * o["sign"]
            m = u > 0
            xb = np.where(m, xb + o["sign"] * o["a"] * u**2, xb)
        return xb

    def rings(self, sx, sy, sz, x0, z_dwl, fwd=None, aft=None, sheer=None, flare=None):
        xb = self.x_map(self.xs, sx, x0, fwd, aft)
        rings = []
        for x, sec in zip(xb, self.sections):
            half = np.stack([sec[:, 0] * sy, (sec[:, 1] - z_dwl) * sz], axis=1)
            if sheer is not None:
                half = to_sheer(half, sheer(x))
            half = snap_dwl(half, resample(half, self.girth))
            if flare is not None:
                half = flare_half(flare, x, half)
            top = half[-1]
            deck = np.stack(
                [np.linspace(top[0], -top[0], self.deck + 2)[1:-1], np.full(self.deck, top[1])], axis=1
            )
            port = half[1:][::-1] * [-1.0, 1.0]
            yz = np.vstack([half, deck, port])
            rings.append(np.column_stack([np.full(len(yz), x), yz]))
        tip_x = self.x_map(np.array([self.x_max]), sx, x0, fwd, aft)[0]
        tip_z = sheer(tip_x) if sheer is not None else (self.z_top - z_dwl) * sz
        return np.array(rings), np.array([tip_x, 0.0, tip_z])


def taper(x, peak, hw_fwd, hw_aft):
    """cos^2 weight in x: 1 at the peak, 0 at hw_fwd forward of it and hw_aft aft of it."""
    d = np.asarray(x, dtype=float) - peak
    u = np.clip(np.where(d > 0, d / hw_fwd, -d / hw_aft), 0.0, 1.0)
    return np.cos(0.5 * np.pi * u) ** 2


def straighten(top, beta):
    """Scale the turning at every vertex of a polyline by 1 - beta, then scale it about its base to its old height.

    Each vertex keeps its turning direction and the first segment keeps its
    direction, so the section gains no inflection and no slope step at the base.
    """
    d = np.diff(top, axis=0)
    ang = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    ang = ang[0] + (1.0 - beta) * (ang - ang[0])
    seg = np.linalg.norm(d, axis=1)[:, None] * np.stack([np.cos(ang), np.sin(ang)], axis=1)
    new = top[0] + np.vstack([[0.0, 0.0], np.cumsum(seg, axis=0)])
    return top[0] + (new - top[0]) * (top[-1, 1] - top[0, 1]) / (new[-1, 1] - top[0, 1])


def flare_beta(flare, x, half):
    """Base index and straightening fraction that put the sheer at y_sheer (1 + (k0 - 1) w(x))."""
    j0 = np.nonzero(half[:, 1] <= 0.0)[0].max() if half[:, 1].min() <= 0.0 else 0
    target = half[-1, 0] * (1.0 + (flare["k0"] - 1.0) * taper(x, flare["peak_x"], flare["hw_fwd"], flare["hw_aft"]))
    lo, hi = 0.0, FLARE_BETA_MAX
    if target >= half[-1, 0] or len(half) - j0 < 3:
        return j0, 0.0
    for _ in range(40):
        b = 0.5 * (lo + hi)
        lo, hi = (b, hi) if straighten(half[j0:], b)[-1, 0] > target else (lo, b)
    return j0, lo


def flare_half(flare, x, half):
    """Flare a starboard half section above the DWL by straightening it from its DWL vertex up."""
    j0, beta = flare_beta(flare, x, half)
    out = half.copy()
    if beta > 0.0:
        out[j0:] = straighten(half[j0:], beta)
    return out


def dwl_kink(half0, half1):
    """Change, in degrees, of the turning angle at the DWL vertex between an unflared and a flared half section."""
    if half0[:, 1].min() > 0.0:
        return 0.0
    j = np.nonzero(half0[:, 1] <= 0.0)[0].max()
    if j == 0 or j == len(half0) - 1:
        return 0.0

    def turn(h):
        a, b = h[j] - h[j - 1], h[j + 1] - h[j]
        return math.atan2(a[0] * b[1] - a[1] * b[0], a @ b)

    return abs(math.degrees(turn(half1) - turn(half0)))


def keel_fairing_gap(rings, x_le, chord, root_z, n=49):
    """Height from the keel root up to the canoe body on the centreline, sampled LE to TE."""
    xq = np.linspace(x_le, x_le - chord, n)
    return np.interp(xq, rings[:, 0, 0], rings[:, 0, 2]) - root_z


def turning(half):
    """Turning direction at each interior point of a half section: +1, -1, or 0 on a straight run."""
    a, b = np.diff(half, axis=0)[:-1], np.diff(half, axis=0)[1:]
    c = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    return np.where(np.abs(c) > 1e-6 * np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1), np.sign(c), 0.0)


def inflections(half):
    sgn = turning(half)
    sgn = sgn[sgn != 0]
    return int(np.count_nonzero(sgn[1:] != sgn[:-1]))


def max_beam(rings):
    i = np.unravel_index(np.argmax(np.abs(rings[:, :, 1])), rings.shape[:2])
    return 2.0 * abs(rings[i][1]), rings[i][0]


def tube(rings, apex):
    """Closed triangle mesh from station rings, a transom fan and a stem apex."""
    n, k, _ = rings.shape
    verts = np.vstack([rings.reshape(-1, 3), [rings[0].mean(axis=0)], [apex]])
    c_aft, c_fwd = n * k, n * k + 1
    i = np.arange(n - 1)[:, None] * k
    j = np.arange(k)[None, :]
    a, b = i + j, i + (j + 1) % k
    faces = [np.stack([a, b, b + k], -1).reshape(-1, 3), np.stack([a, b + k, a + k], -1).reshape(-1, 3)]
    faces.append(np.stack([np.full(k, c_aft), (np.arange(k) + 1) % k, np.arange(k)], -1))
    last = (n - 1) * k
    faces.append(np.stack([np.full(k, c_fwd), last + np.arange(k), last + (np.arange(k) + 1) % k], -1))
    faces = np.vstack(faces)
    if signed_volume(verts, faces) < 0:
        faces = faces[:, ::-1]
    return verts, faces


def signed_volume(v, f):
    p = v[f]
    return np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6.0


def hydro(verts, faces):
    """Upright hydrostatics at z = 0 of a closed full hull, plus LCF and LCB in body x."""
    tris = verts[faces]
    zmin = tris[:, :, 2].min()
    h = hydrostatics(tris, -zmin)
    wet, _ = clip_below(tris, 0.0)
    p0, p1, p2 = wet[:, 0], wet[:, 1], wet[:, 2]
    nz = 0.5 * np.cross(p1 - p0, p2 - p0)[:, 2]
    xc = wet[:, :, 0].mean(axis=1)
    h["x_fp"] = h["x_fp_m"]
    h["x_ap"] = h["x_fp_m"] - h["lwl_m"]
    h["lcb_x"] = h["x_fp_m"] - h["lcb_aft_fp_m"]
    h["lcf_x"] = np.sum(nz * xc) / np.sum(nz)
    h["tc_m"] = -zmin
    return h


def corner_normals(v, f, crease_deg):
    """Area-weighted vertex normals, split across edges sharper than the crease angle."""
    fn = 0.5 * np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    fu = fn / np.maximum(np.linalg.norm(fn, axis=1), 1e-20)[:, None]
    cos_c = math.cos(math.radians(crease_deg))
    vid = f.ravel()
    fid = np.repeat(np.arange(len(f)), 3)
    order = np.argsort(vid, kind="stable")
    bounds = np.searchsorted(vid[order], np.arange(len(v) + 1))
    out = np.zeros((len(vid), 3))
    for vi in range(len(v)):
        cs = order[bounds[vi] : bounds[vi + 1]]
        fs = fid[cs]
        sim = fu[fs] @ fu[fs].T > cos_c
        n = sim.astype(float) @ fn[fs]
        out[cs] = n / np.maximum(np.linalg.norm(n, axis=1), 1e-20)[:, None]
    return out.reshape(-1, 3, 3)


def write_obj(path, v, f, normals=None, header=()):
    lines = [f"# {h}" for h in header]
    lines += [f"v {a:.5f} {b:.5f} {c:.5f}" for a, b, c in v]
    if normals is None:
        lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in f]
    else:
        q = np.round(normals.reshape(-1, 3), 4)
        uniq, inv = np.unique(q, axis=0, return_inverse=True)
        inv = inv.reshape(-1, 3)
        lines += [f"vn {a:.4f} {b:.4f} {c:.4f}" for a, b, c in uniq]
        lines += [
            f"f {a + 1}//{na + 1} {b + 1}//{nb + 1} {c + 1}//{nc + 1}"
            for (a, b, c), (na, nb, nc) in zip(f, inv)
        ]
    path.write_text("\n".join(lines) + "\n")


def read_obj(path):
    v, f = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            v.append([float(t) for t in line.split()[1:4]])
        elif line.startswith("f "):
            f.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    return np.array(v), np.array(f)


def physics_mesh(v, f, target):
    pv, pf = fast_simplification.simplify(v, f, target_count=target)
    # Rounded to the precision write_obj keeps, so vertices the file would
    # merge are merged here and the slivers between them dropped.
    m = trimesh.Trimesh(np.round(pv, 5), pf, process=True)
    m.update_faces(m.nondegenerate_faces())
    m.remove_unreferenced_vertices()
    if not (m.is_watertight and m.is_winding_consistent):
        raise RuntimeError("decimated physics mesh is not watertight")
    if m.volume < 0:
        m.invert()
    return np.asarray(m.vertices), np.asarray(m.faces)


def sheer_line(points):
    """Quadratic through (x, z) sheer points in the body frame."""
    x, z = np.array(points).T
    c = np.polyfit(x, z, 2)
    return lambda q: np.polyval(c, q)


def overhang(x_wl, x_tip, x_target, sign):
    o_p, o_t = sign * (x_tip - x_wl), sign * (x_target - x_wl)
    if o_t <= 0.5 * o_p:
        raise RuntimeError("overhang target too short for a monotonic stretch")
    return {"x_wl": x_wl, "sign": sign, "a": (o_t - o_p) / o_p**2, "parent": o_p, "target": o_t}


def r(x, n=4):
    return float(round(float(x), n))


def run(args):
    boat = args.boat_dir
    part = json.loads((boat / "particulars.json").read_text())
    app = json.loads((boat / "appendages.json").read_text())
    out_json = boat / "hull_hydro.json"
    prev = json.loads(out_json.read_text())
    inputs = prev["inputs"]
    cert = part["displacement"]["source"].split(", p.")[0]

    rho = 1000.0 * val(part, "flotation_sg")
    mass = val(part, "displacement")
    vol_total = mass / rho
    vol_keel = val(app["keel"], "volume_from_offsets")
    keel_c = np.array(val(app["keel"], "centroid"))
    vol_rud, rud_c, k_naca = rudder_volume(app["rudder"])
    vol_c = vol_total - vol_keel - vol_rud
    z_flot = 0.0
    root_z = val(app["keel"], "root_z") + z_flot
    root_le = np.array(val(app["keel"], "root_leading_edge"))
    root_chord = val(app["keel"], "root_chord")
    fair_area = app["keel"]["root_fairing"]["section_area_m2"]
    lwl_t = val(inputs, "lwl")
    aw_cert = val(part, "sink") * 1000.0 / rho

    table = pd.read_csv(args.table)
    ranked = rank_parents(table, lwl_t, vol_c, aw_cert)
    cols = ["hull_id", "overhang", "distance", "lwl_vol13", "j80_lwl_vol13", "bwl_tc", "j80_bwl_tc", "lwl_bwl", "j80_lwl_bwl"]
    print("Parent candidates (log-ratio distance; j80_* are the boat's values with that parent's Cw, Cb):")
    print(ranked[cols].head(5).round(3).to_string(index=False))
    parent, runner = ranked.iloc[0], ranked.iloc[1]
    bwl_t, tc_t = float(parent.bwl_target), float(parent.tc_target)

    tris = read_mesh(args.igs_dir / parent.file, args.deflection_mm)
    morph = Morph(tris, args.stations, args.girth, args.deck)
    x0 = parent.x_fp_m - 0.5 * parent.lwl_m
    sx, sy, sz = lwl_t / parent.lwl_m, bwl_t / parent.bwl_m, tc_t / parent.tc_m
    z_dwl = parent.z_dwl_m
    loa = val(part, "loa")
    fwd = aft = sheer = None
    stem_x = 0.0
    # The girth resampling spans the sheer extension, so the immersed points
    # move with the sheer and overhangs: iterate all of them together.
    for _ in range(30):
        rings, apex = morph.rings(sx, sy, sz, x0, z_dwl, fwd, aft, sheer)
        v, f = tube(rings, apex)
        h = hydro(v, f)
        gap = keel_fairing_gap(rings, root_le[0] + stem_x, root_chord, root_z)
        vol_fair = fair_area * gap.mean()
        vol_c = vol_total - vol_keel - vol_fair - vol_rud
        mid = 0.5 * (h["x_fp"] + h["x_ap"])
        errs = (vol_c / h["vol_m3"], lwl_t / h["lwl_m"], bwl_t / h["bwl_m"])
        stem_new = h["lcf_x"] - mid + val(part, "lcf_from_stem_cl")
        converged = (
            sheer is not None
            and max(abs(e - 1.0) for e in errs) < 1e-6
            and abs(mid) < 1e-6
            and abs(stem_new - stem_x) < 1e-6
        )
        if converged:
            break
        x0 += mid / sx
        sz *= errs[0]
        sx *= errs[1]
        sy *= errs[2]
        stem_x = stem_new
        xb_tip = morph.x_map(np.array([morph.x_max]), sx, x0)[0]
        xb_tail = morph.x_map(np.array([morph.x_min]), sx, x0)[0]
        fwd = overhang(0.5 * lwl_t, xb_tip, stem_x, 1.0)
        aft = overhang(-0.5 * lwl_t, xb_tail, stem_x - loa, -1.0)
        sheer_pts = [
            (stem_x - val(part, "sffp"), val(part, "freeboard_fwd_ffm")),
            (stem_x - val(part, "rig_j"), val(part, "freeboard_at_mast")),
            (stem_x - val(part, "safp"), val(part, "freeboard_aft_fam")),
        ]
        sheer = sheer_line(sheer_pts)
    else:
        raise RuntimeError("scale iteration did not converge")
    rings0, h0 = rings, h
    beam0, beam0_x = max_beam(rings0)
    beam_t, station_t = val(part, "beam"), val(part, "max_beam_station_from_stem")
    w_transom = 0.6
    hw_fwd = station_t
    hw_aft = (loa - station_t) / (2.0 / np.pi * math.acos(math.sqrt(w_transom)))
    flare = {"peak_x": stem_x - station_t, "hw_fwd": hw_fwd, "hw_aft": hw_aft}
    g = morph.girth
    sheer_y = np.abs(rings0[:, g - 1, 1])
    w_st = taper(rings0[:, 0, 0], flare["peak_x"], hw_fwd, hw_aft)
    lo, hi = 1.0, 2.0
    for _ in range(60):
        flare["k0"] = 0.5 * (lo + hi)
        beam = 2.0 * np.max(sheer_y * (1.0 + (flare["k0"] - 1.0) * w_st))
        lo, hi = (flare["k0"], hi) if beam < beam_t else (lo, flare["k0"])
    rings, apex = morph.rings(sx, sy, sz, x0, z_dwl, fwd, aft, sheer, flare)
    v, f = tube(rings, apex)
    hf = hydro(v, f)
    beam1, beam1_x = max_beam(rings)
    betas = np.array([flare_beta(flare, q[0, 0], q[:g, 1:])[1] for q in rings0])
    infl0 = np.array([inflections(q[:g, 1:]) for q in rings0])
    infl1 = np.array([inflections(q[:g, 1:]) for q in rings])
    new_infl = np.nonzero(infl1 > infl0)[0]
    kinks = np.array([dwl_kink(q0[:g, 1:], q1[:g, 1:]) for q0, q1 in zip(rings0, rings)])
    onset = []
    for i in new_infl:
        t0, t1 = turning(rings0[i, :g, 1:]), turning(rings[i, :g, 1:])
        k = np.nonzero((t1 != 0) & (t1 != t0))[0]
        if len(k):
            onset.append(rings[i, k[0] + 1, 2] / rings[i, g - 1, 2])
    onset = [r(min(onset), 2), r(max(onset), 2)] if onset else []
    xb_st = morph.x_map(morph.xs, sx, x0, fwd, aft)
    parent_fb = (morph.z_top - z_dwl) * sz
    j80_fb = sheer(xb_st)

    pv, pf = physics_mesh(v, f, args.physics_tris)
    normals = corner_normals(v, f, args.crease_deg)
    frame_note = "body frame: x forward, y port, z up, m; origin at Lwl midpoint on the DWL, centreline"
    write_obj(boat / "hull_visual.obj", v, f, normals, header=(f"J/80 visual hull morphed from DSYHS {parent.file}", frame_note))
    write_obj(boat / "hull_physics.obj", pv, pf, header=(f"J/80 physics hull decimated from hull_visual.obj", frame_note))

    vol_tot_mesh = hf["vol_m3"] + vol_keel + vol_fair + vol_rud
    keel_b = keel_c + [stem_x, 0.0, 0.0]
    rud_b = rud_c + [stem_x, 0.0, 0.0]
    fair_x = root_le[0] + stem_x - naca_area_and_centroid()[1] * root_chord
    lcb_total = (
        hf["vol_m3"] * hf["lcb_x"] + vol_keel * keel_b[0] + vol_fair * fair_x + vol_rud * rud_b[0]
    ) / vol_tot_mesh
    app_ws = 2.0 * (val(app["keel"], "lateral_area") + val(app["rudder"], "lateral_area"))
    ws_hull_est = val(part, "wetted_surface") - app_ws
    tc_keel_root = -val(app["keel"], "root_z")
    beam_max = 2.0 * np.abs(v[:, 1]).max()
    parent_src = f"DSYHS model {int(parent.hull_id)}, data/dsyhs/table.csv"
    jb = inputs["lwl"]["source"]

    targets = {
        "lwl": inputs["lwl"],
        "bwl": item(
            r(bwl_t), "m", f"{cert}, p.1 (sink); {parent_src}", "derived",
            method=(
                f"Aw / (Cw Lwl): waterplane area from the certificate sink, {val(part, 'sink')} kg/mm / "
                f"rho = {aw_cert:.3f} m^2, with the parent's Cw {parent.cw:.4f}."
            ),
            note=(
                f"Flare cross-check: certificate beam {val(part, 'beam')} m / Bwl = "
                f"{val(part, 'beam') / bwl_t:.3f}; the parent's own Bmax/Bwl is "
                f"{2 * np.abs(tris[:, :, 1]).max() / parent.bwl_m:.3f}."
            ),
        ),
        "tc": item(
            r(tc_t), "m", f"{cert}, p.1; {parent_src}", "derived",
            method=(
                f"canoe volume / (Cb Lwl Bwl) with the parent's Cb {parent.cb:.4f}; the initial guess for "
                "the vertical scale, which is then iterated on volume."
            ),
            note=(
                f"Shallower than the keel root depth below the flotation plane, {tc_keel_root:.3f} m "
                "(appendages.json keel root_z), which would put the canoe body deeper than any DSYHS "
                "Cb allows at this Lwl, Bwl and volume. The keel root fairing fills the gap."
            ),
        ),
        "density": item(rho, "kg/m^3", part["flotation_sg"]["source"], "high", method="flotation SG x 1000"),
        "displacement_volume": item(r(vol_total, 5), "m^3", f"{cert}, p.1", "high", method=f"{mass:g} kg / rho"),
        "keel_volume": item(vol_keel, "m^3", app["keel"]["volume_from_offsets"]["source"], "estimated", method="appendages.json keel volume_from_offsets"),
        "keel_fairing_volume": item(
            r(vol_fair, 5), "m^3", app["keel"]["sections"]["source"], "estimated",
            method=(
                f"appendages.json keel root_fairing: section area {fair_area} m^2 times the mean height from "
                f"root_z up to the canoe body on the centreline over the root chord ({gap.mean():.4f} m); "
                "re-measured in each scale iteration."
            ),
        ),
        "rudder_volume": item(
            r(vol_rud, 5), "m^3", app["rudder"]["sections"]["source"], "estimated",
            method=(
                f"NACA 4-digit section area {k_naca:.4f} c t integrated trapezoidally over Sections 1-5; "
                "root at the flotation plane (appendages.json stock_axis), so all of it is immersed; "
                "nothing below Section 5."
            ),
        ),
        "canoe_volume": item(r(vol_c, 5), "m^3", f"{cert}, p.1; {app['keel']['sections']['source']}", "derived", method="displacement volume - keel - keel fairing - rudder"),
        "waterplane_area": item(r(aw_cert), "m^2", f"{cert}, p.1", "derived", method="sink kg/mm x 1000 / rho"),
        "lcb": item(
            r(hf["lcb_frac_lwl"]), "fraction of Lwl aft of the FP", parent_src, "derived",
            note="not on the certificate; the morph keeps the parent's longitudinal distribution",
        ),
        "cp": item(r(parent.cp), "-", parent_src, "derived", note="not on the certificate; the parent's Cp"),
    }

    result = {
        "lwl": r(hf["lwl_m"]),
        "bwl": r(hf["bwl_m"]),
        "tc": r(hf["tc_m"]),
        "canoe_volume": r(hf["vol_m3"], 5),
        "total_volume": r(vol_tot_mesh, 5),
        "displacement_kg": r(vol_tot_mesh * rho, 1),
        "lcb_x_canoe": r(hf["lcb_x"]),
        "lcb_frac_lwl_aft_fp": r(hf["lcb_frac_lwl"]),
        "lcb_x_total": r(lcb_total),
        "lcf_x": r(hf["lcf_x"]),
        "cp": r(hf["cp"]),
        "cm": r(hf["cm"]),
        "cb": r(hf["cb"]),
        "cw": r(hf["cw"]),
        "waterplane_area": r(hf["aw_m2"]),
        "max_section_area": r(hf["ax_m2"]),
        "wetted_area_canoe": r(hf["sc_m2"]),
        "keel_fairing_height_le": r(gap[0]),
        "keel_fairing_height_te": r(gap[-1]),
        "keel_fairing_height_range": [r(gap.min()), r(gap.max())],
        "keel_fairing_volume": r(vol_fair, 5),
        "displacement_breakdown": {
            "canoe": r(hf["vol_m3"], 5),
            "keel": vol_keel,
            "keel_fairing": r(vol_fair, 5),
            "rudder": r(vol_rud, 5),
            "total": r(vol_tot_mesh, 5),
        },
        "loa": r(v[:, 0].max() - v[:, 0].min()),
        "max_beam": r(beam_max),
        "max_beam_station_from_stem": r(stem_x - beam1_x),
        "units": "m, m^2, m^3, kg; x in the body frame",
    }

    doc = {
        "frame": (
            "Body frame: x forward, y to port, z up, metres. Origin on the centreline at the midpoint of "
            "Lwl on the DWL. The DWL is the certificate flotation plane (measurement trim, no crew), "
            "taken as level."
        ),
        "inputs": inputs,
        "transform": {
            "stem_x_in_body": r(stem_x),
            "flotation_z_in_body": 0.0,
            "rule": "p_body = p_provisional + (stem_x_in_body, 0, flotation_z_in_body), for the provisional frame of appendages.json, rig.json, sails.json and mass.json",
            "method": (
                f"Stem at the morphed hull's LCF plus the certificate LCF from stem on CL "
                f"({val(part, 'lcf_from_stem_cl')} m); the overhangs are stretched so the stem and transom "
                f"land there and LOA is {loa} m. The displacement target is the measurement-trim displacement, "
                "so the DWL is the flotation plane."
            ),
            "confidence": "derived",
        },
        "parent": {
            "hull_id": int(parent.hull_id),
            "overhang": float(parent.overhang),
            "file": parent.file,
            "distance": r(parent.distance),
            "runner_up": {"hull_id": int(runner.hull_id), "overhang": float(runner.overhang), "distance": r(runner.distance)},
            "top5": [
                {"hull_id": int(c.hull_id), "overhang": float(c.overhang), "distance": r(c.distance)}
                for c in ranked.head(5).itertuples()
            ],
            "method": (
                "Minimum of sqrt(sum ln(p_parent / p_boat)^2) over Lwl/∇c^(1/3), Bwl/Tc and Lwl/Bwl. "
                "The boat's Bwl and Tc are evaluated with each candidate's own Cw and Cb, so the distance "
                "measures how far the morph must distort that candidate. Cp and LCB are not on the "
                "certificate, so they are not in the distance; the morph inherits them. The ranking uses the "
                "canoe volume without the keel fairing, which is only known once the hull is morphed."
            ),
        },
        "scale": {
            "sx": r(sx, 5),
            "sy": r(sy, 5),
            "sz": r(sz, 5),
            "parent_x_at_body_origin": r(x0, 5),
            "parent_z_dwl": r(z_dwl, 5),
            "rule": "body = (sx (x - parent_x_at_body_origin), sy y, sz (z - parent_z_dwl)) in parent metres, model scale, then the overhang stretch",
            "overhang_fwd": {"parent_m": r(fwd["parent"]), "target_m": r(fwd["target"]), "a": r(fwd["a"], 5)},
            "overhang_aft": {"parent_m": r(aft["parent"]), "target_m": r(aft["target"]), "a": r(aft["a"], 5)},
            "overhang_rule": "beyond each waterline end, at distance u outside it, x moves outward by a u^2; the immersed hull is untouched",
        },
        "sheer": {
            "points": [[r(x), z] for x, z in sheer_pts],
            "source": f"{cert}, p.2 (FFM at SFFP, freeboard at mast at J, FAM at SAFP)",
            "fit": "quadratic through the three points",
            "parent_freeboard_scaled": r(parent_fb),
            "j80_freeboard_range": [r(j80_fb.min()), r(j80_fb.max())],
            "max_extension_m": r(max(0.0, (j80_fb - parent_fb).max())),
            "max_trim_m": r(max(0.0, (parent_fb - j80_fb).max())),
            "note": "Where the scaled DSYHS surface stops below the sheer the topsides are extended vertically; where it is above, it is trimmed. Deck flat at the sheer. The flare below is applied after this.",
        },
        "flare": {
            "rule": (
                "each half section is straightened above its DWL vertex (its lowest point where the section is "
                "clear of the water): the turning angle at every girth vertex above it is scaled by (1 - beta) "
                "and the straightened part is scaled uniformly about the DWL vertex back to the sheer height. "
                "beta (0 to FLARE_BETA_MAX) is solved per station so the sheer half-breadth becomes "
                "y_sheer (1 + (k0 - 1) w(x)); w = cos^2(pi/2 d / hw), d the distance from the peak, hw_fwd "
                "forward and hw_aft aft, 0 beyond hw"
            ),
            "k0": r(flare["k0"], 5),
            "beta_max_allowed": FLARE_BETA_MAX,
            "beta_range": [r(betas.min(), 3), r(betas.max(), 3)],
            "beta_at_peak": r(betas[np.argmin(np.abs(rings0[:, 0, 0] - flare["peak_x"]))], 3),
            "peak_from_stem": station_t,
            "half_width_fwd_m": r(hw_fwd),
            "half_width_aft_m": r(hw_aft),
            "w_at_transom": r(taper(stem_x - loa, flare["peak_x"], hw_fwd, hw_aft)),
            "method": (
                f"k0 solved by bisection for max beam {beam_t} m ({cert}, p.1). The peak is the certificate "
                f"max_beam_station_from_stem; hw_fwd puts w = 0 at the stem and hw_aft leaves w = {w_transom} "
                "at the transom so the wide stern is kept. Applied to the section points before the surface is "
                "closed, so topology and deck are unchanged. The girth point nearest the DWL is moved onto the "
                "section's DWL crossing and only points above it move, so the immersed hull, BWL and the "
                "waterplane do not. Straightening rather than adding an offset in z: every vertex keeps the "
                "sign of its turn, so the single-signed parent topsides stay single-signed, and the segment "
                "leaving the DWL vertex keeps its direction, so there is no slope step at the waterline. "
                "Offsets y s(z) or s(z) with s(0) = 0 either turn the topsides concave (s'' > 0, e.g. z^2) or "
                "put a reverse turn at the DWL (s'(0) > 0)."
            ),
            "unflared": {
                "canoe_volume": r(h0["vol_m3"], 6),
                "bwl": r(h0["bwl_m"], 6),
                "lwl": r(h0["lwl_m"], 6),
                "waterplane_area": r(h0["aw_m2"], 6),
                "max_beam": r(beam0),
                "max_beam_station_from_stem": r(stem_x - beam0_x),
            },
            "flared": {
                "canoe_volume": r(hf["vol_m3"], 6),
                "bwl": r(hf["bwl_m"], 6),
                "lwl": r(hf["lwl_m"], 6),
                "waterplane_area": r(hf["aw_m2"], 6),
                "max_beam": r(beam1),
                "max_beam_station_from_stem": r(stem_x - beam1_x),
            },
            "fairness": {
                "method": "count of turning-direction sign changes along each starboard half section, keel to sheer, straight runs ignored, before and after the flare",
                "stations": int(len(rings)),
                "stations_with_new_inflections": int(len(new_infl)),
                "new_inflection_span_from_stem_m": (
                    [r(stem_x - rings[new_infl, 0, 0].max(), 3), r(stem_x - rings[new_infl, 0, 0].min(), 3)]
                    if len(new_infl) else []
                ),
                "onset_fraction_of_freeboard": onset,
                "dwl_kink_method": "change in the turning angle at each section's DWL vertex, flared minus unflared",
                "dwl_kink_deg_max": r(kinks.max(), 3),
                "max_fraction_with_new_inflections": FLARE_INFLECTION_MAX,
            },
            "confidence": "estimated",
        },
        "targets": targets,
        "result": result,
        "cross_checks": {
            "loa": {"mesh": result["loa"], "certificate": loa},
            "max_beam": {"mesh": result["max_beam"], "certificate": val(part, "beam")},
            "tc_vs_keel_root": {
                "mesh": result["tc"],
                "keel_root_depth": r(tc_keel_root),
                "note": "the keel root fairing (result keel_fairing_*) fills the gap, so root_z and the certificate draft are kept",
            },
            "wetted_area_canoe": {
                "mesh": result["wetted_area_canoe"],
                "certificate_ws_minus_appendages": r(ws_hull_est),
                "method": "certificate WS minus twice the keel and rudder lateral areas",
            },
            "waterplane_area": {"mesh": result["waterplane_area"], "certificate_sink": r(aw_cert)},
        },
        "meshes": {
            "visual": {
                "file": "hull_visual.obj",
                "triangles": int(len(f)),
                "vertices": int(len(v)),
                "note": "for rendering only; split vertex normals at creases, not required to be watertight",
            },
            "physics": {
                "file": "hull_physics.obj",
                "triangles": int(len(pf)),
                "vertices": int(len(pv)),
                "note": "watertight, outward winding; checked by --check",
            },
        },
        "source_note": f"{jb}; {cert}; {app['keel']['sections']['source']}; DSYHS geometry doi:10.4121/21501330",
    }
    out_json.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    print(f"parent {parent.file}, runner-up model {int(runner.hull_id)}; sx {sx:.4f} sy {sy:.4f} sz {sz:.4f}")
    print(f"visual {len(f)} tris, physics {len(pf)} tris; stem_x_in_body {stem_x:.4f}")
    print(
        f"keel fairing: height LE {gap[0]:.4f} TE {gap[-1]:.4f} (range {gap.min():.4f}-{gap.max():.4f}) m, "
        f"volume {vol_fair:.5f} m^3"
    )
    print(
        f"flare: k0 {flare['k0']:.5f}, half widths fwd {hw_fwd:.3f} aft {hw_aft:.3f} m; max beam "
        f"{beam0:.4f} -> {beam1:.4f} m at {stem_x - beam0_x:.3f} -> {stem_x - beam1_x:.3f} m from stem; "
        f"beta {betas.min():.3f}-{betas.max():.3f}"
    )
    print(
        f"fairness: {len(new_infl)} of {len(rings)} sections gain an inflection"
        + (f", the reversal starting at {onset[0]:.2f}-{onset[1]:.2f} of the local freeboard" if onset else "")
        + f"; DWL kink at most {kinks.max():.2g} deg"
    )


def check(boat):
    doc = json.loads((boat / "hull_hydro.json").read_text())
    part = json.loads((boat / "particulars.json").read_text())
    keel = json.loads((boat / "appendages.json").read_text())["keel"]
    t = doc["targets"]
    vol_app = val(t, "keel_volume") + val(t, "keel_fairing_volume") + val(t, "rudder_volume")
    stem_x = doc["transform"]["stem_x_in_body"]
    z_flot = doc["transform"]["flotation_z_in_body"]
    ok = True

    def flag(p):
        nonlocal ok
        ok &= bool(p)
        return "PASS" if p else "FAIL"

    def line(name, got, want):
        nonlocal ok
        e = 100.0 * (got - want) / want
        p = abs(e) <= 2.0
        ok &= p
        return f"  {name:<14} {got:9.4f} vs {want:9.4f}  {e:+6.2f}%  {'PASS' if p else 'FAIL'}"

    vols = {}
    for name in ("visual", "physics"):
        v, f = read_obj(boat / doc["meshes"][name]["file"])
        h = hydro(v, f)
        vols[name] = signed_volume(v, f)
        if name == "visual":
            vis_v, vis_h = v, h
        print(f"{name} mesh ({len(f)} tris):")
        print(line("displacement", h["vol_m3"] + vol_app, val(t, "displacement_volume")) + "  m^3")
        print(line("LWL", h["lwl_m"], val(t, "lwl")) + "  m")
        print(line("BWL", h["bwl_m"], val(t, "bwl")) + "  m")
        if name == "physics":
            m = trimesh.Trimesh(v, f, process=True)
            wt = m.is_watertight and m.is_winding_consistent and m.volume > 0
            ok &= wt
            print(f"  watertight {m.is_watertight}, winding consistent {m.is_winding_consistent}, outward {m.volume > 0}  {'PASS' if wt else 'FAIL'}")
    e = 100.0 * (vols["physics"] - vols["visual"]) / vols["visual"]
    p = abs(e) <= 1.0
    ok &= p
    print(f"physics vs visual enclosed volume: {vols['physics']:.4f} vs {vols['visual']:.4f} m^3  {e:+.2f}%  {'PASS' if p else 'FAIL'}")

    root_z = val(keel, "root_z") + z_flot
    x_le = val(keel, "root_leading_edge")[0] + stem_x
    cl = vis_v[np.abs(vis_v[:, 1]) < 1e-6]
    xs, inv = np.unique(cl[:, 0], return_inverse=True)
    zb = np.full(len(xs), np.inf)
    np.minimum.at(zb, inv, cl[:, 2])
    gap = np.interp(np.linspace(x_le, x_le - val(keel, "root_chord"), 49), xs, zb) - root_z
    res = doc["result"]
    p = gap.min() > 0.0 and gap.max() < 0.15
    print(f"keel fairing height along the root chord (visual mesh): {gap.min():.4f} to {gap.max():.4f} m, in (0, 0.15)  {flag(p)}")
    p = abs(gap[0] - res["keel_fairing_height_le"]) < 1e-3 and abs(gap[-1] - res["keel_fairing_height_te"]) < 1e-3
    print(f"  LE {gap[0]:.4f} vs {res['keel_fairing_height_le']:.4f}, TE {gap[-1]:.4f} vs {res['keel_fairing_height_te']:.4f} m (hull_hydro.json), within 1 mm  {flag(p)}")
    tip = root_z - val(keel, "span")
    p = abs(tip + val(part, "draft")) <= 0.005
    print(f"keel tip z: root_z - span = {tip:.4f} m vs -draft {-val(part, 'draft'):.4f} m, within 5 mm  {flag(p)}")

    i = np.argmax(np.abs(vis_v[:, 1]))
    beam, station = 2.0 * abs(vis_v[i, 1]), stem_x - vis_v[i, 0]
    print(line("max beam", beam, val(part, "beam")) + "  m (visual)")
    st_t = val(part, "max_beam_station_from_stem")
    p = abs(station - st_t) <= 0.25
    print(f"  max-beam station from stem {station:.3f} vs {st_t:.3f} m, within 0.25 m  {flag(p)}")
    un = doc["flare"]["unflared"]
    for name, got, want in (("canoe volume", vis_h["vol_m3"], un["canoe_volume"]), ("BWL", vis_h["bwl_m"], un["bwl"])):
        e = (got - want) / want
        print(f"flare leaves {name} unchanged: {got:.6f} vs unflared {want:.6f}  {e:+.1e} rel, within 1e-4  {flag(abs(e) <= 1e-4)}")
    fair = doc["flare"]["fairness"]
    n_new, n_st = fair["stations_with_new_inflections"], fair["stations"]
    p = n_new <= FLARE_INFLECTION_MAX * n_st
    print(
        f"flare fairness: {n_new} of {n_st} sections gain an inflection, at most {FLARE_INFLECTION_MAX:.0%}; "
        f"DWL kink at most {fair['dwl_kink_deg_max']:.2g} deg (hull_hydro.json)  {flag(p)}"
    )
    print("ALL PASS" if ok else "FAIL")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boat-dir", type=Path, default=ROOT / "data/boats/j80")
    ap.add_argument("--table", type=Path, default=ROOT / "data/dsyhs/table.csv")
    ap.add_argument("--igs-dir", type=Path, default=ROOT / "data/dsyhs/raw/IGS")
    ap.add_argument("--deflection-mm", type=float, default=0.05)
    ap.add_argument("--stations", type=int, default=112)
    ap.add_argument("--girth", type=int, default=40, help="points per half section, keel to sheer")
    ap.add_argument("--deck", type=int, default=11, help="interior deck points per station")
    ap.add_argument("--physics-tris", type=int, default=1600)
    ap.add_argument("--crease-deg", type=float, default=40.0)
    ap.add_argument("--check", action="store_true", help="only recompute and check the written meshes")
    args = ap.parse_args(argv)
    if not args.check:
        run(args)
    sys.exit(0 if check(args.boat_dir) else 1)


if __name__ == "__main__":
    main()
