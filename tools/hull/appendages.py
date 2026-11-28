"""Build the keel and rudder meshes and foil specs from a boat's appendage offsets.

Each appendage is a stack of horizontal NACA 4-digit sections taken from the
class-rule offset table in appendages.json, linear in depth between table
sections, closed into one watertight solid with outward winding. The keel
gets an elliptical tip below its last section and a root fairing: Section 1
extruded up to the canoe body and on into it by OVERLAP, so the solid joins
the hull with no gap. The rudder hangs from its stock axis with a flat root
and a flat foot at its last table section.

Writes appendages.obj (objects `keel` and `rudder`) and the `foils` block of
appendages.json. --check recomputes everything from the written OBJ and the
hull meshes and exits non-zero on any miss.

Body frame: x forward, y to port, z up, metres; origin on the centreline at
the midpoint of Lwl on the DWL.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import manifold3d
import numpy as np
import trimesh

# -I leaves the script directory off sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from morph import NACA_A, NACA_P, item, read_obj, val  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]

OVERLAP = 0.02
CHORD_POINTS = {"keel": 40, "rudder": 32}
TIP_RINGS = 10
RUDDER_SUBDIV = 3
# Shrinking the tip sections about the max-thickness station keeps the
# elliptical tip centred under the bulb's fullest part rather than its nose.
TIP_CENTRE_XC = 0.3
TOL_VOLUME = 0.05
TOL_AREA = 0.05
TOL_TIP = 0.005
TOL_GAP = 0.005
TOL_STORED = 1e-3
# A fin under a hull with no gap reflects in the hull bottom, doubling its
# effective span for induced drag; heel, leeway and the hull's finite width
# make this an upper bound.
KEEL_AR_FACTOR = 2.0
RUDDER_AR_FACTOR = 1.0


def naca_half(xc, t):
    return 5.0 * t * sum(a * xc**p for a, p in zip(NACA_A, NACA_P))


def chord_stations(n):
    return 0.5 * (1.0 - np.cos(np.linspace(0.0, math.pi, n + 1)))


def ring(x_le, chord, thick, z, xc):
    """Closed section polygon: port side LE to TE, then starboard TE back to just aft of the LE."""
    x = x_le - xc * chord
    y = naca_half(xc, thick / chord) * chord
    up = np.stack([x, y, np.full_like(x, z)], -1)
    lo = np.stack([x[:0:-1], -y[:0:-1], np.full(len(x) - 1, z)], -1)
    return np.vstack([up, lo])


def bands(n_rings, k):
    i = np.arange(n_rings - 1)[:, None] * k
    j = np.arange(k)[None, :]
    a, b = i + j, i + (j + 1) % k
    return np.vstack([np.stack([a, b + k, b], -1).reshape(-1, 3), np.stack([a, a + k, b + k], -1).reshape(-1, 3)])


def cap(start, n, up):
    """Strip triangulation of a ring's interior, pairing port and starboard points at the same x/c."""
    k = 2 * n + 1
    port = start + np.arange(n + 1)
    stbd = start + np.array([0] + [k - i for i in range(1, n + 1)])
    faces = [[port[0], port[1], stbd[1]]]
    for i in range(1, n):
        faces += [[port[i], port[i + 1], stbd[i + 1]], [port[i], stbd[i + 1], stbd[i]]]
    faces = np.array(faces)
    return faces[:, ::-1] if up else faces


def orient(v, f, n_cap_faces):
    """Flip the side bands to match the caps if needed, then everything if the volume is negative."""
    m = trimesh.Trimesh(v, f, process=False)
    if not m.is_winding_consistent:
        f = f.copy()
        side = len(f) - n_cap_faces
        f[:side] = f[:side, ::-1]
    if signed_volume(v, f) < 0:
        f = f[:, ::-1]
    return f


def signed_volume(v, f):
    p = v[f]
    return np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6.0


def hull_bottom_z(tris, xy):
    """Lowest z of the hull surface straight above or below each (x, y)."""
    lo, hi = xy.min(axis=0) - 0.05, xy.max(axis=0) + 0.05
    tmin, tmax = tris[:, :, :2].min(axis=1), tris[:, :, :2].max(axis=1)
    t = tris[np.all((tmax >= lo) & (tmin <= hi), axis=1)]
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    out = np.full(len(xy), np.nan)
    for i, p in enumerate(xy):
        v0, v1, v2 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2], p - a[:, :2]
        den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
        ok = np.abs(den) > 1e-14
        den = np.where(ok, den, 1.0)
        u = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
        w = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
        hit = ok & (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9)
        if hit.any():
            z = a[hit, 2] + u[hit] * (b[hit, 2] - a[hit, 2]) + w[hit] * (c[hit, 2] - a[hit, 2])
            out[i] = z.min()
    return out


def winding(tris, pts, chunk=8):
    """Generalized winding number of each point; about 1 inside a closed surface, 0 outside."""
    out = np.empty(len(pts))
    for s in range(0, len(pts), chunk):
        p = pts[s : s + chunk, None, None, :]
        d = tris[None] - p
        n = np.linalg.norm(d, axis=-1)
        a, b, c = d[..., 0, :], d[..., 1, :], d[..., 2, :]
        na, nb, nc = n[..., 0], n[..., 1], n[..., 2]
        num = np.einsum("...i,...i", a, np.cross(b, c))
        den = na * nb * nc + np.einsum("...i,...i", a, b) * nc + np.einsum("...i,...i", b, c) * na + np.einsum("...i,...i", c, a) * nb
        out[s : s + chunk] = np.arctan2(num, den).sum(axis=1) / (2.0 * math.pi)
    return out


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.einsum("...i,...i", p - a, ab) / np.maximum(np.einsum("...i,...i", ab, ab), 1e-30), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1)


def surface_dist(tris, pts, chunk=32):
    """Distance from each point to the nearest point of a triangle soup."""
    out = np.empty(len(pts))
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    size = np.linalg.norm(nrm, axis=1)
    # A zero-area triangle has no plane, and its zero normal would pass the
    # inside test for every point; its edges are covered by its neighbours.
    keep = size > 1e-14
    tris, nrm = tris[keep], nrm[keep] / size[keep, None]
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    for s in range(0, len(pts), chunk):
        p = pts[s : s + chunk, None, :]
        h = np.einsum("ptj,tj->pt", p - a, nrm)
        q = p - h[..., None] * nrm
        inside = np.ones(h.shape, bool)
        for u, w in ((a, b), (b, c), (c, a)):
            inside &= np.einsum("ptj,tj->pt", np.cross(w - u, q - u), nrm) >= 0
        d = np.minimum(np.minimum(seg_dist(p, a, b), seg_dist(p, b, c)), seg_dist(p, c, a))
        out[s : s + chunk] = np.where(inside, np.abs(h), d).min(axis=1)
    return out


def table(sec, key, default=None):
    return np.array([s.get(key, default) for s in sec], dtype=float)


def keel_mesh(keel, stem_x, z_flot, hull_tris):
    sec = val(keel, "sections")
    d, c, t = table(sec, "depth_below_root_m"), table(sec, "chord_m"), table(sec, "max_thickness_m")
    le_aft = table(sec, "leading_edge_x_aft_of_root_le_m")
    x0 = val(keel, "root_leading_edge")[0] + stem_x
    z0 = val(keel, "root_z") + z_flot
    h_tip = val(keel, "span") - d[-1]
    n = CHORD_POINTS["keel"]
    xc = chord_stations(n)

    root = ring(x0, c[0], t[0], z0, xc)
    z_hull = hull_bottom_z(hull_tris, root[:, :2])
    rings = [root.copy(), root.copy()]
    rings[0][:, 2] = z_hull + OVERLAP
    rings[1][:, 2] = z_hull
    rings += [ring(x0 - a, ci, ti, z0 - di, xc) for a, ci, ti, di in zip(le_aft, c, t, d)]
    x_mid = x0 - le_aft[-1] - TIP_CENTRE_XC * c[-1]
    for th in np.linspace(0.0, 0.5 * math.pi, TIP_RINGS + 1)[1:-1]:
        q = math.cos(th)
        rings.append(ring(x_mid + TIP_CENTRE_XC * c[-1] * q, c[-1] * q, t[-1] * q, z0 - d[-1] - h_tip * math.sin(th), xc))
    k = 2 * n + 1
    apex = np.array([[x_mid, 0.0, z0 - d[-1] - h_tip]])
    v = np.vstack(rings + [apex])
    side = bands(len(rings), k)
    last = (len(rings) - 1) * k
    fan = np.stack([np.full(k, len(v) - 1), last + (np.arange(k) + 1) % k, last + np.arange(k)], -1)
    top = cap(0, n, up=True)
    f = orient(v, np.vstack([side, fan, top]), len(top))
    return v, f, {"z_hull": z_hull, "footprint_area": polygon_area(root[:, :2])}


def rudder_mesh(rudder, stem_x, z_flot):
    sec = val(rudder, "sections")
    d, c, t = table(sec, "depth_below_root_m"), table(sec, "chord_m"), table(sec, "max_thickness_m")
    axis = np.array(val(rudder, "stock_axis")) + [stem_x, 0.0, z_flot]
    n = CHORD_POINTS["rudder"]
    xc = chord_stations(n)
    s = np.concatenate([np.linspace(d[i], d[i + 1], RUDDER_SUBDIV + 1)[:-1] for i in range(len(d) - 1)] + [d[-1:]])
    rings = [ring(axis[0], np.interp(di, d, c), np.interp(di, d, t), axis[2] - di, xc) for di in s]
    k = 2 * n + 1
    v = np.vstack(rings)
    side = bands(len(rings), k)
    caps = np.vstack([cap(0, n, up=True), cap((len(rings) - 1) * k, n, up=False)])
    f = orient(v, np.vstack([side, caps]), len(caps))
    return v, f, {}


def polygon_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def write_obj(path, objs, header):
    lines = [f"# {h}" for h in header]
    base = 0
    for name, (v, f) in objs.items():
        lines.append(f"o {name}")
        lines += [f"v {a:.5f} {b:.5f} {c:.5f}" for a, b, c in v]
        lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in f + base]
        base += len(v)
    path.write_text("\n".join(lines) + "\n")


def read_objects(path):
    objs, name, verts, faces = {}, None, [], []
    for line in path.read_text().splitlines():
        if line.startswith("o "):
            name = line.split()[1]
            objs[name] = []
        elif line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            objs[name].append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    verts = np.array(verts)
    out = {}
    for name, f in objs.items():
        f = np.array(f)
        used, inv = np.unique(f, return_inverse=True)
        out[name] = (verts[used], inv.reshape(-1, 3))
    return out


def slice_extent(v, f, z):
    """x and y extent of the mesh's horizontal section at z."""
    e = np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    a, b = v[e[:, 0]], v[e[:, 1]]
    cross = ((a[:, 2] - z) * (b[:, 2] - z) <= 0) & (a[:, 2] != b[:, 2])
    a, b = a[cross], b[cross]
    p = a + ((z - a[:, 2]) / (b[:, 2] - a[:, 2]))[:, None] * (b - a)
    return p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()


def manifold(v, f):
    mesh = manifold3d.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(f, np.uint32))
    return manifold3d.Manifold(mesh)


def foil_props(v, f, z_top, z_root, z_tip_chord, z_sweep, hull_phys, hull_vis):
    """Planform, section and volume figures of one appendage, from its mesh and the hull."""
    z_min = v[:, 2].min()
    zs = np.linspace(z_min + 1e-7, z_top - 1e-7, 800)
    ext = np.array([slice_extent(v, f, z) for z in zs])
    chord, x_le = ext[:, 1] - ext[:, 0], ext[:, 1]
    x_qc = x_le - 0.25 * chord
    area = np.trapezoid(chord, zs)
    qc = [np.trapezoid(chord * x_qc, zs) / area, 0.0, np.trapezoid(chord * zs, zs) / area]
    fin = (zs >= z_sweep[0]) & (zs <= z_sweep[1])
    sweep_le = math.degrees(math.atan(np.polyfit(zs[fin], x_le[fin], 1)[0]))
    sweep_qc = math.degrees(math.atan(np.polyfit(zs[fin], x_qc[fin], 1)[0]))

    def section(z):
        x0, x1, y0, y1 = slice_extent(v, f, z)
        return x1 - x0, (y1 - y0) / (x1 - x0), x1

    c_root, tc_root, x_le_root = section(z_root)
    c_tip, tc_tip, _ = section(z_tip_chord)
    below = manifold(v, f) - manifold(*hull_phys)
    tris = v[f]
    cen = tris.mean(axis=1)
    z_hull = hull_bottom_z(hull_vis, cen[:, :2])
    wet = ~(cen[:, 2] >= z_hull) & (cen[:, 2] < -1e-6)
    tri_area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    span = z_top - z_min
    return {
        "span": span,
        "z_tip": z_min,
        "root_chord": c_root,
        "tip_chord": c_tip,
        "x_le_root": x_le_root,
        "sweep_le": sweep_le,
        "sweep_qc": sweep_qc,
        "tc_root": tc_root,
        "tc_tip": tc_tip,
        "area": area,
        "area_below": lambda z: np.trapezoid(chord[zs <= z], zs[zs <= z]),
        "ar": span**2 / area,
        "qc_centre": qc,
        "wetted_area": tri_area[wet].sum(),
        "volume_below_hull": below.volume(),
        "volume_mesh": signed_volume(v, f),
    }


def hull_tris(boat, name):
    v, f = read_obj(boat / name)
    return v, f, v[f]


def r(x, n=4):
    return float(round(float(x), n))


def keel_z_top(hull_vis, x_le, chord):
    """Mean height of the canoe body on the centreline over the keel root chord."""
    xq = np.linspace(x_le, x_le - chord, 49)
    return float(hull_bottom_z(hull_vis, np.stack([xq, np.zeros_like(xq)], -1)).mean())


def props_for(name, v, f, app, frame, hulls):
    stem_x, z_flot = frame
    _, _, vis = hulls["visual"]
    phys = hulls["physics"][:2]
    if name == "keel":
        keel = app["keel"]
        sec = val(keel, "sections")
        z0 = val(keel, "root_z") + z_flot
        z8 = z0 - sec[-1]["depth_below_root_m"]
        z4 = z0 - next(s for s in sec if s["section"] == 4)["depth_below_root_m"]
        x0 = val(keel, "root_leading_edge")[0] + stem_x
        z_top = keel_z_top(vis, x0, sec[0]["chord_m"])
        return foil_props(v, f, z_top, z0 - 1e-6, z8 + 1e-6, (z4, z0), phys, vis), z0
    z_top = v[:, 2].max()
    return foil_props(v, f, z_top, z_top - 1e-6, v[:, 2].min() + 1e-6, (v[:, 2].min(), z_top), phys, vis), z_top


def foils_block(props, app, frame, extra):
    k, ru = props["keel"], props["rudder"]
    h1 = app["keel"]["sections"]["source"]
    src = f"{h1}; appendages.obj from tools/hull/appendages.py"
    stem_x, z_flot = frame
    axis = val(app["rudder"], "stock_axis")
    stock = [r(axis[0] + stem_x), r(axis[1]), r(axis[2] + z_flot)]
    ks = val(app["keel"], "sections")
    h_tip = val(app["keel"], "span") - ks[-1]["depth_below_root_m"]
    c_min = min(s["chord_m"] for s in ks)
    narrow = [s["section"] for s in ks if s["chord_m"] == c_min]
    narrow_txt = f"Sections {narrow[0]}-{narrow[-1]}" if len(narrow) > 1 else f"Section {narrow[0]}"

    def foil(p, name, ar_factor, ar_method, span_method, tip_note, stock_items):
        out = {
            "span": item(r(p["span"]), "m", src, "estimated", method=span_method),
            "root_chord": item(r(p["root_chord"]), "m", src, "high", method="chord of the mesh section at the root (Section 1)"),
            "tip_chord": item(r(p["tip_chord"]), "m", src, "high", method="chord of the mesh section at the last table section", note=tip_note),
            "leading_edge_sweep": item(r(p["sweep_le"], 2), "deg", src, "high", method=extra[name]["sweep_method"].format(line="leading edge")),
            "quarter_chord_sweep": item(r(p["sweep_qc"], 2), "deg", src, "derived", method=extra[name]["sweep_method"].format(line="quarter-chord line")),
            "thickness_chord_ratio_root": item(r(p["tc_root"]), "-", src, "high", method="max thickness / chord of the mesh section at the root"),
            "thickness_chord_ratio_tip": item(r(p["tc_tip"]), "-", src, "high", method="max thickness / chord of the mesh section at the tip_chord station"),
            "planform_area": item(r(p["area"]), "m^2", src, "estimated", method=extra[name]["area_method"]),
            "geometric_aspect_ratio": item(r(p["ar"]), "-", src, "estimated", method="span^2 / planform_area"),
            "effective_aspect_ratio": item(r(ar_factor * p["ar"]), "-", src, "estimated", method=ar_method, note="An assumption, not a measurement; the physics may replace it with its own end-plate model."),
            "quarter_chord_centre": item([r(x) for x in p["qc_centre"]], "m", src, "estimated", method="chord-weighted mean of the quarter-chord points of horizontal mesh sections over the span, i.e. the quarter-chord point at the planform area centroid's depth"),
            "wetted_area": item(r(p["wetted_area"]), "m^2", src, "estimated", method=extra[name]["wetted_method"]),
            "volume_below_hull": item(r(p["volume_below_hull"], 5), "m^3", src, "estimated", method=extra[name]["volume_method"]),
        }
        out.update(stock_items)
        out.update(extra[name].get("items", {}))
        return out

    keel = foil(
        k,
        "keel",
        KEEL_AR_FACTOR,
        "2x geometric: the fin is attached to the hull with no gap, so the hull bottom acts as a reflection plane. An upper bound; heel and leeway lower it.",
        "from the canoe body on the centreline (mean over the root chord) down to the keel tip; includes the root fairing",
        f"Section {ks[-1]['section']}, the widest bulb section, directly above the {h_tip:.3f} m elliptical tip; "
        f"the fin's narrowest chord is {c_min:.2f} m at {narrow_txt}",
        {},
    )
    rudder = foil(
        ru,
        "rudder",
        RUDDER_AR_FACTOR,
        "1x geometric: transom-hung, so the root sits clear of the hull with no end plate. The free surface at the root acts partly as a reflection plane at low Froude number, which this ignores.",
        "from the root at the flotation plane (appendages.json stock_axis z) down to Section 5; nothing below Section 5",
        "Section 5; the blade ends flat there",
        {
            "stock_position": item(stock, "m", f"{h1}; {app['rudder']['stock_axis']['source']}", "estimated", method="appendages.json rudder stock_axis converted to the body frame: on the vertical leading edge at the root"),
            "stock_axis_direction": item([0.0, 0.0, 1.0], "-", h1, "estimated", method="vertical, along the leading edge, which H.1 gives as vertical; the transom rake of the pintle line is not in any source"),
        },
    )
    return {
        "frame": "Body frame: x forward, y to port, z up, metres; origin on the centreline at the midpoint of Lwl on the DWL (hull_hydro.json transform). Unlike the rest of this file, which is in the provisional frame.",
        "mesh": "appendages.obj, objects keel and rudder",
        "keel": keel,
        "rudder": rudder,
    }


def run(boat):
    app_path = boat / "appendages.json"
    app = json.loads(app_path.read_text())
    doc = json.loads((boat / "hull_hydro.json").read_text())
    frame = (doc["transform"]["stem_x_in_body"], doc["transform"]["flotation_z_in_body"])
    hulls = {n: hull_tris(boat, doc["meshes"][n]["file"]) for n in ("visual", "physics")}

    kv, kf, kinfo = keel_mesh(app["keel"], *frame, hulls["visual"][2])
    rv, rf, _ = rudder_mesh(app["rudder"], *frame)
    objs = {"keel": (np.round(kv, 5), kf), "rudder": (np.round(rv, 5), rf)}
    props = {}
    for name, (v, f) in objs.items():
        props[name], _ = props_for(name, v, f, app, frame, hulls)

    overlap = OVERLAP * kinfo["footprint_area"]
    h_fair = kinfo["z_hull"] - (val(app["keel"], "root_z") + frame[1])
    extra = {
        "keel": {
            "sweep_method": "least-squares slope of the mesh {line} over the fin, Section 1 to Section 4 (above the bulb)",
            "area_method": "chord of horizontal mesh sections integrated from the canoe body to the tip; includes the root fairing. The H.1 part (Section 1 to the tip) is planform_area_h1.",
            "wetted_method": "area of the mesh triangles whose centroid is below hull_visual.obj",
            "volume_method": "mesh volume minus its intersection with hull_physics.obj (manifold3d boolean): fin, bulb, tip and the root fairing below the canoe body",
            "items": {
                "planform_area_h1": item(r(props["keel"]["area_below"](val(app["keel"], "root_z") + frame[1])), "m^2", app["keel"]["lateral_area"]["source"], "high", method="as planform_area, Section 1 down to the tip only; compare appendages.json keel lateral_area"),
                "root_overlap": item(
                    r(overlap, 5),
                    "m^3",
                    "tools/hull/appendages.py",
                    "derived",
                    method=f"the root fairing extends {OVERLAP:.3f} m into the canoe body at every root vertex, so the overlap is {OVERLAP:.3f} m times the Section 1 area ({kinfo['footprint_area']:.4f} m^2); volume_below_hull excludes it",
                ),
                "root_overlap_depth": item(
                    OVERLAP,
                    "m",
                    "tools/hull/appendages.py",
                    "derived",
                    method="depth the root fairing reaches into the canoe body above the hull bottom at every root vertex; the simulator places the root_overlap correction in this band below the top of the keel mesh",
                ),
                "root_fairing_height": item(
                    [r(h_fair.min()), r(h_fair.max())],
                    "m",
                    "tools/hull/appendages.py",
                    "derived",
                    method="range over the Section 1 outline of the height from root_z up to hull_visual.obj",
                ),
            },
        },
        "rudder": {
            "sweep_method": "least-squares slope of the mesh {line} over the whole span",
            "area_method": "chord of horizontal mesh sections integrated from the root to Section 5",
            "wetted_method": "area of the mesh triangles below the flotation plane (the root face at z = 0 is excluded)",
            "volume_method": "mesh volume; the blade is clear of the hull, so none of it is inside hull_physics.obj",
        },
    }
    app["foils"] = foils_block(props, app, frame, extra)
    name = val(json.loads((boat / "particulars.json").read_text()), "name")
    try:
        app_rel = app_path.resolve().relative_to(ROOT)
    except ValueError:
        app_rel = app_path
    header = (
        f"Keel and rudder of the {name}, body frame (x forward, y to port, z up, m; origin at DWL midship).",
        f"Generated by tools/hull/appendages.py from {app_rel}. Objects: keel, rudder.",
    )
    write_obj(boat / "appendages.obj", objs, header)
    app_path.write_text(json.dumps(app, indent=2, ensure_ascii=False) + "\n")
    for name, (v, f) in objs.items():
        print(f"wrote {name}: {len(f)} triangles, {len(v)} vertices")


def check(boat):
    app = json.loads((boat / "appendages.json").read_text())
    doc = json.loads((boat / "hull_hydro.json").read_text())
    part = json.loads((boat / "particulars.json").read_text())
    frame = (doc["transform"]["stem_x_in_body"], doc["transform"]["flotation_z_in_body"])
    hulls = {n: hull_tris(boat, doc["meshes"][n]["file"]) for n in ("visual", "physics")}
    objs = read_objects(boat / "appendages.obj")
    foils = app["foils"]
    res = doc["result"]
    ok = True

    def flag(p):
        nonlocal ok
        ok &= bool(p)
        return "PASS" if p else "FAIL"

    def rel(name, got, want, tol, unit):
        e = (got - want) / want
        return f"  {name:<34} {got:9.5f} vs {want:9.5f} {unit:<4} {100 * e:+6.2f}%, within {100 * tol:.0f}%  {flag(abs(e) <= tol)}"

    for name in ("keel", "rudder"):
        if name not in objs:
            print(f"{name}: missing from appendages.obj  {flag(False)}")
            continue
        v, f = objs[name]
        m = trimesh.Trimesh(v, f, process=True)
        wt = m.is_watertight and m.is_winding_consistent and m.volume > 0
        print(f"{name} ({len(f)} triangles, {len(v)} vertices):")
        print(f"  watertight {m.is_watertight}, winding consistent {m.is_winding_consistent}, outward {m.volume > 0}  {flag(wt)}")
        p, z_root = props_for(name, v, f, app, frame, hulls)
        st = foils[name]
        stored = ["span", "root_chord", "tip_chord", "planform_area", "wetted_area", "volume_below_hull"]
        keys = {"planform_area": "area"}
        worst = max(abs(p[keys.get(s, s)] - val(st, s)) / abs(val(st, s)) for s in stored)
        print(f"  foils block matches the OBJ (span, chords, areas, volume), worst {worst:.1e} rel, within {TOL_STORED:.0e}  {flag(worst <= TOL_STORED)}")

        if name == "keel":
            keel = app["keel"]
            want = val(keel, "volume_from_offsets") + res["keel_fairing_volume"]
            print(rel("volume below hull vs offsets+fairing", p["volume_below_hull"], want, TOL_VOLUME, "m^3"))
            print(f"    (mesh {p['volume_mesh']:.5f} m^3, of which inside the hull {p['volume_mesh'] - p['volume_below_hull']:.5f}; stored overlap {val(st, 'root_overlap'):.5f})")
            print(rel("planform area S1-tip vs lateral_area", p["area_below"](z_root), val(keel, "lateral_area"), TOL_AREA, "m^2"))
            print(f"    (planform area below the hull, with the fairing: {p['area']:.4f} m^2)")
            tip = p["z_tip"]
            print(f"  tip z {tip:.4f} m vs -draft {-val(part, 'draft'):.4f} m, within {1000 * TOL_TIP:.0f} mm  {flag(abs(tip + val(part, 'draft')) <= TOL_TIP)}")
            root = v[v[:, 2] > z_root + 1e-4]
        else:
            rudder = app["rudder"]
            print(rel("volume vs morph.py rudder estimate", p["volume_below_hull"], res["displacement_breakdown"]["rudder"], TOL_VOLUME, "m^3"))
            print(rel("planform area vs lateral_area", p["area"], val(rudder, "lateral_area"), TOL_AREA, "m^2"))
            root = v[v[:, 2] >= v[:, 2].max() - 1e-6]
        for hname in ("visual", "physics"):
            tris = hulls[hname][2]
            inside = winding(tris, root) >= 0.5
            dist = surface_dist(tris, root)
            good = inside | (dist <= TOL_GAP)
            msg = f"  root vertices vs hull_{hname}: {int(inside.sum())}/{len(root)} inside, worst outside {dist[~inside].max() if (~inside).any() else 0.0:.4f} m"
            if name == "rudder" and "transom-hung" in val(app["rudder"], "type"):
                print(msg + f", nearest {dist.min():.4f} m; transom-hung, hangs clear of the hull  N/A")
            else:
                print(msg + f", all within {1000 * TOL_GAP:.0f} mm or inside  {flag(good.all())}")
        if name == "rudder":
            axis = np.array(val(app["rudder"], "stock_axis")) + [frame[0], 0.0, frame[1]]
            transom = hulls["visual"][0][:, 0].min()
            e = abs(p["x_le_root"] - axis[0])
            print(f"  leading edge on the stock axis: x {p['x_le_root']:.4f} vs {axis[0]:.4f} m, LE sweep {p['sweep_le']:+.2f} deg; {transom - p['x_le_root']:.4f} m aft of the transom  {flag(e <= 1e-3 and abs(p['sweep_le']) < 0.1)}")
    print("ALL PASS" if ok else "FAIL")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--boat-dir", type=Path, default=ROOT / "data/boats/j80")
    ap.add_argument("--check", action="store_true", help="only recompute and check the written OBJ")
    args = ap.parse_args(argv)
    if not args.check:
        run(args.boat_dir)
    sys.exit(0 if check(args.boat_dir) else 1)


if __name__ == "__main__":
    main()
