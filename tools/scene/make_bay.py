#!/usr/bin/env python3
"""Generate the bay heightmap (data/scenes/bay/heightmap.png) and its meta.json.

The bay is a hand-shaped base (signed-distance coastline: a northern beach shore, two headlands
framing a southward harbour mouth, open sea to the south) perturbed by seeded value-noise fBm.
The PNG is written by hand (one IDAT, filter 0 on every row, no ancillary chunks) so the same seed
gives a byte-identical file and the 16-bit samples need no PNG library to read back.

    make_bay.py [--seed N] [--out DIR]   write heightmap.png and meta.json
    make_bay.py --check [--out DIR]      regenerate from meta.json's seed, require the files on
                                         disk to match byte for byte, and run the sanity checks
"""

import argparse
import hashlib
import json
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

GENERATOR_VERSION = 1
DEFAULT_SEED = 7

SIZE_PX = 1024
PIXEL_M = 4.0
ORIGIN_X_M = -2048.0
ORIGIN_Y_M = 2048.0

HEIGHT_SCALE_M = 0.0025
HEIGHT_OFFSET_M = -60.0

START_X_M = 0.0
START_Y_M = -1600.0
START_HEADING_DEG = 0.0
INNER_BAY_X_M = 0.0
INNER_BAY_Y_M = -200.0

WEST_TIP = (-650.0, -850.0)
WEST_TIP_RADIUS_M = 280.0
EAST_TIP = (650.0, -800.0)
EAST_TIP_RADIUS_M = 240.0
MOUTH_Y_M = -825.0
BEACH_CENTRE = (0.0, 1150.0)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "data" / "scenes" / "bay"


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def smin(a, b, k):
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b + (a - b) * h - k * h * (1.0 - h)


def capsule(x, y, a, b, r):
    pax, pay = x - a[0], y - a[1]
    bax, bay = b[0] - a[0], b[1] - a[1]
    h = np.clip((pax * bax + pay * bay) / (bax * bax + bay * bay), 0.0, 1.0)
    return np.hypot(pax - bax * h, pay - bay * h) - r


def value_noise(rng, n, cell_px):
    g = n // cell_px + 2
    lattice = rng.uniform(-1.0, 1.0, size=(g, g))
    u = np.arange(n) / cell_px
    i0 = np.floor(u).astype(np.int64)
    t = u - i0
    s = t * t * (3.0 - 2.0 * t)
    rows = lattice[i0] * (1.0 - s)[:, None] + lattice[i0 + 1] * s[:, None]
    return rows[:, i0] * (1.0 - s)[None, :] + rows[:, i0 + 1] * s[None, :]


def fbm(rng, n, base_cell_px, octaves):
    total = np.zeros((n, n))
    amp = 1.0
    cell = base_cell_px
    for _ in range(octaves):
        total += amp * value_noise(rng, n, max(cell, 1))
        amp *= 0.5
        cell //= 2
    return total / np.max(np.abs(total))


def generate_heights(seed):
    """Heights in metres, shape (rows, cols), row 0 the northern edge."""
    rng = np.random.default_rng(seed)
    coast_noise = fbm(rng, SIZE_PX, 100, 4)
    land_noise = fbm(rng, SIZE_PX, 80, 5)
    sea_noise = fbm(rng, SIZE_PX, 50, 4)

    idx = np.arange(SIZE_PX, dtype=np.float64)
    x = (ORIGIN_X_M + idx * PIXEL_M)[None, :].repeat(SIZE_PX, axis=0)
    y = (ORIGIN_Y_M - idx * PIXEL_M)[:, None].repeat(SIZE_PX, axis=1)

    # Signed distance to the coast, negative on land.
    north = (BEACH_CENTRE[1] + 0.00012 * x * x) - y
    west_side = np.maximum(x + 1550.0, -400.0 - y)
    east_side = np.maximum(1600.0 - x, -250.0 - y)
    west_head = capsule(x, y, (-1700.0, -100.0), WEST_TIP, WEST_TIP_RADIUS_M)
    east_head = capsule(x, y, (1750.0, 0.0), EAST_TIP, EAST_TIP_RADIUS_M)
    sdf = smin(north, west_side, 150.0)
    sdf = smin(sdf, east_side, 150.0)
    sdf = smin(sdf, west_head, 120.0)
    sdf = smin(sdf, east_head, 120.0)
    sdf = sdf + 60.0 * coast_noise
    inland = -sdf

    beach = np.exp(-((x - BEACH_CENTRE[0]) / 800.0) ** 2) * smoothstep(300.0, 900.0, y)

    hill = 0.75 + 0.35 * land_noise
    rock = (2.0 * (1.0 - np.exp(-inland / 15.0)) + 65.0 * (1.0 - np.exp(-inland / 320.0))) * hill
    sand = 2.5 * (1.0 - np.exp(-inland / 100.0)) + 45.0 * smoothstep(150.0, 1000.0, inland) * hill
    land = rock + (sand - rock) * beach

    # The shelf scale sets how fast the seabed drops off the shore: steep under the rocky
    # headlands, long and shoaling off the beach.
    shelf_m = 80.0 + (320.0 - 80.0) * beach
    basin_depth = 11.0 + (34.0 - 11.0) * smoothstep(-500.0, -1900.0, y)
    basin_depth = basin_depth * (1.0 + 0.12 * sea_noise)
    offshore = np.maximum(-inland, 0.0)
    seabed = -basin_depth * (1.0 - np.exp(-offshore / shelf_m))

    return np.where(inland > 0.0, land, seabed)


def encode(heights):
    values = np.rint((heights - HEIGHT_OFFSET_M) / HEIGHT_SCALE_M)
    return np.clip(values, 0, 65535).astype(np.uint16)


def decode(values):
    return values.astype(np.float64) * HEIGHT_SCALE_M + HEIGHT_OFFSET_M


def png_chunk(kind, payload):
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def write_png(values):
    rows, cols = values.shape
    big_endian = values.astype(">u2")
    raw = b"".join(b"\x00" + big_endian[r].tobytes() for r in range(rows))
    ihdr = struct.pack(">IIBBBBB", cols, rows, 16, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", ihdr)
        + png_chunk(b"IDAT", zlib.compress(raw, 9))
        + png_chunk(b"IEND", b"")
    )


def world_to_pixel(x_m, y_m):
    return (x_m - ORIGIN_X_M) / PIXEL_M, (ORIGIN_Y_M - y_m) / PIXEL_M


def sample(heights, x_m, y_m):
    """Bilinear sample as meta.json documents it for the core."""
    col, row = world_to_pixel(x_m, y_m)
    col = min(max(col, 0.0), SIZE_PX - 1.0)
    row = min(max(row, 0.0), SIZE_PX - 1.0)
    c0 = min(int(np.floor(col)), SIZE_PX - 2)
    r0 = min(int(np.floor(row)), SIZE_PX - 2)
    tc, tr = col - c0, row - r0
    top = heights[r0, c0] * (1 - tc) + heights[r0, c0 + 1] * tc
    bottom = heights[r0 + 1, c0] * (1 - tc) + heights[r0 + 1, c0 + 1] * tc
    return float(top * (1 - tr) + bottom * tr)


def mouth_span(heights, min_depth_m):
    """West and east x of the water at least min_depth_m deep across the mouth, around x = 0."""
    xs = np.arange(-1200.0, 1200.0 + 1e-9, PIXEL_M)
    deep = [sample(heights, x, MOUTH_Y_M) <= -min_depth_m for x in xs]
    centre = len(xs) // 2
    if not deep[centre]:
        return None
    lo = hi = centre
    while lo > 0 and deep[lo - 1]:
        lo -= 1
    while hi < len(xs) - 1 and deep[hi + 1]:
        hi += 1
    return float(xs[lo]), float(xs[hi])


def stats(heights):
    water = heights < 0.0
    depth = -heights[water]
    return {
        "water_fraction": float(np.mean(water)),
        "water_at_least_5m_fraction": float(np.mean(depth >= 5.0)),
        "max_depth_m": float(depth.max()),
        "max_land_m": float(heights.max()),
        "land_above_10m_fraction": float(np.mean(heights > 10.0)),
    }


def sanity_failures(heights):
    failures = []
    start_z = sample(heights, START_X_M, START_Y_M)
    if start_z > -5.0:
        failures.append(f"start is {-start_z:.2f} m deep, want >= 5")
    steps = 2000
    shallowest = min(
        -sample(
            heights,
            START_X_M + (INNER_BAY_X_M - START_X_M) * i / steps,
            START_Y_M + (INNER_BAY_Y_M - START_Y_M) * i / steps,
        )
        for i in range(steps + 1)
    )
    if shallowest < 3.0:
        failures.append(f"start-to-bay line through the mouth is {shallowest:.2f} m deep at its shallowest, want >= 3")
    if heights.max() <= 10.0:
        failures.append(f"highest land is {heights.max():.2f} m, want > 10")
    span = mouth_span(heights, 3.0)
    if span is None or span[1] - span[0] < 300.0:
        failures.append(f"harbour mouth 3 m channel is {span}, want >= 300 m wide")
    return failures, shallowest


def build_meta(seed, png_bytes, heights):
    span = mouth_span(heights, 3.0)
    return {
        "generator": "tools/scene/make_bay.py",
        "generator_version": GENERATOR_VERSION,
        "seed": seed,
        "heightmap": {
            "file": "heightmap.png",
            "sha256": hashlib.sha256(png_bytes).hexdigest(),
            "width_px": SIZE_PX,
            "height_px": SIZE_PX,
            "format": "PNG greyscale, 16 bit big-endian samples, not interlaced, filter type 0 on every row",
        },
        "pixel_size_m": PIXEL_M,
        "frame": "core world frame: x east, y north, z up, sea level z = 0",
        "origin": {
            "x_m": ORIGIN_X_M,
            "y_m": ORIGIN_Y_M,
            "note": "world x, y of the centre of pixel (col 0, row 0), the image's top-left sample",
        },
        "axes": {
            "col": "east: x_m = origin.x_m + col * pixel_size_m",
            "row": "south: y_m = origin.y_m - row * pixel_size_m (row 0 is the northern edge)",
        },
        "height_encoding": {
            "scale_m": HEIGHT_SCALE_M,
            "offset_m": HEIGHT_OFFSET_M,
            "formula": "z_m = value * scale_m + offset_m",
            "sea_level_m": 0.0,
        },
        "sampling": {
            "method": "bilinear between pixel centres",
            "col": "(x_m - origin.x_m) / pixel_size_m",
            "row": "(origin.y_m - y_m) / pixel_size_m",
            "integer_coordinates": "an integer (col, row) is a pixel centre",
            "outside": "clamp col and row to [0, size - 1]",
            "seabed": "z_m < 0 is the seabed and the water depth is -z_m",
        },
        "start": {
            "x_m": START_X_M,
            "y_m": START_Y_M,
            "heading_deg": START_HEADING_DEG,
            "heading_convention": "compass: clockwise from north, 0 = north, 90 = east",
            "note": "open water south of the harbour mouth, heading in through it",
        },
        "layout": {
            "open_sea": "south, deepening to about 34 m at the southern edge",
            "west_headland_tip_xy_m": list(WEST_TIP),
            "east_headland_tip_xy_m": list(EAST_TIP),
            "harbour_mouth_y_m": MOUTH_Y_M,
            "harbour_mouth_3m_channel_x_m": list(span) if span else None,
            "inner_bay": "x within about +-1500 m, y from about -400 to the beach, 8 to 12 m deep",
            "beach_centre_xy_m": list(BEACH_CENTRE),
            "beach": "north shore, x within about +-800 m, shoaling over about 200 to 400 m",
        },
        "stats": {k: round(v, 3) for k, v in stats(heights).items()},
    }


def generate(seed):
    values = encode(generate_heights(seed))
    png_bytes = write_png(values)
    heights = decode(values)
    meta_text = json.dumps(build_meta(seed, png_bytes, heights), indent=2) + "\n"
    return png_bytes, meta_text, heights


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=None, help=f"noise seed (default {DEFAULT_SEED})")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output directory")
    parser.add_argument("--check", action="store_true", help="verify the files on disk instead of writing")
    args = parser.parse_args()

    png_path = args.out / "heightmap.png"
    meta_path = args.out / "meta.json"

    if args.check:
        seed = args.seed if args.seed is not None else json.loads(meta_path.read_text())["seed"]
        png_bytes, meta_text, heights = generate(seed)
        again, _, _ = generate(seed)
        failures, shallowest = sanity_failures(heights)
        if png_bytes != again:
            failures.append("two generations with the same seed differ")
        if png_path.read_bytes() != png_bytes:
            failures.append(f"{png_path} does not match a fresh generation with seed {seed}")
        if meta_path.read_text() != meta_text:
            failures.append(f"{meta_path} does not match a fresh generation with seed {seed}")
        print(f"sha256 {hashlib.sha256(png_bytes).hexdigest()}")
        print(f"start depth {-sample(heights, START_X_M, START_Y_M):.2f} m, line minimum {shallowest:.2f} m")
        print(f"mouth 3 m channel x {mouth_span(heights, 3.0)}")
        print(json.dumps(stats(heights)))
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1 if failures else 0

    seed = args.seed if args.seed is not None else DEFAULT_SEED
    png_bytes, meta_text, heights = generate(seed)
    failures, _ = sanity_failures(heights)
    for failure in failures:
        print(f"warning: {failure}", file=sys.stderr)
    args.out.mkdir(parents=True, exist_ok=True)
    png_path.write_bytes(png_bytes)
    meta_path.write_text(meta_text)
    print(f"wrote {png_path} ({len(png_bytes)} bytes, sha256 {hashlib.sha256(png_bytes).hexdigest()})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
