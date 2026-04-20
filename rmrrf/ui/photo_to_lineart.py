"""
photo_to_lineart.py — vpype `script` generator for photo → single-stroke line art.

Pipeline: image -> weighted stipple -> Lloyd's -> nearest-neighbor TSP + 2-opt ->
Catmull-Rom. Returns the smoothed continuous path in SVG pixel units so vpype can
layout / linesimplify / filter / show / export.

Run via vpype:
    vpype script rmrrf/ui/photo_to_lineart.py \
        linemerge --tolerance 0.5mm \
        linesort --two-opt \
        linesimplify --tolerance 0.1mm \
        filter --min-length 0.5mm \
        layout --fit-to-margins 0mm 125mmx125mm \
        show

Or use the recipe:
    vpype -I rmrrf/ui/blot.vpy

Params are module-level defaults below. Override any at the shell with PTL_<NAME>,
e.g. `PTL_DENSITY=0.4 PTL_GAMMA=1.8 vpype -I rmrrf/ui/blot.vpy`.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

import mask_utils

_HERE = Path(__file__).parent

# ---- tunables ----
INPUT           = str(_HERE / "tools/20260326_030033.jpg")
MASK            = ""           # optional foreground-mask PNG from remove_bg.swift
CANVAS_MM       = 125.0
PEN_WIDTH_MM    = 0.4
DENSITY         = 5.0          # total-count multiplier (pts ≈ DENSITY × area_mm² × mean_darkness)
MIN_VALUE       = 0.85         # brightness >= this -> white (no points)
MAX_VALUE       = 0.15         # brightness <= this -> black (max density)
CONTRAST        = 2.0          # linear contrast around mid-gray (1.0 = no change, >1 = more contrast)
GAMMA           = 1.4          # >1 emphasizes darks
LLOYD_ITERS     = 30
TWO_OPT_PASSES  = 0            # vpype's `linesort --two-opt` runs after; keep 0 unless you want in-script 2-opt
SMOOTH_SUBDIV   = 6
WORK_SIZE_PX    = 1024
MIN_ISLAND_FRAC = 0.01         # drop mask islands smaller than this fraction of total foreground
DRAW_OUTLINE    = 1            # trace the mask's perimeter as an extra stroke
SEED            = 0
# ------------------

PX_PER_MM = 96.0 / 25.4     # SVG user unit is 1/96"


def _env(name, default, cast):
    v = os.environ.get("PTL_" + name)
    return cast(v) if v is not None else default


def _params():
    return {
        "input":            _env("INPUT",            INPUT,            str),
        "mask":             _env("MASK",             MASK,             str),
        "canvas_mm":        _env("CANVAS_MM",        CANVAS_MM,        float),
        "pen_width_mm":     _env("PEN_WIDTH_MM",     PEN_WIDTH_MM,     float),
        "density":          _env("DENSITY",          DENSITY,          float),
        "min_value":        _env("MIN_VALUE",        MIN_VALUE,        float),
        "max_value":        _env("MAX_VALUE",        MAX_VALUE,        float),
        "contrast":         _env("CONTRAST",         CONTRAST,         float),
        "gamma":            _env("GAMMA",            GAMMA,            float),
        "lloyd_iters":      _env("LLOYD_ITERS",      LLOYD_ITERS,      int),
        "two_opt_passes":   _env("TWO_OPT_PASSES",   TWO_OPT_PASSES,   int),
        "smooth_subdiv":    _env("SMOOTH_SUBDIV",    SMOOTH_SUBDIV,    int),
        "work_size_px":     _env("WORK_SIZE_PX",     WORK_SIZE_PX,     int),
        "min_island_frac":  _env("MIN_ISLAND_FRAC",  MIN_ISLAND_FRAC,  float),
        "draw_outline":     _env("DRAW_OUTLINE",     DRAW_OUTLINE,     int),
        "seed":             _env("SEED",             SEED,             int),
    }


def load_darkness(path, size, contrast, min_value, max_value, gamma):
    """Square-stretch to (size, size), then contrast + levels + gamma → darkness in [0, 1]."""
    img = Image.open(path).convert("L").resize((size, size), Image.LANCZOS)
    brightness = np.asarray(img, dtype=np.float32) / 255.0
    # Linear contrast around mid-gray: contrast=1 → identity, 2 → 2× slope.
    if contrast != 1.0:
        brightness = np.clip((brightness - 0.5) * contrast + 0.5, 0.0, 1.0)
    span = max(min_value - max_value, 1e-6)
    dk = np.clip((min_value - brightness) / span, 0.0, 1.0)
    return dk ** gamma


def rejection_sample(dk, n, rng):
    h, w = dk.shape
    peak = float(dk.max()) or 1.0
    pts = np.empty((n, 2), dtype=np.float32)
    got = 0
    while got < n:
        batch = max(n - got, 2048)
        xs = rng.integers(0, w, batch)
        ys = rng.integers(0, h, batch)
        keep = rng.random(batch) < (dk[ys, xs] / peak)
        idx = np.where(keep)[0][: n - got]
        pts[got : got + len(idx), 0] = xs[idx]
        pts[got : got + len(idx), 1] = ys[idx]
        got += len(idx)
    return pts


def lloyd_weighted(points, dk, iters, stop_move_px=0.5):
    """Lloyd's relaxation with darkness-weighted centroids.

    Optimizations vs. the naive version:
    - Materialize coords only for non-white pixels (saves memory at high resolution).
    - Parallelize cKDTree queries across cores.
    - Early-stop once the max per-point movement drops below stop_move_px.
    """
    _, w = dk.shape
    flat = dk.ravel()
    idx = np.flatnonzero(flat > 1e-4)       # skip white pixels entirely
    xs = (idx % w).astype(np.float32)
    ys = (idx // w).astype(np.float32)
    px = np.stack([xs, ys], axis=1)
    wts = flat[idx].astype(np.float32)

    stop_sq = stop_move_px * stop_move_px
    for it in range(iters):
        _, nearest = cKDTree(points).query(px, k=1, workers=-1)
        n = len(points)
        wsum = np.bincount(nearest, weights=wts,            minlength=n)
        xsum = np.bincount(nearest, weights=wts * px[:, 0], minlength=n)
        ysum = np.bincount(nearest, weights=wts * px[:, 1], minlength=n)
        nz = wsum > 0
        new = points.copy()
        new[nz, 0] = xsum[nz] / wsum[nz]
        new[nz, 1] = ysum[nz] / wsum[nz]
        delta = new - points
        points = new
        if it >= 5 and float((delta * delta).sum(axis=1).max()) < stop_sq:
            break
    return points


def dedupe_close(points, min_dist):
    if min_dist <= 0 or len(points) < 2:
        return points
    tree = cKDTree(points)
    keep = np.ones(len(points), dtype=bool)
    for i in range(len(points)):
        if not keep[i]:
            continue
        for j in tree.query_ball_point(points[i], min_dist):
            if j != i and keep[j]:
                keep[j] = False
    return points[keep]


def tsp_nearest_neighbor(points):
    """Greedy NN tour. Pre-queries k=16 neighbors once (parallelized) and only
    falls back to live queries when a point's cached neighbors are all visited."""
    n = len(points)
    tree = cKDTree(points)
    k_cached = min(16, n)
    _, cached = tree.query(points, k=k_cached, workers=-1)  # shape (n, k)

    remaining = np.ones(n, dtype=bool)
    start = int(np.argmin(points[:, 0] + points[:, 1]))
    order = np.empty(n, dtype=np.int64)
    order[0] = start
    remaining[start] = False
    cur = start

    for i in range(1, n):
        picked = -1
        for c in cached[cur]:
            c = int(c)
            if c != cur and remaining[c]:
                picked = c
                break
        if picked < 0:
            k = 32
            while True:
                _, idxs = tree.query(points[cur], k=min(k, n))
                idxs = np.atleast_1d(idxs)
                for c in idxs:
                    c = int(c)
                    if remaining[c]:
                        picked = c
                        break
                if picked >= 0:
                    break
                k *= 2
        order[i] = picked
        remaining[picked] = False
        cur = picked
    return order


def two_opt(points, order, passes):
    if passes <= 0:
        return order
    order = order.copy()
    n = len(order)
    for _ in range(passes):
        improved = False
        for i in range(n - 2):
            a = points[order[i]]
            b = points[order[i + 1]]
            d_ab = np.linalg.norm(a - b)
            js = np.arange(i + 2, n - 1)
            if js.size == 0:
                continue
            c = points[order[js]]
            d = points[order[js + 1]]
            delta = (
                np.linalg.norm(a - c, axis=1)
                + np.linalg.norm(b - d, axis=1)
                - d_ab
                - np.linalg.norm(c - d, axis=1)
            )
            k = int(np.argmin(delta))
            if delta[k] < -1e-9:
                j = int(js[k])
                order[i + 1 : j + 1] = order[i + 1 : j + 1][::-1]
                improved = True
        if not improved:
            break
    return order


def catmull_rom(pts, subdiv):
    if len(pts) < 4 or subdiv <= 1:
        return pts.copy()
    p = np.vstack([pts[0], pts, pts[-1]])
    ts = np.linspace(0.0, 1.0, subdiv, endpoint=False)
    t2, t3 = ts * ts, ts * ts * ts
    a = (-0.5 * t3 + t2 - 0.5 * ts)[:, None]
    b = (1.5 * t3 - 2.5 * t2 + 1.0)[:, None]
    c = (-1.5 * t3 + 2.0 * t2 + 0.5 * ts)[:, None]
    d = (0.5 * t3 - 0.5 * t2)[:, None]
    segs = []
    for i in range(1, len(p) - 2):
        segs.append(a * p[i - 1] + b * p[i] + c * p[i + 1] + d * p[i + 2])
    segs.append(pts[-1:])
    return np.vstack(segs)


def _as_complex(pts_xy: np.ndarray, scale: float) -> np.ndarray:
    """Nx2 float (image-pixel coords) → complex128 in SVG user units."""
    svg = pts_xy * scale
    return svg[:, 0].astype(np.float64) + 1j * svg[:, 1].astype(np.float64)


def generate():
    p = _params()
    dk = load_darkness(
        p["input"], p["work_size_px"],
        p["contrast"], p["min_value"], p["max_value"], p["gamma"],
    )
    h, w = dk.shape

    # Optional foreground mask: everything the Vision pipeline tagged as
    # background is zeroed out of the darkness map too, so stipple points never
    # get sampled outside the subject.
    mask = mask_utils.load_mask(p["mask"], p["work_size_px"])
    if mask is not None:
        dk = dk * mask

    n_points = int(p["density"] * (p["canvas_mm"] ** 2) * float(dk.mean()))
    n_points = max(200, min(n_points, 50000))

    rng = np.random.default_rng(p["seed"])
    pts = rejection_sample(dk, n_points, rng)
    pts = lloyd_weighted(pts, dk, p["lloyd_iters"])

    # Minimum spacing = pen width, in image-pixel units.
    min_dist_image_px = p["pen_width_mm"] * w / p["canvas_mm"]
    pts = dedupe_close(pts, min_dist=min_dist_image_px)

    order = tsp_nearest_neighbor(pts)
    order = two_opt(pts, order, p["two_opt_passes"])
    smoothed = catmull_rom(pts[order], p["smooth_subdiv"])

    # Catmull-Rom can overshoot its control points at sharp turns; clamp so
    # nothing escapes the canvas.
    np.clip(smoothed[:, 0], 0.0, w - 1.0, out=smoothed[:, 0])
    np.clip(smoothed[:, 1], 0.0, h - 1.0, out=smoothed[:, 1])

    # Convert image-pixel space into SVG user units (1/96 inch) sized to CANVAS_MM.
    scale = p["canvas_mm"] * PX_PER_MM / w
    # Safety-net: break any segment longer than 5× the pen width. Catches TSP
    # nearest-neighbor "long edges" that would otherwise be dragged as an ugly
    # straight ink line across the foreground.
    max_jump_px = max(3.0, p["pen_width_mm"] * 5.0 * w / p["canvas_mm"])

    out_polylines: list[np.ndarray] = []

    if mask is not None:
        # Break the smoothed TSP path whenever it leaves the foreground so the
        # plotter lifts the pen across the background instead of dragging ink.
        sub_paths = mask_utils.split_path_on_mask(smoothed, mask, step_px=1.0)
    else:
        sub_paths = [smoothed]

    for sub in sub_paths:
        for piece in mask_utils.split_long_segments(sub, max_jump_px):
            if len(piece) >= 2:
                out_polylines.append(_as_complex(piece, scale))

    # vpype's LineCollection stores points as complex128 (real=x, imag=y).
    return out_polylines
