"""
pintr_lineart.py — vpype `script` generator for photo → long-stroke line art
(port of javierbyte/pintr).

Algorithm per step:
- Optionally (if not single-line) resample K random pixels and set the stroke
  start to the darkest one. Otherwise the start is the previous stroke's end.
- Sample K random endpoints. For each candidate segment, compute the mean
  "remaining darkness" along the line. Pick the darkest candidate.
- Emit the stroke, and subtract `pen_opacity` from the remaining-darkness map
  along it (drawn ink no longer needs to be drawn again).

Output:
- single_line=True  → one continuous polyline (chained strokes).
- single_line=False → one 2-point polyline per stroke (vpype's linemerge will
  re-chain consecutive strokes whose endpoints coincide).

Env-var overrides are PTL_<NAME>, same prefix as the other generators.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from PIL import Image

_HERE = Path(__file__).parent

# ---- tunables ----
INPUT          = str(_HERE / "tools/20260326_030033.jpg")
CANVAS_MM      = 125.0
PEN_WIDTH_MM   = 0.4
MIN_VALUE      = 0.85
MAX_VALUE      = 0.15
CONTRAST       = 2.0
GAMMA          = 1.4
TOTAL_LINES    = 4000      # number of strokes to emit
CANDIDATES     = 20        # random endpoints probed per step (~pintr "definition")
PEN_OPACITY    = 0.33      # how much each stroke "consumes" remaining darkness
SINGLE_LINE    = 1         # 1 = continuous (no pen lifts), 0 = pintr-classic disjoint
WORK_SIZE_PX   = 1024
SEED           = 0
# ------------------

PX_PER_MM = 96.0 / 25.4


def _env(name, default, cast):
    v = os.environ.get("PTL_" + name)
    return cast(v) if v is not None else default


def _params():
    return {
        "input":         _env("INPUT",         INPUT,         str),
        "canvas_mm":     _env("CANVAS_MM",     CANVAS_MM,     float),
        "pen_width_mm":  _env("PEN_WIDTH_MM",  PEN_WIDTH_MM,  float),
        "min_value":     _env("MIN_VALUE",     MIN_VALUE,     float),
        "max_value":     _env("MAX_VALUE",     MAX_VALUE,     float),
        "contrast":      _env("CONTRAST",      CONTRAST,      float),
        "gamma":         _env("GAMMA",         GAMMA,         float),
        "total_lines":   _env("TOTAL_LINES",   TOTAL_LINES,   int),
        "candidates":    _env("CANDIDATES",    CANDIDATES,    int),
        "pen_opacity":   _env("PEN_OPACITY",   PEN_OPACITY,   float),
        "single_line":   _env("SINGLE_LINE",   SINGLE_LINE,   int),
        "work_size_px":  _env("WORK_SIZE_PX",  WORK_SIZE_PX,  int),
        "seed":          _env("SEED",          SEED,          int),
    }


def load_darkness(path, size, contrast, min_value, max_value, gamma):
    img = Image.open(path).convert("L").resize((size, size), Image.LANCZOS)
    brightness = np.asarray(img, dtype=np.float32) / 255.0
    if contrast != 1.0:
        brightness = np.clip((brightness - 0.5) * contrast + 0.5, 0.0, 1.0)
    span = max(min_value - max_value, 1e-6)
    dk = np.clip((min_value - brightness) / span, 0.0, 1.0)
    return dk ** gamma


def _line_pixels(x0, y0, x1, y1, w, h):
    dx = x1 - x0
    dy = y1 - y0
    steps = max(int(round(abs(dx))), int(round(abs(dy))), 1)
    ts = np.linspace(0.0, 1.0, steps + 1)
    xs = np.clip(np.round(x0 + ts * dx).astype(np.int32), 0, w - 1)
    ys = np.clip(np.round(y0 + ts * dy).astype(np.int32), 0, h - 1)
    return xs, ys


def _best_line_from(remaining, fx, fy, candidates, rng):
    """Among K random endpoints, return the one whose mean darkness is highest."""
    h, w = remaining.shape
    xs_end = rng.integers(0, w, candidates).astype(np.float32)
    ys_end = rng.integers(0, h, candidates).astype(np.float32)

    lens = np.maximum(np.abs(xs_end - fx), np.abs(ys_end - fy)).astype(np.int32)
    np.maximum(lens, 1, out=lens)
    L = int(lens.max())
    ts = np.linspace(0.0, 1.0, L, dtype=np.float32)[None, :]          # (1, L)
    xs_f = fx + ts * (xs_end - fx)[:, None]                            # (K, L)
    ys_f = fy + ts * (ys_end - fy)[:, None]
    xs_i = np.clip(xs_f.astype(np.int32), 0, w - 1)
    ys_i = np.clip(ys_f.astype(np.int32), 0, h - 1)

    vals = remaining[ys_i, xs_i]                                       # (K, L)
    mask = (np.arange(L)[None, :] < lens[:, None])
    scores = np.where(mask, vals, 0.0).sum(axis=1) / lens
    best = int(np.argmax(scores))
    return float(xs_end[best]), float(ys_end[best])


def pintr_draw(src, p, rng):
    h, w = src.shape
    remaining = src.copy()
    K = int(p["candidates"])
    N = int(p["total_lines"])
    pen = float(p["pen_opacity"])
    single = bool(p["single_line"])

    cx, cy = w / 2.0, h / 2.0
    pts = [(cx, cy)]
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []

    for _ in range(N):
        fx, fy = cx, cy
        if not single:
            # pintr-style: among K random points, keep the darkest as stroke start.
            xs = rng.integers(0, w, K)
            ys = rng.integers(0, h, K)
            vals = remaining[ys, xs]
            idx = int(np.argmax(vals))
            cur_v = remaining[int(np.clip(cy, 0, h - 1)), int(np.clip(cx, 0, w - 1))]
            if vals[idx] > cur_v:
                fx, fy = float(xs[idx]), float(ys[idx])

        tx, ty = _best_line_from(remaining, fx, fy, K, rng)

        # Consume remaining darkness along the drawn stroke.
        xsL, ysL = _line_pixels(fx, fy, tx, ty, w, h)
        remaining[ysL, xsL] = np.maximum(remaining[ysL, xsL] - pen, 0.0)

        if single:
            pts.append((tx, ty))
        else:
            segments.append(((fx, fy), (tx, ty)))
        cx, cy = tx, ty

    if single:
        return [np.array(pts, dtype=np.float32)]
    return [np.array([a, b], dtype=np.float32) for a, b in segments]


def generate():
    p = _params()
    dk = load_darkness(
        p["input"], p["work_size_px"],
        p["contrast"], p["min_value"], p["max_value"], p["gamma"],
    )
    rng = np.random.default_rng(p["seed"])
    polylines = pintr_draw(dk, p, rng)

    scale = p["canvas_mm"] * PX_PER_MM / dk.shape[1]
    out = []
    for poly in polylines:
        svg = poly * scale
        out.append(svg[:, 0].astype(np.float64) + 1j * svg[:, 1].astype(np.float64))
    return out
