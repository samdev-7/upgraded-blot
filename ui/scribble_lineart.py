"""
scribble_lineart.py — vpype `script` generator for photo → greedy error-guided
continuous scribble (port of iapafoto/PicoPrint's ImageToDrawEvaluateLine).

Algorithm per step:
- Keep two buffers: `src` (target darkness) and `drawn` (ink laid down so far).
- Probe M random angles from the current pen point. For each, Bresenham-sample
  the candidate segment and score it as Σ |drawn+pen − src| − |drawn − src|.
  Negative = this stroke reduces the error.
- Take the best stroke, ink it into `drawn`, move the pen, repeat.
- Every ~1% of the max step count, scan the image with coarse block-means;
  stop once the worst cell's mean error is ≤ 0 (drawn everywhere enough).

Output is one continuous polyline smoothed into chained quadratic Béziers via
the midpoint trick (each original vertex becomes a control point between the
midpoints of its two adjacent segments — rounds corners without pen lifts).

Env-var overrides mirror photo_to_lineart.py: PTL_<NAME>, e.g. PTL_PEN_BLACKNESS=48.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from PIL import Image

_HERE = Path(__file__).parent

# ---- tunables ----
INPUT            = str(_HERE / "tools/20260326_030033.jpg")
CANVAS_MM        = 125.0
PEN_WIDTH_MM     = 0.4
MIN_VALUE        = 0.85
MAX_VALUE        = 0.15
CONTRAST         = 2.0
GAMMA            = 1.4
PEN_BLACKNESS    = 64.0     # ink deposited per stroke, on a 0-255 scale
MAX_DARKNESS     = 1.0      # clamp target darkness: 1.0 = no clamp, lower = lighter output
MAX_STEPS        = 30000    # hard cap on stroke count
ANGLE_SAMPLES    = 200      # random angles probed per step
BASE_SEGMENT_PX  = 3.0      # minimum stroke length, in work-size pixels
MAX_SEGMENT_PX   = 48.0     # extra stroke length, scaled by local brightness
SMOOTH_SUBDIV    = 6        # midpoint-quadratic subdivisions per vertex
WORK_SIZE_PX     = 1024
SEED             = 0
# ------------------

PX_PER_MM = 96.0 / 25.4


def _env(name, default, cast):
    v = os.environ.get("PTL_" + name)
    return cast(v) if v is not None else default


def _params():
    return {
        "input":            _env("INPUT",            INPUT,            str),
        "canvas_mm":        _env("CANVAS_MM",        CANVAS_MM,        float),
        "pen_width_mm":     _env("PEN_WIDTH_MM",     PEN_WIDTH_MM,     float),
        "min_value":        _env("MIN_VALUE",        MIN_VALUE,        float),
        "max_value":        _env("MAX_VALUE",        MAX_VALUE,        float),
        "contrast":         _env("CONTRAST",         CONTRAST,         float),
        "gamma":            _env("GAMMA",            GAMMA,            float),
        "pen_blackness":    _env("PEN_BLACKNESS",    PEN_BLACKNESS,    float),
        "max_darkness":     _env("MAX_DARKNESS",     MAX_DARKNESS,     float),
        "max_steps":        _env("MAX_STEPS",        MAX_STEPS,        int),
        "angle_samples":    _env("ANGLE_SAMPLES",    ANGLE_SAMPLES,    int),
        "base_segment_px":  _env("BASE_SEGMENT_PX",  BASE_SEGMENT_PX,  float),
        "max_segment_px":   _env("MAX_SEGMENT_PX",   MAX_SEGMENT_PX,   float),
        "smooth_subdiv":    _env("SMOOTH_SUBDIV",    SMOOTH_SUBDIV,    int),
        "work_size_px":     _env("WORK_SIZE_PX",     WORK_SIZE_PX,     int),
        "seed":             _env("SEED",             SEED,             int),
    }


def load_darkness(path, size, contrast, min_value, max_value, gamma):
    """Square-stretch to (size, size), contrast + levels + gamma → darkness in [0, 1]."""
    img = Image.open(path).convert("L").resize((size, size), Image.LANCZOS)
    brightness = np.asarray(img, dtype=np.float32) / 255.0
    if contrast != 1.0:
        brightness = np.clip((brightness - 0.5) * contrast + 0.5, 0.0, 1.0)
    span = max(min_value - max_value, 1e-6)
    dk = np.clip((min_value - brightness) / span, 0.0, 1.0)
    return dk ** gamma


def worst_zone_mean(src, drawn, cell):
    """Return (max block-mean of (src - drawn), (cx, cy)) across an axis-aligned grid."""
    h, w = src.shape
    hh, ww = h // cell, w // cell
    if hh == 0 or ww == 0:
        diff = src - drawn
        return float(diff.max()), (w // 2, h // 2)
    diff = src[: hh * cell, : ww * cell] - drawn[: hh * cell, : ww * cell]
    blocks = diff.reshape(hh, cell, ww, cell).mean(axis=(1, 3))
    idx = int(np.argmax(blocks))
    by, bx = divmod(idx, ww)
    return float(blocks[by, bx]), (bx * cell + cell // 2, by * cell + cell // 2)


def _line_pixels(x0, y0, x1, y1, w, h):
    """Bresenham-coverage integer pixels along the segment (unique, endpoint-inclusive)."""
    dx = x1 - x0
    dy = y1 - y0
    steps = max(int(round(abs(dx))), int(round(abs(dy))), 1)
    ts = np.linspace(0.0, 1.0, steps + 1)
    xs = np.clip(np.round(x0 + ts * dx).astype(np.int32), 0, w - 1)
    ys = np.clip(np.round(y0 + ts * dy).astype(np.int32), 0, h - 1)
    return xs, ys


def greedy_scribble(src, p, rng):
    """Return the raw scribble polyline in work-pixel coordinates, shape (N, 2)."""
    h, w = src.shape
    max_dark = float(p["max_darkness"])
    # Clip the target darkness: the algorithm never tries to draw any pixel
    # darker than `max_dark`, regardless of how black it is in the source.
    target = np.minimum(src, max_dark)
    drawn = np.zeros_like(target)
    pen = float(p["pen_blackness"]) / 255.0
    base = float(p["base_segment_px"])
    add = float(p["max_segment_px"])
    M = int(p["angle_samples"])
    max_steps = int(p["max_steps"])

    # Seed at the darkest coarse zone (gives a more "artful" starting stroke).
    cell = max(1, int(w * 0.1))
    _, (sx, sy) = worst_zone_mean(target, drawn, cell)
    x, y = float(sx), float(sy)

    paths: list[list[tuple[float, float]]] = [[(x, y)]]
    check_every = max(1, max_steps // 100)

    for step in range(1, max_steps + 1):
        if step % check_every == 0:
            err, _ = worst_zone_mean(target, drawn, cell)
            if err <= 0.0:
                break

        xi = int(x) if 0 <= x < w else int(np.clip(x, 0, w - 1))
        yi = int(y) if 0 <= y < h else int(np.clip(y, 0, h - 1))

        # Step length is driven by *remaining* darkness at the pen: over-drawn
        # zones should jump out with long strokes instead of bouncing locally.
        remaining = max(0.0, float(target[yi, xi]) - float(drawn[yi, xi]))
        kinit = base + add * (1.0 - remaining)

        angles = rng.uniform(0.0, 2.0 * np.pi, size=M)
        ks = np.maximum(base, 0.5 * kinit + 0.5 * np.abs(rng.standard_normal(M)) * kinit)

        x1s = np.clip(x + ks * np.cos(angles), 0.0, w - 1.0)
        y1s = np.clip(y + ks * np.sin(angles), 0.0, h - 1.0)

        seg_len = np.maximum(np.abs(x1s - x), np.abs(y1s - y)).astype(np.int32)
        np.maximum(seg_len, 1, out=seg_len)
        L = int(seg_len.max())
        ts = np.linspace(0.0, 1.0, L, dtype=np.float32)[None, :]  # (1, L)

        xs_f = x + ts * (x1s - x)[:, None].astype(np.float32)       # (M, L)
        ys_f = y + ts * (y1s - y)[:, None].astype(np.float32)
        xs_i = np.clip(xs_f.astype(np.int32), 0, w - 1)
        ys_i = np.clip(ys_f.astype(np.int32), 0, h - 1)

        target_v = target[ys_i, xs_i]
        drawn_v = drawn[ys_i, xs_i]
        delta = np.abs(drawn_v + pen - target_v) - np.abs(drawn_v - target_v)

        # Only count samples up to each candidate's actual length.
        mask = (np.arange(L)[None, :] < seg_len[:, None])
        score = np.where(mask, delta, 0.0).sum(axis=1) / seg_len

        best = int(np.argmin(score))
        best_score = float(score[best])

        # score < 0: helpful stroke — take the best.
        # score == 0: neutral (e.g. walking through a capped region) — pick a
        #   random direction so we keep moving without teleporting.
        # score > 0: every option is strictly harmful — teleport to the worst
        #   under-drawn zone and start a new subpath.
        if best_score > 0.0:
            err, (zx, zy) = worst_zone_mean(src, drawn, cell)
            if err <= 0.0:
                break
            x, y = float(zx), float(zy)
            paths.append([(x, y)])
            continue
        if best_score == 0.0:
            best = int(rng.integers(0, M))

        nx, ny = float(x1s[best]), float(y1s[best])

        # Ink the chosen stroke. The target is already clipped to max_dark, so
        # over-drawing past the cap is scored as harmful (positive delta) and
        # the algorithm naturally moves away.
        xs, ys = _line_pixels(x, y, nx, ny, w, h)
        drawn[ys, xs] += pen

        x, y = nx, ny
        paths[-1].append((x, y))

    return [np.array(p, dtype=np.float32) for p in paths if len(p) >= 2]


def midpoint_quad_smooth(pts, subdiv):
    """Chain quadratic Béziers through segment-midpoints with vertices as controls.

    Classic corner-rounding: the curve passes through midpoints and bends
    smoothly around each original vertex, so sharp direction changes become
    gentle arcs — kind to the plotter's motion system.
    """
    if len(pts) < 3 or subdiv <= 1:
        return pts.copy()
    mids = 0.5 * (pts[:-1] + pts[1:])                         # (n-1, 2)
    ts = np.linspace(0.0, 1.0, subdiv, endpoint=False, dtype=np.float32)  # (S,)
    P0 = mids[:-1]            # (n-2, 2)
    C  = pts[1:-1]            # (n-2, 2)
    P2 = mids[1:]             # (n-2, 2)
    w0 = ((1.0 - ts) ** 2)[None, :, None]                     # (1, S, 1)
    w1 = (2.0 * (1.0 - ts) * ts)[None, :, None]
    w2 = (ts ** 2)[None, :, None]
    segs = w0 * P0[:, None, :] + w1 * C[:, None, :] + w2 * P2[:, None, :]  # (n-2, S, 2)
    return np.vstack([pts[0:1], segs.reshape(-1, 2), pts[-1:]])


def generate():
    p = _params()
    dk = load_darkness(
        p["input"], p["work_size_px"],
        p["contrast"], p["min_value"], p["max_value"], p["gamma"],
    )
    rng = np.random.default_rng(p["seed"])

    paths = greedy_scribble(dk, p, rng)

    scale = p["canvas_mm"] * PX_PER_MM / dk.shape[1]
    out = []
    for pts in paths:
        smoothed = midpoint_quad_smooth(pts, p["smooth_subdiv"])
        svg = smoothed * scale
        out.append(svg[:, 0].astype(np.float64) + 1j * svg[:, 1].astype(np.float64))
    return out
