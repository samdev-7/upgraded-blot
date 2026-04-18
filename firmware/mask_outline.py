"""
mask_outline.py — vpype `script` generator for the foreground mask outline.

Reads the mask PNG produced by `remove_bg.swift` (via env var PTL_MASK),
labels connected components, drops islands smaller than PTL_MIN_ISLAND_FRAC,
and emits the outline of each surviving island as a polyline. Runs as a
separate `script` command from `photo_to_lineart.py` so it can be routed to
its own vpype layer (and shown in a different color in the viewer).

Coordinate system matches the stipple script: canvas_mm → SVG pixel units.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

import mask_utils

_HERE = Path(__file__).parent

# ---- tunables (mirror photo_to_lineart.py for convenience) ----
MASK            = ""
CANVAS_MM       = 125.0
WORK_SIZE_PX    = 1024
MIN_ISLAND_FRAC = 0.01
# ----------------------------------------------------------------

PX_PER_MM = 96.0 / 25.4


def _env(name, default, cast):
    v = os.environ.get("PTL_" + name)
    return cast(v) if v is not None else default


def generate():
    mask_path = _env("MASK", MASK, str)
    canvas_mm = _env("CANVAS_MM", CANVAS_MM, float)
    work_size = _env("WORK_SIZE_PX", WORK_SIZE_PX, int)
    min_island_frac = _env("MIN_ISLAND_FRAC", MIN_ISLAND_FRAC, float)

    mask = mask_utils.load_mask(mask_path, work_size)
    if mask is None:
        return []

    scale = canvas_mm * PX_PER_MM / mask.shape[1]
    out = []
    for island in mask_utils.find_islands(mask, min_island_frac):
        for ring in mask_utils.island_outlines(island):
            svg = ring * scale
            out.append(svg[:, 0].astype(np.float64) + 1j * svg[:, 1].astype(np.float64))
    return out
