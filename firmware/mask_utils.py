"""
mask_utils.py — foreground-mask helpers for the stipple pipeline.

The mask is produced by `remove_bg.swift` via Apple's Vision framework
(`VNGenerateForegroundInstanceMaskRequest`) and saved alongside the composited
foreground as `<output>.mask.png` (255 = foreground, 0 = background).

These helpers:
  - load the mask at the stipple's working resolution
  - label connected components (islands) and drop tiny ones
  - extract an ordered outline polyline per island via marching squares
  - split a polyline wherever it leaves the mask, so the plotter lifts the pen
    across the background instead of dragging ink through the white space
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label as nd_label
from skimage import measure


def load_mask(path: str | Path, size: int) -> np.ndarray | None:
    """Load a grayscale mask at (size, size). True = foreground. None if missing."""
    path = str(path)
    if not path or not Path(path).exists():
        return None
    img = Image.open(path).convert("L").resize((size, size), Image.LANCZOS)
    return np.asarray(img, dtype=np.uint8) > 128


def find_islands(mask: np.ndarray, min_area_frac: float = 0.01) -> list[np.ndarray]:
    """Return the list of connected foreground components whose area >= the
    given fraction of the total foreground area. Each component is a bool mask
    the same shape as `mask`."""
    labeled, n = nd_label(mask)
    if n == 0:
        return []
    sizes = np.bincount(labeled.ravel())           # sizes[0] = background count
    total_fg = int(mask.sum())
    threshold = max(1, int(total_fg * min_area_frac))
    return [
        (labeled == i)
        for i in range(1, n + 1)
        if sizes[i] >= threshold
    ]


def island_outlines(island_mask: np.ndarray) -> list[np.ndarray]:
    """Return a list of Nx2 (x, y) polylines tracing the outline of `island_mask`.

    Uses scikit-image's marching-squares implementation, which may return multiple
    closed curves (outer perimeter + any interior holes). Coordinates are in
    image-pixel space, matching the mask's shape.
    """
    if not island_mask.any():
        return []
    # find_contours returns (row, col) = (y, x) at 0.5 isolevel. Pad by 1 so the
    # contour closes even when the island touches the image border.
    padded = np.pad(island_mask.astype(np.float32), 1, mode="constant")
    contours = measure.find_contours(padded, 0.5)
    out: list[np.ndarray] = []
    for c in contours:
        if len(c) < 4:
            continue
        # Undo the padding offset and swap to (x, y).
        xy = np.column_stack([c[:, 1] - 1.0, c[:, 0] - 1.0])
        # Clip to the original image extent.
        xy[:, 0] = np.clip(xy[:, 0], 0, island_mask.shape[1] - 1)
        xy[:, 1] = np.clip(xy[:, 1], 0, island_mask.shape[0] - 1)
        out.append(xy)
    return out


def split_long_segments(pts: np.ndarray, max_len_px: float) -> list[np.ndarray]:
    """Break a polyline wherever a consecutive-vertex jump exceeds `max_len_px`.

    Catches TSP "nearest-neighbor" failures that connect distant clusters with
    a single long edge — those show up as an ugly straight pen-down line across
    the canvas. Each returned sub-polyline has only short jumps between
    consecutive points; the plotter lifts the pen between them.
    """
    if len(pts) < 2:
        return [pts] if len(pts) >= 2 else []
    threshold_sq = float(max_len_px) ** 2
    diffs = np.diff(pts, axis=0)
    jumps = np.einsum("ij,ij->i", diffs, diffs) > threshold_sq
    if not jumps.any():
        return [pts]
    # Split indices fall between vertex i and i+1 whenever jumps[i] is True.
    split_after = np.flatnonzero(jumps) + 1
    starts = np.concatenate([[0], split_after])
    ends   = np.concatenate([split_after, [len(pts)]])
    return [pts[s:e] for s, e in zip(starts, ends) if e - s >= 2]


def split_path_on_mask(pts: np.ndarray, mask: np.ndarray,
                       step_px: float = 1.0) -> list[np.ndarray]:
    """Break a polyline wherever it leaves the mask.

    Returns a list of sub-polylines, each contained in the foreground. Points
    outside the mask are dropped entirely — the plotter will lift its pen across
    the gap. `step_px` controls how finely each segment is sampled for the
    inside/outside test (smaller = more accurate, slower).
    """
    if len(pts) == 0:
        return []
    if mask is None:
        return [pts]

    h, w = mask.shape
    xi = np.clip(pts[:, 0].astype(np.int32), 0, w - 1)
    yi = np.clip(pts[:, 1].astype(np.int32), 0, h - 1)
    inside = mask[yi, xi]

    # Also test each straight segment's midpoints so we don't pass diagonals
    # across the background between two in-mask vertices.
    out: list[np.ndarray] = []
    buf: list[np.ndarray] = []

    def flush():
        if len(buf) >= 2:
            out.append(np.asarray(buf, dtype=np.float32))
        buf.clear()

    for i in range(len(pts)):
        if not inside[i]:
            flush()
            continue
        # Check the segment from the previous kept vertex to this one.
        if buf:
            prev = buf[-1]
            p = pts[i]
            seg_len = float(np.hypot(p[0] - prev[0], p[1] - prev[1]))
            n_samples = max(2, int(seg_len / step_px))
            if n_samples > 2:
                ts = np.linspace(0.0, 1.0, n_samples)[1:-1]
                sx = np.clip((prev[0] + ts * (p[0] - prev[0])).astype(np.int32), 0, w - 1)
                sy = np.clip((prev[1] + ts * (p[1] - prev[1])).astype(np.int32), 0, h - 1)
                if not mask[sy, sx].all():
                    flush()
                    buf.append(p)
                    continue
        buf.append(pts[i])
    flush()
    return out
