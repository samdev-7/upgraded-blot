"""
combing.py — mask-aware pen-up travel planner (3D-print-style combing).

Post-processes G-code: every pen-up travel (`M5 … G00 X Y … M3`) is tested
against the foreground mask. If a straight rapid would cross background, the
jump is replaced with a sequence of `G00` waypoints that stay inside the
mask. Works by:

  1. Eroding the mask slightly (safety margin) so travel doesn't hug edges.
  2. Skeletonizing the safe region → a 1-pixel-wide "highway" through the
     interior.
  3. For each jump, snapping the endpoints to the nearest skeleton pixel and
     running A* along the skeleton graph.
  4. Douglas-Peucker simplification to keep the waypoint count low.

Complexity is dominated by A* across ~O(perimeter) skeleton nodes, not the full
mask pixel count, so per-jump planning is milliseconds even on 256×256 masks.
"""

from __future__ import annotations

import heapq
import re
from pathlib import Path

import numpy as np
from PIL import Image
from skimage.morphology import disk, erosion, skeletonize


PX_PER_MM = 96.0 / 25.4


def _douglas_peucker(pts: list[tuple[float, float]], tol: float) -> list[tuple[float, float]]:
    """Polyline simplification — drop vertices inside `tol` of the chord."""
    if len(pts) < 3:
        return list(pts)
    a = np.asarray(pts[0], dtype=np.float32)
    b = np.asarray(pts[-1], dtype=np.float32)
    ab = b - a
    denom = float(np.hypot(ab[0], ab[1])) or 1.0
    best_i, best_d = 0, 0.0
    for i in range(1, len(pts) - 1):
        p = np.asarray(pts[i], dtype=np.float32)
        d = abs(ab[0] * (a[1] - p[1]) - (a[0] - p[0]) * ab[1]) / denom
        if d > best_d:
            best_d, best_i = d, i
    if best_d <= tol:
        return [pts[0], pts[-1]]
    left = _douglas_peucker(pts[: best_i + 1], tol)
    right = _douglas_peucker(pts[best_i:], tol)
    return left[:-1] + right


class CombingPlanner:
    """Plans interior pen-up routes given a foreground mask."""

    def __init__(
        self,
        mask_path: str | Path,
        canvas_mm: float,
        work_size: int = 256,
        safety_margin_px: int = 2,
    ) -> None:
        self.canvas_mm = float(canvas_mm)
        img = Image.open(str(mask_path)).convert("L").resize(
            (work_size, work_size), Image.LANCZOS
        )
        self.mask = np.asarray(img) > 128
        self.h, self.w = self.mask.shape
        # One work-px in mm (assume square canvas — matches our pipeline).
        self.mm_per_px = self.canvas_mm / self.w

        # Erode for a safety margin so travel doesn't hug edges (the pen body
        # extends past the tip). Fall back to un-eroded if it collapses.
        if safety_margin_px > 0:
            safe = erosion(self.mask, disk(safety_margin_px)).astype(bool)
            self.safe = safe if safe.any() else self.mask.copy()
        else:
            self.safe = self.mask.copy()

        self.skel = skeletonize(self.safe)
        ys, xs = np.nonzero(self.skel)
        self.skel_ys = ys.astype(np.int32)
        self.skel_xs = xs.astype(np.int32)
        n = len(ys)
        self.idx_map = np.full(self.mask.shape, -1, dtype=np.int64)
        if n > 0:
            self.idx_map[self.skel_ys, self.skel_xs] = np.arange(n)

        # Neighbor lists (8-connectivity, Euclidean edge weights).
        self.neighbors: list[list[tuple[int, float]]] = [[] for _ in range(n)]
        for i in range(n):
            y, x = int(self.skel_ys[i]), int(self.skel_xs[i])
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < self.h and 0 <= nx < self.w:
                        j = int(self.idx_map[ny, nx])
                        if j >= 0:
                            self.neighbors[i].append((j, float(np.hypot(dy, dx))))

    # ------- mm <-> work-pixel conversions -------
    def mm_to_px(self, xy_mm: tuple[float, float]) -> tuple[float, float]:
        return (xy_mm[0] / self.mm_per_px, xy_mm[1] / self.mm_per_px)

    def px_to_mm(self, xy_px: tuple[float, float]) -> tuple[float, float]:
        return (xy_px[0] * self.mm_per_px, xy_px[1] * self.mm_per_px)

    # ------- geometry tests -------
    def line_in_safe(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        length = max(2, int(np.hypot(x1 - x0, y1 - y0)) + 1)
        ts = np.linspace(0.0, 1.0, length, dtype=np.float32)
        xs = np.clip(np.round(x0 + ts * (x1 - x0)).astype(np.int32), 0, self.w - 1)
        ys = np.clip(np.round(y0 + ts * (y1 - y0)).astype(np.int32), 0, self.h - 1)
        return bool(self.safe[ys, xs].all())

    def _nearest_skel(self, x: float, y: float) -> int:
        if len(self.skel_ys) == 0:
            return -1
        dx = self.skel_xs.astype(np.float32) - x
        dy = self.skel_ys.astype(np.float32) - y
        return int(np.argmin(dx * dx + dy * dy))

    # ------- A* on the skeleton graph -------
    def _astar(self, start: int, goal: int) -> list[int]:
        if start == goal:
            return [start]
        gx = int(self.skel_xs[goal])
        gy = int(self.skel_ys[goal])
        open_heap: list[tuple[float, int]] = [(0.0, start)]
        came_from: dict[int, int | None] = {start: None}
        cost: dict[int, float] = {start: 0.0}
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == goal:
                break
            for nb, w in self.neighbors[cur]:
                nc = cost[cur] + w
                if nb not in cost or nc < cost[nb]:
                    cost[nb] = nc
                    nx = int(self.skel_xs[nb])
                    ny = int(self.skel_ys[nb])
                    heapq.heappush(open_heap, (nc + np.hypot(nx - gx, ny - gy), nb))
                    came_from[nb] = cur
        if goal not in came_from:
            return []
        path = []
        cur: int | None = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

    # ------- main entry point -------
    def plan(self, start_mm: tuple[float, float], end_mm: tuple[float, float]
             ) -> list[tuple[float, float]] | None:
        """Return list of waypoints in mm (excluding start and end) that keep
        the pen-up travel inside the mask. None if a straight rapid is fine."""
        x0, y0 = self.mm_to_px(start_mm)
        x1, y1 = self.mm_to_px(end_mm)
        if self.line_in_safe(x0, y0, x1, y1):
            return None
        if len(self.skel_ys) == 0:
            return None

        s = self._nearest_skel(x0, y0)
        g = self._nearest_skel(x1, y1)
        if s < 0 or g < 0:
            return None
        idxs = self._astar(s, g)
        if not idxs:
            return None
        pts_px = [(float(self.skel_xs[i]), float(self.skel_ys[i])) for i in idxs]
        # Reduce the waypoint blizzard — a 1.0-px tolerance removes the stair
        # stepping of 8-connectivity without losing the overall route.
        pts_px = _douglas_peucker(pts_px, tol=1.0)
        return [self.px_to_mm(p) for p in pts_px]


# --------------------------------------------------------------------- G-code -

_G_XY = re.compile(r"([XY])\s*(-?\d+(?:\.\d+)?)", re.IGNORECASE)


def _parse_xy(line: str) -> tuple[float | None, float | None]:
    x = y = None
    for axis, val in _G_XY.findall(line):
        if axis.upper() == "X":
            x = float(val)
        else:
            y = float(val)
    return x, y


def apply_combing(lines: list[str], planner: CombingPlanner) -> list[str]:
    """Insert G00 combing waypoints into `lines` wherever a pen-up travel
    would cross the background. Pure-function; returns a new list of lines."""
    out: list[str] = []
    cx = cy = 0.0
    pen_up = True                 # firmware boots pen-up; our preamble also M5s

    # We process line-by-line. When we encounter a G00 that follows an M5
    # (pen-up move), we insert waypoints just before it.
    for raw in lines:
        line = raw.rstrip("\n\r")
        stripped = line.strip()
        upper = stripped.upper()

        if upper.startswith("M5"):
            pen_up = True
            out.append(raw)
            continue
        if upper.startswith("M3"):
            pen_up = False
            out.append(raw)
            continue

        if upper.startswith("G00") or upper.startswith("G0 "):
            nx, ny = _parse_xy(stripped)
            if pen_up and nx is not None and ny is not None:
                waypoints = planner.plan((cx, cy), (nx, ny))
                if waypoints:
                    for wx, wy in waypoints:
                        out.append(f"G00 X{wx:.3f} Y{wy:.3f}\n")
            if nx is not None:
                cx = nx
            if ny is not None:
                cy = ny
            out.append(raw)
            continue

        if upper.startswith("G01") or upper.startswith("G1 "):
            nx, ny = _parse_xy(stripped)
            if nx is not None:
                cx = nx
            if ny is not None:
                cy = ny
            out.append(raw)
            continue

        out.append(raw)
    return out
