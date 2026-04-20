"""
blot_ui.py — PySide6 GUI for the Blot pen plotter.

Two tabs:
- Generate: photo → stipple/TSP/spline via vpype → embedded preview
- Control:  live serial console to drive Blot, with jog buttons, custom
            commands, and pause/cancel-able G-code streaming

Launch via the vpype pipx env (PySide6 / QtViewer / vpype_cli / pyserial on path):

    /Users/samliu/.local/pipx/venvs/vpype/bin/python rmrrf/ui/blot_ui.py
"""

from __future__ import annotations

import hashlib
import os
import re
import subprocess
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Qt reads QT_SCALE_FACTOR when QApplication is constructed. Set it before any
# PySide6 import triggers Qt initialization so the whole UI renders at 75%.
os.environ.setdefault("QT_SCALE_FACTOR", "0.8")

from PySide6.QtCore import QObject, QPoint, QRect, QSize, QThread, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPixmap, QTextCursor
from PySide6.QtMultimedia import (
    QCamera,
    QImageCapture,
    QMediaCaptureSession,
    QMediaDevices,
)
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import serial.tools.list_ports
import vpype as vp
from vpype_cli import execute
from vpype_viewer.qtviewer import QtViewer

import photo_to_lineart as ptl
import pintr_lineart as pil
import scribble_lineart as scl
from blot_serial import SerialWorker
from combing import CombingPlanner, apply_combing


HERE = Path(__file__).parent
BLOT_CONFIG = HERE / "blot_vpype.toml"
BG_REMOVE_SWIFT = HERE / "remove_bg.swift"
OUTLINE_SCRIPT = HERE / "mask_outline.py"


def _bg_cache_path(input_path: str) -> Path:
    """Stable cache path keyed on absolute path + mtime + size."""
    st = os.stat(input_path)
    key = f"{os.path.abspath(input_path)}:{st.st_mtime_ns}:{st.st_size}"
    h = hashlib.md5(key.encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / f"blot_bg_{h}.png"


def ensure_camera_permission() -> None:
    """Trigger the macOS camera-permission prompt once.

    Qt's QCamera tries to use the camera immediately but gets silently denied
    on first launch because the Python binary has no camera entitlement. Going
    through AVFoundation's explicit request() asks macOS to show the native
    prompt, after which Qt can use the camera normally.
    """
    try:
        import AVFoundation   # from pyobjc-framework-AVFoundation
    except ImportError:
        return
    status = AVFoundation.AVCaptureDevice.authorizationStatusForMediaType_(
        AVFoundation.AVMediaTypeVideo
    )
    if status in (2, 1):  # denied / restricted → can't re-prompt
        print(
            "Camera access is denied — grant it in "
            "System Settings → Privacy & Security → Camera, "
            "then relaunch.",
            file=sys.stderr,
        )
        return
    if status == 3:
        return
    # notDetermined → ask the OS to show the prompt. Fire-and-forget; the
    # callback runs on a background queue and we don't block on it.
    AVFoundation.AVCaptureDevice.requestAccessForMediaType_completionHandler_(
        AVFoundation.AVMediaTypeVideo, lambda granted: None
    )


def remove_background(input_path: str) -> str:
    """Run the Vision-framework Swift script to isolate the foreground.

    Caches by (abs path, mtime, size) so re-running Generate on the same file
    skips the ~1-2s Swift round-trip. Falls back to the untouched input if
    `swift` is missing or the script errors.
    """
    cache = _bg_cache_path(input_path)
    if cache.exists():
        return str(cache)
    try:
        result = subprocess.run(
            ["swift", str(BG_REMOVE_SWIFT), input_path, str(cache)],
            capture_output=True, text=True, timeout=120,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        print(f"Background removal skipped: {exc}", file=sys.stderr)
        return input_path
    if result.returncode != 0 or not cache.exists():
        print(f"Background removal failed: {result.stderr.strip()}", file=sys.stderr)
        return input_path
    return str(cache)

# (display name, script filename, python module holding defaults)
ALGOS: dict[str, tuple[str, object]] = {
    "Stipple / TSP":        ("photo_to_lineart.py", ptl),
    "Scribble (PicoPrint)": ("scribble_lineart.py", scl),
    "Long lines (pintr)":   ("pintr_lineart.py",    pil),
}

PIPELINE_TAIL_FMT = (
    "linemerge --tolerance 0.5mm "
    "linesimplify --tolerance 0.05mm "
    "filter --min-length 0.5mm "
    "linesort "
    "pagesize {canvas}mmx{canvas}mm"
)

# --- Branding (hackclub.com watermark in the top-left) -----------------------
# 1 CSS px = 1/96 in, so 1 mm = 96/25.4 px — vpype uses pixels internally.
_PX_PER_MM = 96.0 / 25.4
_BRANDING_TEXT = "HACKCLUB.COM"
_BRANDING_FONT = "futural"
_BRANDING_SIZE_MM = 5.0
_BRANDING_EDGE_MM = 2.5           # distance from canvas edge to glyph top/left
_BRANDING_MARGIN_MM = 1.5         # whitespace kept clear around the text
_BRANDING_LAYER = 3


def _branding_text_cmd(
    layer: int = _BRANDING_LAYER,
    pos_px: tuple[float, float] | None = None,
    scale: float = 1.0,
) -> str:
    """Emit the vpype `text` command. `pos_px` overrides the baseline position
    (in pixels) — used by the measurement pass to render at (0, 0) before the
    caller translates the result into the corner with matching margins.
    `scale` multiplies the font size."""
    if pos_px is None:
        x_str = y_str = "0"
    else:
        x_str = f"{pos_px[0]}px"
        y_str = f"{pos_px[1]}px"
    size_mm = _BRANDING_SIZE_MM * scale
    return (
        f"text --font {_BRANDING_FONT} "
        f"--size {size_mm}mm "
        f"--position {x_str} {y_str} "
        f"--layer {layer} "
        f'"{_BRANDING_TEXT}"'
    )


def _clip_polyline_against_box(
    line: np.ndarray, box: tuple[float, float, float, float],
    step_px: float = 2.0,
) -> list[np.ndarray]:
    """Split a vpype polyline wherever it enters the [x0,y0,x1,y1] box.

    Points inside are dropped; consecutive outside points are also broken at
    segments whose midpoints cross the box (so a long diagonal between two
    outside vertices can't streak ink through the branding whitespace).
    The plotter lifts its pen between the returned sub-lines.
    """
    if len(line) < 2:
        return []
    x0, y0, x1, y1 = box
    pts = np.column_stack([line.real, line.imag])

    out: list[np.ndarray] = []
    buf: list[complex] = []

    def flush() -> None:
        if len(buf) >= 2:
            out.append(np.asarray(buf, dtype=np.complex128))
        buf.clear()

    for i in range(len(pts)):
        px, py = float(pts[i, 0]), float(pts[i, 1])
        if x0 <= px <= x1 and y0 <= py <= y1:
            flush()
            continue
        if buf:
            prev = buf[-1]
            seg_len = float(np.hypot(px - prev.real, py - prev.imag))
            if seg_len > step_px:
                n = max(2, int(seg_len / step_px))
                ts = np.linspace(0.0, 1.0, n)[1:-1]
                sx = prev.real + ts * (px - prev.real)
                sy = prev.imag + ts * (py - prev.imag)
                if ((sx >= x0) & (sx <= x1) & (sy >= y0) & (sy <= y1)).any():
                    flush()
                    buf.append(complex(px, py))
                    continue
        buf.append(complex(px, py))
    flush()
    return out


def _split_strokes_at_incidence(
    lines: list[np.ndarray], tol: float, max_iters: int = 6,
) -> list[np.ndarray]:
    """Split each polyline wherever another polyline's endpoint lands on its
    interior (within `tol`). This turns Hershey glyphs like `K`, where the
    diagonals meet the vertical bar at its midpoint, into a graph with shared
    endpoints that downstream traversal can chain without pen-lifts.
    """
    def do_pass(pool: list[np.ndarray]) -> tuple[list[np.ndarray], bool]:
        endpoints = [pl[0] for pl in pool] + [pl[-1] for pl in pool]
        out: list[np.ndarray] = []
        changed = False
        for pl in pool:
            if len(pl) < 2:
                continue
            pts = np.column_stack([pl.real, pl.imag])
            own = {
                (round(pl[0].real, 2), round(pl[0].imag, 2)),
                (round(pl[-1].real, 2), round(pl[-1].imag, 2)),
            }
            splits: dict[tuple[int, float], None] = {}
            for ep in endpoints:
                if (round(ep.real, 2), round(ep.imag, 2)) in own:
                    continue
                best: tuple[int, float, float] | None = None
                for i in range(len(pts) - 1):
                    p0x, p0y = pts[i]
                    p1x, p1y = pts[i + 1]
                    sx, sy = p1x - p0x, p1y - p0y
                    sl = sx * sx + sy * sy
                    if sl < 1e-9:
                        continue
                    t = ((ep.real - p0x) * sx + (ep.imag - p0y) * sy) / sl
                    t = max(0.0, min(1.0, t))
                    qx = p0x + t * sx
                    qy = p0y + t * sy
                    d = float(np.hypot(ep.real - qx, ep.imag - qy))
                    if d < tol and 0.05 < t < 0.95:
                        if best is None or d < best[2]:
                            best = (i, float(t), d)
                if best is not None:
                    splits[(best[0], round(best[1], 4))] = None
            if not splits:
                out.append(pl)
                continue
            changed = True
            ordered = sorted(splits.keys())
            parts: list[np.ndarray] = []
            current: list[complex] = [complex(pl[0])]
            idx = 0
            for i in range(len(pl) - 1):
                p0 = pl[i]
                p1 = pl[i + 1]
                while idx < len(ordered) and ordered[idx][0] == i:
                    _, t = ordered[idx]
                    new_pt = p0 + t * (p1 - p0)
                    current.append(new_pt)
                    if len(current) >= 2:
                        parts.append(np.asarray(current, dtype=np.complex128))
                    current = [new_pt]
                    idx += 1
                current.append(p1)
            if len(current) >= 2:
                parts.append(np.asarray(current, dtype=np.complex128))
            out.extend(parts)
        return out, changed

    lines = [pl for pl in lines if len(pl) >= 2]
    for _ in range(max_iters):
        lines, changed = do_pass(lines)
        if not changed:
            break
    return lines


def _optimize_text_strokes(lc, tol_px: float) -> list[np.ndarray]:
    """Minimize pen-lifts for a Hershey-text LineCollection.

    1. Pre-split strokes where one touches another's interior.
    2. Cluster endpoints into graph nodes.
    3. Per connected component, greedy-walk edges; when stuck with edges
       unvisited elsewhere, backtrack over already-drawn strokes until a
       vertex has an unvisited edge. The backtracked edges are re-emitted
       in the output polyline, so the pen stays down and simply re-traces.

    Each connected component becomes one output polyline → one pen-down per
    glyph.
    """
    raw = [np.asarray(pl, dtype=np.complex128) for pl in lc if len(pl) >= 2]
    if not raw:
        return []
    lines = _split_strokes_at_incidence(raw, tol_px)

    nodes_xy: list[tuple[float, float]] = []

    def cluster(pt: complex) -> int:
        for i, (nx, ny) in enumerate(nodes_xy):
            if abs(nx - pt.real) < tol_px and abs(ny - pt.imag) < tol_px:
                return i
        nodes_xy.append((float(pt.real), float(pt.imag)))
        return len(nodes_xy) - 1

    edges: list[tuple[int, int, np.ndarray]] = []
    for pl in lines:
        a = cluster(pl[0])
        b = cluster(pl[-1])
        edges.append((a, b, pl))

    adj: list[list[tuple[int, int, int]]] = [[] for _ in nodes_xy]
    for e_id, (a, b, _) in enumerate(edges):
        adj[a].append((b, e_id, 0))    # traverse polyline forward
        adj[b].append((a, e_id, 1))    # traverse polyline reversed

    # Connected components via union-find.
    parent = list(range(len(nodes_xy)))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x: int, y: int) -> None:
        parent[find(x)] = find(y)

    for a, b, _ in edges:
        union(a, b)

    comps: dict[int, set[int]] = {}
    for n in range(len(nodes_xy)):
        comps.setdefault(find(n), set()).add(n)

    result: list[np.ndarray] = []
    for node_set in comps.values():
        comp_edge_set = {e_id for e_id, (a, _, _) in enumerate(edges) if a in node_set}
        if not comp_edge_set:
            continue
        # Prefer starting at an odd-degree node (a glyph leaf, e.g. the tip
        # of an `L`) so the traversal naturally reaches all leaves.
        degs = {n: sum(1 for _, e_id, _ in adj[n] if e_id in comp_edge_set)
                for n in node_set}
        odd = [n for n, d in degs.items() if d % 2 == 1]
        start = odd[0] if odd else next(iter(node_set))

        visited: set[int] = set()
        path: list[tuple[int, int]] = []
        stack = [start]
        current = start
        guard = 0
        limit = len(comp_edge_set) * 8 + 16

        while guard < limit:
            guard += 1
            step = next(
                ((nbr, e_id, rev) for nbr, e_id, rev in adj[current]
                 if e_id in comp_edge_set and e_id not in visited),
                None,
            )
            if step is not None:
                nbr, e_id, rev = step
                path.append((e_id, rev))
                visited.add(e_id)
                stack.append(nbr)
                current = nbr
                continue
            if len(visited) == len(comp_edge_set):
                break
            if len(stack) < 2:
                break
            prev = stack[-2]
            retrace = next(
                ((e_id, rev) for nbr, e_id, rev in adj[current]
                 if nbr == prev and e_id in comp_edge_set),
                None,
            )
            if retrace is None:
                break
            path.append(retrace)
            stack.pop()
            current = prev

        stroke: list[complex] = []
        for e_id, rev in path:
            poly = edges[e_id][2]
            if rev:
                poly = poly[::-1]
            if stroke:
                stroke.extend(poly[1:])
            else:
                stroke.extend(poly)
        if len(stroke) >= 2:
            result.append(np.asarray(stroke, dtype=np.complex128))
    return result


ALL_ALGOS = frozenset({"stipple", "scribble", "pintr"})


@dataclass
class Knob:
    attr: str
    label: str
    lo: float
    hi: float
    step: float
    kind: str                 # "float" | "int" | "bool"
    algos: frozenset[str]     # which algorithms use this knob
    decimals: int = 2


KNOBS: list[Knob] = [
    # Common (all algorithms)
    Knob("canvas_mm",       "Paper size (mm)",   20,   125,   1,    "int",   ALL_ALGOS),
    Knob("min_value",       "White above",       0.2,  1.0,   0.01, "float", ALL_ALGOS),
    Knob("max_value",       "Black below",       0.0,  0.8,   0.01, "float", ALL_ALGOS),
    Knob("contrast",        "Contrast",          0.3,  4.0,   0.05, "float", ALL_ALGOS),
    Knob("gamma",           "Gamma",             0.3,  3.0,   0.05, "float", ALL_ALGOS),
    Knob("pen_width_mm",    "Pen width (mm)",    0.1,  2.0,   0.05, "float", ALL_ALGOS),
    Knob("work_size_px",    "Resolution (px)",   128,  10000, 32,   "int",   ALL_ALGOS),
    Knob("smooth_subdiv",   "Smooth subdiv",     1,    16,    1,    "int",   frozenset({"stipple", "scribble"})),
    # Stipple / TSP
    Knob("density",         "Density",           0.05, 10.0,  0.05, "float", frozenset({"stipple"})),
    Knob("lloyd_iters",     "Lloyd iters",       1,    80,    1,    "int",   frozenset({"stipple"})),
    Knob("two_opt_passes",  "2-opt passes",      0,    5,     1,    "int",   frozenset({"stipple"})),
    Knob("min_island_frac", "Min island (frac)", 0.0,  0.2,   0.005,"float", frozenset({"stipple"})),
    Knob("draw_outline",    "Draw outline",      0,    1,     1,    "bool",  frozenset({"stipple"})),
    # Scribble (PicoPrint)
    Knob("pen_blackness",   "Pen blackness",     4.0,  255.0, 1.0,  "float", frozenset({"scribble"})),
    Knob("max_darkness",    "Max darkness",      0.05, 1.5,   0.05, "float", frozenset({"scribble"})),
    Knob("max_steps",       "Max steps",         1000, 100000, 500, "int",   frozenset({"scribble"})),
    Knob("angle_samples",   "Angle samples",     50,   2000,  10,   "int",   frozenset({"scribble"})),
    Knob("base_segment_px", "Min segment (px)",  1.0,  20.0,  0.5,  "float", frozenset({"scribble"})),
    Knob("max_segment_px",  "Max segment (px)",  4.0,  200.0, 1.0,  "float", frozenset({"scribble"})),
    # Long lines (pintr)
    Knob("total_lines",     "Total lines",       100,  20000, 100,  "int",   frozenset({"pintr"})),
    Knob("candidates",      "Candidates",        3,    200,   1,    "int",   frozenset({"pintr"})),
    Knob("pen_opacity",     "Pen opacity",       0.05, 1.0,   0.05, "float", frozenset({"pintr"})),
    Knob("single_line",     "Single line",       0,    1,     1,    "bool",  frozenset({"pintr"})),
]


# Algorithm key → python module (for reading default values per knob).
ALGO_KEYS: dict[str, object] = {
    "stipple":  ptl,
    "scribble": scl,
    "pintr":    pil,
}


def _algo_key(display_name: str) -> str:
    """Map the human-readable algorithm name to the short key used on knobs."""
    return {
        "Stipple / TSP":        "stipple",
        "Scribble (PicoPrint)": "scribble",
        "Long lines (pintr)":   "pintr",
    }[display_name]


def _default_for(knob: Knob):
    """Find a module among the knob's algos that defines this attr as a default."""
    for key in ("stipple", "scribble", "pintr"):
        if key in knob.algos:
            mod = ALGO_KEYS[key]
            if hasattr(mod, knob.attr.upper()):
                return getattr(mod, knob.attr.upper())
    return knob.lo


# --------------------------------------------------------- time estimation ---


# Conservative Blot plot rates. Real firmware caps F100000 well below the draw
# rate here — we err on the slow side so the pre-send estimate doesn't under-
# promise mid-job. Per-line overhead covers the ok/ack round-trip at 115200.
_DRAW_MM_S = 60.0
_TRAVEL_MM_S = 150.0
_PER_LINE_S = 0.012
_PEN_TOGGLE_S = 0.15

_XY_RE = re.compile(r"([XY])(-?\d+(?:\.\d+)?)", re.IGNORECASE)


def estimate_gcode_time(lines: list[str]) -> float:
    """Parse G-code and estimate total streaming time in seconds."""
    x = y = 0.0
    t = 0.0
    for raw in lines:
        s = raw.strip()
        if not s or s.startswith(";"):
            continue
        t += _PER_LINE_S
        head = s.split(None, 1)[0].upper()
        if head in ("M3", "M5") or head.startswith("M3") or head.startswith("M5"):
            t += _PEN_TOGGLE_S
            continue
        if not (head.startswith("G0") or head.startswith("G1")):
            continue
        nx, ny = x, y
        for axis, val in _XY_RE.findall(s):
            if axis.upper() == "X":
                nx = float(val)
            else:
                ny = float(val)
        dist = ((nx - x) ** 2 + (ny - y) ** 2) ** 0.5
        rate = _TRAVEL_MM_S if head.startswith("G0") else _DRAW_MM_S
        t += dist / rate
        x, y = nx, ny
    return t


def fmt_hms(seconds: float) -> str:
    s = max(0, int(round(seconds)))
    if s < 60:
        return f"{s}s"
    if s < 3600:
        return f"{s // 60}:{s % 60:02d}"
    return f"{s // 3600}:{(s % 3600) // 60:02d}:{s % 60:02d}"


class CropWidget(QWidget):
    """Shows a captured QPixmap with a draggable, resizable square crop overlay.

    - Drag inside the square to move it.
    - Drag near any corner to resize (stays square, clamped to the image).
    - The overlay darkens everything outside the crop so it's obvious what will
      be used.
    """

    HANDLE_PX = 16            # grab region for the corner handles (widget px)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(200, 200)
        self.setMouseTracking(True)
        self._pixmap: QPixmap | None = None
        self._crop = QRect()              # in image pixel coords
        self._drag_mode: str | None = None   # "move" | "nw"/"ne"/"sw"/"se" | None
        self._drag_origin_mouse = QPoint()
        self._drag_origin_crop = QRect()
        self._img_rect = QRect()          # where the image is painted, in widget coords
        self._scale = 1.0                 # widget px per image px

    # ------- public API -------
    def set_image(self, pixmap: QPixmap) -> None:
        self._pixmap = pixmap
        size = min(pixmap.width(), pixmap.height())
        self._crop = QRect(
            (pixmap.width() - size) // 2,
            (pixmap.height() - size) // 2,
            size, size,
        )
        self.update()

    def cropped(self) -> QPixmap | None:
        if self._pixmap is None or self._crop.isEmpty():
            return None
        return self._pixmap.copy(self._crop)

    # ------- painting -------
    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(30, 30, 30))
        if self._pixmap is None:
            return

        scaled = self._pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        ox = (self.width() - scaled.width()) // 2
        oy = (self.height() - scaled.height()) // 2
        self._img_rect = QRect(ox, oy, scaled.width(), scaled.height())
        self._scale = scaled.width() / float(self._pixmap.width())
        painter.drawPixmap(ox, oy, scaled)

        # Darken the regions outside the crop.
        cx = ox + int(self._crop.x() * self._scale)
        cy = oy + int(self._crop.y() * self._scale)
        cw = int(self._crop.width() * self._scale)
        ch = int(self._crop.height() * self._scale)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 140))
        # top, bottom, left, right strips
        painter.drawRect(ox, oy, scaled.width(), cy - oy)
        painter.drawRect(ox, cy + ch, scaled.width(), oy + scaled.height() - (cy + ch))
        painter.drawRect(ox, cy, cx - ox, ch)
        painter.drawRect(cx + cw, cy, ox + scaled.width() - (cx + cw), ch)

        # Crop rectangle border + handles.
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("white"), 2))
        painter.drawRect(cx, cy, cw, ch)
        handle = self.HANDLE_PX
        painter.setBrush(QColor("white"))
        painter.setPen(Qt.NoPen)
        for hx, hy in ((cx, cy), (cx + cw, cy), (cx, cy + ch), (cx + cw, cy + ch)):
            painter.drawRect(hx - handle // 2, hy - handle // 2, handle, handle)

    # ------- mouse interaction -------
    def _widget_to_image(self, wpt: QPoint) -> QPoint:
        if self._pixmap is None or self._scale == 0:
            return QPoint(0, 0)
        x = (wpt.x() - self._img_rect.x()) / self._scale
        y = (wpt.y() - self._img_rect.y()) / self._scale
        return QPoint(int(round(x)), int(round(y)))

    def _hit_handle(self, wpt: QPoint) -> str | None:
        if self._pixmap is None:
            return None
        cx = self._img_rect.x() + self._crop.x() * self._scale
        cy = self._img_rect.y() + self._crop.y() * self._scale
        cw = self._crop.width() * self._scale
        ch = self._crop.height() * self._scale
        h = self.HANDLE_PX
        corners = {
            "nw": (cx, cy),
            "ne": (cx + cw, cy),
            "sw": (cx, cy + ch),
            "se": (cx + cw, cy + ch),
        }
        for name, (hx, hy) in corners.items():
            if abs(wpt.x() - hx) <= h and abs(wpt.y() - hy) <= h:
                return name
        # Inside the crop → move
        if cx <= wpt.x() <= cx + cw and cy <= wpt.y() <= cy + ch:
            return "move"
        return None

    def mousePressEvent(self, event) -> None:
        if self._pixmap is None:
            return
        mode = self._hit_handle(event.position().toPoint())
        if mode is None:
            return
        self._drag_mode = mode
        self._drag_origin_mouse = event.position().toPoint()
        self._drag_origin_crop = QRect(self._crop)

    def mouseMoveEvent(self, event) -> None:
        if self._pixmap is None:
            self.setCursor(Qt.ArrowCursor)
            return
        wpt = event.position().toPoint()
        if self._drag_mode is None:
            # Hover cursor feedback.
            mode = self._hit_handle(wpt)
            self.setCursor({
                "move": Qt.SizeAllCursor,
                "nw":   Qt.SizeFDiagCursor,
                "se":   Qt.SizeFDiagCursor,
                "ne":   Qt.SizeBDiagCursor,
                "sw":   Qt.SizeBDiagCursor,
            }.get(mode, Qt.ArrowCursor))
            return

        img_pixel_delta_x = (wpt.x() - self._drag_origin_mouse.x()) / self._scale
        img_pixel_delta_y = (wpt.y() - self._drag_origin_mouse.y()) / self._scale
        iw, ih = self._pixmap.width(), self._pixmap.height()
        orig = self._drag_origin_crop

        if self._drag_mode == "move":
            nx = int(round(orig.x() + img_pixel_delta_x))
            ny = int(round(orig.y() + img_pixel_delta_y))
            nx = max(0, min(nx, iw - orig.width()))
            ny = max(0, min(ny, ih - orig.height()))
            self._crop = QRect(nx, ny, orig.width(), orig.height())
        else:
            # Square-constrained corner drag: the new side length is the max of
            # the |x| and |y| delta, signed to match the corner being pulled.
            sx = +1 if "e" in self._drag_mode else -1
            sy = +1 if "s" in self._drag_mode else -1
            dx = sx * img_pixel_delta_x
            dy = sy * img_pixel_delta_y
            delta = max(dx, dy)                       # keep square, follow max
            new_side = max(16, int(round(orig.width() + delta)))
            # Anchor the opposite corner, then clamp to the image.
            if sx > 0:
                x0 = orig.x()
            else:
                x0 = orig.x() + orig.width() - new_side
            if sy > 0:
                y0 = orig.y()
            else:
                y0 = orig.y() + orig.height() - new_side
            # Clamp so the square stays fully inside the image.
            new_side = min(new_side, iw, ih)
            x0 = max(0, min(x0, iw - new_side))
            y0 = max(0, min(y0, ih - new_side))
            self._crop = QRect(x0, y0, new_side, new_side)
        self.update()

    def mouseReleaseEvent(self, _event) -> None:
        self._drag_mode = None


class KnobRow(QWidget):
    """Slider + spin-box pair (or checkbox for bools) with bidirectional sync."""

    def __init__(self, knob: Knob, default) -> None:
        super().__init__()
        self.knob = knob
        self.default = default

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        if knob.kind == "bool":
            self.check = QCheckBox()
            self.check.setChecked(bool(int(default)))
            lay.addWidget(self.check)
            lay.addStretch(1)
            self.slider = None
            self.spin = None
            return

        # Slider works in ints; pick a scale so `step` maps to 1 slider tick.
        self.scale = (max(1, int(round(1.0 / knob.step)))
                      if knob.kind == "float" else 1)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(round(knob.lo * self.scale)))
        self.slider.setMaximum(int(round(knob.hi * self.scale)))
        self.slider.setSingleStep(max(1, int(round(knob.step * self.scale))))
        self.slider.setPageStep(max(1, int(round(knob.step * self.scale * 10))))

        if knob.kind == "float":
            self.spin: QSpinBox | QDoubleSpinBox = QDoubleSpinBox()
            self.spin.setDecimals(knob.decimals)
            self.spin.setRange(float(knob.lo), float(knob.hi))
            self.spin.setSingleStep(float(knob.step))
        else:
            self.spin = QSpinBox()
            self.spin.setRange(int(knob.lo), int(knob.hi))
            self.spin.setSingleStep(int(knob.step))
        self.spin.setMinimumWidth(76)
        self.spin.setMaximumWidth(96)
        self.spin.setButtonSymbols(QDoubleSpinBox.NoButtons)

        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)

        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)

        self.set_value(default)

    def _on_slider(self, v: int) -> None:
        val = v / self.scale if self.knob.kind == "float" else int(v)
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)

    def _on_spin(self, v) -> None:
        sv = int(round(v * self.scale)) if self.knob.kind == "float" else int(v)
        self.slider.blockSignals(True)
        self.slider.setValue(sv)
        self.slider.blockSignals(False)

    def value(self):
        if self.knob.kind == "bool":
            return 1 if self.check.isChecked() else 0
        return self.spin.value()

    def set_value(self, v) -> None:
        if self.knob.kind == "bool":
            self.check.setChecked(bool(int(v)))
            return
        if self.knob.kind == "float":
            self.spin.setValue(float(v))
        else:
            self.spin.setValue(int(v))

    def reset(self) -> None:
        self.set_value(self.default)


# Common Blot commands for jog buttons on the control tab.
QUICK_CMDS: list[tuple[str, str]] = [
    ("Enable motors (M17)",  "M17"),
    ("Disable motors (M18)", "M18"),
    ("Set origin (G92)",     "G92 X0 Y0"),
    ("Pen up (M5)",          "M5"),
    ("Pen down (M3)",        "M3 S1700"),
    ("Home (G28)",           "G28"),
    ("Status (?)",           "?"),
]


# ---------------------------------------------------------- generate worker --


class GenerateWorker(QObject):
    done = Signal(object)       # vp.Document
    failed = Signal(str)
    progress = Signal(str)

    def __init__(self, script_path: Path, params: dict[str, str], remove_bg: bool):
        super().__init__()
        self.script_path = script_path
        self.params = params
        self.remove_bg = remove_bg
        self._mask_path_used: str | None = None

    def run(self) -> None:
        try:
            if self.remove_bg and self.params.get("input"):
                self.progress.emit("Removing background…")
                fg_path = remove_background(self.params["input"])
                self.params["input"] = fg_path
                # remove_bg.swift writes the mask as "<stem>.mask.png" next to
                # the foreground PNG. If present, hand it to the generator so
                # it can clip stipple points, split the pen path at background
                # crossings, and trace the island outlines.
                mask_path = Path(fg_path).with_suffix("").as_posix() + ".mask.png"
                if Path(mask_path).exists():
                    self.params["mask"] = mask_path
                    self._mask_path_used = mask_path
            for key, val in self.params.items():
                os.environ["PTL_" + key.upper()] = val
            self.progress.emit("Rendering…")
            canvas = float(self.params.get("canvas_mm", 125.0))
            canvas_str = f"{canvas:g}"
            tail = PIPELINE_TAIL_FMT.format(canvas=canvas_str)
            # Chain the main generator (layer 1) and — when a mask is available
            # and this is the stipple algorithm — the outline (layer 2). vpype's
            # viewer renders each layer in its own color so the outline stands
            # out from the stipple fill.
            want_outline = (
                self.script_path.name == "photo_to_lineart.py"
                and bool(self.params.get("mask"))
                and int(self.params.get("draw_outline", 1))
            )
            if want_outline:
                pipeline = (
                    f'script -l 1 "{self.script_path}" '
                    f'script -l 2 "{OUTLINE_SCRIPT}" '
                    f'{tail}'
                )
            else:
                pipeline = f'script "{self.script_path}" {tail}'
            doc = execute(pipeline)
            self.done.emit(doc)
        except Exception as exc:
            self.failed.emit(f"{exc}\n{traceback.format_exc(limit=2)}")


# ---------------------------------------------------------- main window ------


class Main(QMainWindow):
    # cross-thread requests to SerialWorker (QueuedConnection)
    connect_req = Signal(str, int)
    disconnect_req = Signal()
    send_line_req = Signal(str)
    stream_req = Signal(list)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Blot")
        self.resize(1320, 880)

        self.last_doc: vp.Document | None = None
        self.knob_widgets: dict[str, QWidget] = {}
        self.knob_labels: dict[str, QLabel] = {}
        self._gcode_cache: list[str] | None = None
        self._estimated_total_s: float = 0.0
        self._stream_started: float | None = None
        # Mask path & canvas from the last successful generate, for combing.
        self._last_mask_path: str | None = None
        self._last_canvas_mm: float = 125.0

        # central = tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._build_camera_tab(), "Camera")
        self.tabs.addTab(self._build_generate_tab(), "Generate")
        self.tabs.addTab(self._build_control_tab(), "Control")
        # Pause the camera whenever the Camera tab isn't showing.
        self.tabs.currentChanged.connect(self._on_tab_changed)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready.")

        # serial worker in its own thread
        self.serial_thread = QThread(self)
        self.serial = SerialWorker()
        self.serial.moveToThread(self.serial_thread)

        self.connect_req.connect(self.serial.connect_port)
        self.disconnect_req.connect(self.serial.disconnect_port)
        self.send_line_req.connect(self.serial.send_line)
        self.stream_req.connect(self.serial.stream)

        self.serial.log.connect(self._append_log)
        self.serial.connected.connect(self._on_connected)
        self.serial.state_changed.connect(self._on_state)
        self.serial.progress.connect(self._on_stream_progress)
        self.serial.stream_done.connect(self._on_stream_done)

        self.serial_thread.start()

        self._refresh_ports()
        self._set_connected(False)
        self._on_algo_change(self.algo_combo.currentText())

    # ---------------------------- Generate tab ----------------------------

    def _build_generate_tab(self) -> QWidget:
        tab = QWidget()
        root = QHBoxLayout(tab)

        panel = QWidget()
        panel.setFixedWidth(320)
        controls = QVBoxLayout(panel)

        file_box = QGroupBox("Image")
        fl = QVBoxLayout(file_box)
        row = QHBoxLayout()
        self.path_edit = QLineEdit(ptl.INPUT)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        row.addWidget(self.path_edit, 1)
        row.addWidget(browse)
        fl.addLayout(row)
        self.bg_toggle = QCheckBox("Remove background (Apple Vision)")
        self.bg_toggle.setChecked(True)
        fl.addWidget(self.bg_toggle)
        self.comb_toggle = QCheckBox("Comb pen-up travel (keep above foreground)")
        self.comb_toggle.setChecked(True)
        fl.addWidget(self.comb_toggle)
        controls.addWidget(file_box)

        algo_box = QGroupBox("Algorithm")
        al = QHBoxLayout(algo_box)
        self.algo_combo = QComboBox()
        self.algo_combo.addItems(list(ALGOS.keys()))
        self.algo_combo.currentTextChanged.connect(self._on_algo_change)
        al.addWidget(self.algo_combo, 1)
        controls.addWidget(algo_box)

        layout_box = QGroupBox("Layout")
        lform = QFormLayout(layout_box)
        lform.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.branding_toggle = QCheckBox()
        self.branding_toggle.setChecked(True)
        lform.addRow(QLabel("Branding"), self.branding_toggle)
        scale_knob = Knob("scale", "Scale", 0.4, 2.5, 0.05, "float", frozenset())
        self.scale_row = KnobRow(scale_knob, 1.0)
        scale_label = QLabel("Branding scale")
        lform.addRow(scale_label, self.scale_row)
        # Scale only affects the branding text — dim it when branding is off.
        self.branding_toggle.toggled.connect(self.scale_row.setEnabled)
        self.branding_toggle.toggled.connect(scale_label.setEnabled)
        controls.addWidget(layout_box)

        knobs_box = QGroupBox("Parameters")
        form = QFormLayout(knobs_box)
        form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        for k in KNOBS:
            default = _default_for(k)
            row = KnobRow(k, default)
            label = QLabel(k.label)
            self.knob_widgets[k.attr] = row
            self.knob_labels[k.attr] = label
            form.addRow(label, row)
        controls.addWidget(knobs_box)

        self.reset_btn = QPushButton("Reset to defaults")
        self.reset_btn.clicked.connect(self._reset_knobs)
        controls.addWidget(self.reset_btn)

        self.generate_btn = QPushButton("Generate")
        self.generate_btn.setDefault(True)
        self.generate_btn.clicked.connect(self._generate)
        controls.addWidget(self.generate_btn)

        self.save_btn = QPushButton("Save G-code…")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self._save_gcode)
        controls.addWidget(self.save_btn)

        self.save_svg_btn = QPushButton("Save SVG…")
        self.save_svg_btn.setEnabled(False)
        self.save_svg_btn.clicked.connect(self._save_svg)
        controls.addWidget(self.save_svg_btn)

        controls.addStretch(1)
        hint = QLabel("Canvas: 125 × 125 mm. Tweak, click Generate, then\n"
                      "switch to the Control tab to stream it to the Blot.")
        hint.setStyleSheet("color: #888;")
        hint.setWordWrap(True)
        controls.addWidget(hint)

        root.addWidget(panel)

        self.viewer = QtViewer()
        self.viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self.viewer, 1)
        return tab

    # ---------------------------- Camera tab ------------------------------

    def _build_camera_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        # Top row: camera selector + refresh.
        top_box = QGroupBox("Camera")
        top = QHBoxLayout(top_box)
        top.addWidget(QLabel("Device:"))
        self.cam_combo = QComboBox()
        self.cam_combo.setMinimumWidth(260)
        top.addWidget(self.cam_combo, 1)
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self._refresh_cameras)
        top.addWidget(refresh)
        root.addWidget(top_box)

        # Stacked: live preview ↔ captured (with crop overlay).
        from PySide6.QtWidgets import QStackedWidget
        self.cam_stack = QStackedWidget()
        self.video_widget = QVideoWidget()
        self.video_widget.setMinimumHeight(400)
        self.cam_stack.addWidget(self.video_widget)
        self.crop_widget = CropWidget()
        self.cam_stack.addWidget(self.crop_widget)
        root.addWidget(self.cam_stack, 1)

        # Action row.
        actions = QHBoxLayout()
        self.capture_btn = QPushButton("Capture")
        self.capture_btn.clicked.connect(self._camera_capture)
        self.retake_btn = QPushButton("Retake")
        self.retake_btn.clicked.connect(self._camera_retake)
        self.retake_btn.setEnabled(False)
        self.use_photo_btn = QPushButton("Use cropped photo →")
        self.use_photo_btn.clicked.connect(self._camera_use_cropped)
        self.use_photo_btn.setEnabled(False)
        actions.addWidget(self.capture_btn)
        actions.addWidget(self.retake_btn)
        actions.addStretch(1)
        actions.addWidget(self.use_photo_btn)
        root.addLayout(actions)

        hint = QLabel(
            "Capture, drag the square overlay to crop, then send to the "
            "Generate tab as the input image."
        )
        hint.setStyleSheet("color: #888;")
        hint.setWordWrap(True)
        root.addWidget(hint)

        # Capture session — wired up now, camera started in _refresh_cameras().
        self.cam_session = QMediaCaptureSession()
        self.cam_session.setVideoOutput(self.video_widget)
        self.image_capture = QImageCapture()
        self.cam_session.setImageCapture(self.image_capture)
        self.image_capture.imageSaved.connect(self._on_image_saved)
        self.image_capture.errorOccurred.connect(
            lambda _id, _err, msg: self._append_log(f"Capture error: {msg}")
        )

        self._current_camera: QCamera | None = None
        self._captured_path: str | None = None

        self._refresh_cameras()
        self.cam_combo.currentIndexChanged.connect(self._on_camera_change)
        return tab

    def _refresh_cameras(self) -> None:
        current = self.cam_combo.currentData() if hasattr(self, "cam_combo") else None
        self.cam_combo.blockSignals(True)
        self.cam_combo.clear()
        for cam in QMediaDevices.videoInputs():
            self.cam_combo.addItem(cam.description(), cam.id())
        self.cam_combo.blockSignals(False)
        if self.cam_combo.count() == 0:
            return
        # Try to preserve the previous selection; else start the first camera.
        target_idx = 0
        if current is not None:
            idx = self.cam_combo.findData(current)
            if idx >= 0:
                target_idx = idx
        self.cam_combo.setCurrentIndex(target_idx)
        self._on_camera_change(target_idx)

    def _on_camera_change(self, idx: int) -> None:
        if idx < 0 or self.cam_combo.count() == 0:
            return
        cam_id = self.cam_combo.currentData()
        for cam_info in QMediaDevices.videoInputs():
            if cam_info.id() == cam_id:
                if self._current_camera is not None:
                    self._current_camera.stop()
                self._current_camera = QCamera(cam_info)
                self.cam_session.setCamera(self._current_camera)
                self._current_camera.start()
                return

    def _camera_capture(self) -> None:
        if self._current_camera is None:
            return
        fd, path = tempfile.mkstemp(suffix=".jpg", prefix="blot_cap_")
        os.close(fd)
        self._captured_path = path
        self.image_capture.captureToFile(path)

    def _on_image_saved(self, _req_id: int, path: str) -> None:
        pixmap = QPixmap(path)
        if pixmap.isNull():
            self._append_log(f"Failed to load captured image: {path}")
            return
        self.crop_widget.set_image(pixmap)
        self.cam_stack.setCurrentIndex(1)
        self.capture_btn.setEnabled(False)
        self.retake_btn.setEnabled(True)
        self.use_photo_btn.setEnabled(True)
        # Pause the live preview while we crop.
        if self._current_camera is not None:
            self._current_camera.stop()

    def _camera_retake(self) -> None:
        self.cam_stack.setCurrentIndex(0)
        self.capture_btn.setEnabled(True)
        self.retake_btn.setEnabled(False)
        self.use_photo_btn.setEnabled(False)
        if self._current_camera is not None:
            self._current_camera.start()

    def _on_tab_changed(self, idx: int) -> None:
        on_camera = self.tabs.tabText(idx) == "Camera"
        if self._current_camera is None:
            return
        # Only the live-preview mode needs the sensor running. If we're already
        # in "captured" mode on the Camera tab we leave it stopped.
        if on_camera and self.cam_stack.currentIndex() == 0:
            self._current_camera.start()
        else:
            self._current_camera.stop()

    def _camera_use_cropped(self) -> None:
        cropped = self.crop_widget.cropped()
        if cropped is None:
            return
        fd, out_path = tempfile.mkstemp(suffix=".png", prefix="blot_crop_")
        os.close(fd)
        if not cropped.save(out_path, "PNG"):
            self._append_log(f"Failed to save cropped image to {out_path}")
            return
        self.path_edit.setText(out_path)
        # Jump to the Generate tab (index 1 now that Camera is index 0).
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i) == "Generate":
                self.tabs.setCurrentIndex(i)
                break

    # ---------------------------- Control tab -----------------------------

    def _build_control_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        # -- port row --
        port_box = QGroupBox("Connection")
        hp = QHBoxLayout(port_box)
        hp.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(320)
        hp.addWidget(self.port_combo, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        hp.addWidget(self.refresh_btn)
        hp.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "9600"])
        hp.addWidget(self.baud_combo)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        hp.addWidget(self.connect_btn)
        root.addWidget(port_box)

        # -- quick commands grid --
        cmds_box = QGroupBox("Quick commands")
        grid = QGridLayout(cmds_box)
        self.quick_buttons: list[QPushButton] = []
        for idx, (label, cmd) in enumerate(QUICK_CMDS):
            btn = QPushButton(label)
            btn.clicked.connect(lambda _=False, c=cmd: self._send_line(c))
            grid.addWidget(btn, idx // 4, idx % 4)
            self.quick_buttons.append(btn)
        root.addWidget(cmds_box)

        # -- custom command row --
        custom_box = QGroupBox("Custom command")
        hc = QHBoxLayout(custom_box)
        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText("e.g. G1 X10 Y10 F1500")
        self.custom_edit.returnPressed.connect(self._send_custom)
        hc.addWidget(self.custom_edit, 1)
        self.custom_send_btn = QPushButton("Send")
        self.custom_send_btn.clicked.connect(self._send_custom)
        hc.addWidget(self.custom_send_btn)
        root.addWidget(custom_box)

        # -- stream controls --
        stream_box = QGroupBox("Stream current G-code")
        sv = QVBoxLayout(stream_box)
        row = QHBoxLayout()
        self.stream_btn = QPushButton("Send G-code")
        self.stream_btn.clicked.connect(self._start_stream)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setEnabled(False)
        self.pause_btn.toggled.connect(self._toggle_pause)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self._cancel_stream)
        row.addWidget(self.stream_btn)
        row.addWidget(self.pause_btn)
        row.addWidget(self.cancel_btn)
        row.addStretch(1)
        self.state_label = QLabel("Idle")
        self.state_label.setStyleSheet("color: #888;")
        row.addWidget(self.state_label)
        sv.addLayout(row)

        eta_row = QHBoxLayout()
        self.estimate_label = QLabel("No G-code yet")
        self.estimate_label.setStyleSheet("color: #888;")
        self.eta_label = QLabel("")
        self.eta_label.setStyleSheet("color: #888;")
        eta_row.addWidget(self.estimate_label)
        eta_row.addStretch(1)
        eta_row.addWidget(self.eta_label)
        sv.addLayout(eta_row)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setFormat("%v / %m  (%p%)")
        self.progress.setTextVisible(True)
        sv.addWidget(self.progress)
        root.addWidget(stream_box)

        # -- log --
        log_box = QGroupBox("Log")
        lv = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        mono = QFont("Menlo")
        mono.setStyleHint(QFont.Monospace)
        self.log_view.setFont(mono)
        lv.addWidget(self.log_view)

        btn_row = QHBoxLayout()
        clear = QPushButton("Clear log")
        clear.clicked.connect(self.log_view.clear)
        btn_row.addStretch(1)
        btn_row.addWidget(clear)
        lv.addLayout(btn_row)
        root.addWidget(log_box, 1)

        return tab

    # ---------------------------- Generate slots --------------------------

    def _browse(self) -> None:
        start = str(Path(self.path_edit.text()).parent) if self.path_edit.text() else ""
        path, _ = QFileDialog.getOpenFileName(
            self, "Open image", start, "Images (*.jpg *.jpeg *.png *.bmp)"
        )
        if path:
            self.path_edit.setText(path)

    def _current_algo_key(self) -> str:
        return _algo_key(self.algo_combo.currentText())

    def _on_algo_change(self, _name: str) -> None:
        key = self._current_algo_key()
        for knob in KNOBS:
            visible = key in knob.algos
            self.knob_widgets[knob.attr].setVisible(visible)
            self.knob_labels[knob.attr].setVisible(visible)

    def _params(self) -> dict[str, str]:
        key = self._current_algo_key()
        params = {"input": self.path_edit.text()}
        for knob in KNOBS:
            if key not in knob.algos:
                continue
            params[knob.attr] = str(self.knob_widgets[knob.attr].value())
        return params

    def _reset_knobs(self) -> None:
        for row in self.knob_widgets.values():
            row.reset()
        self.scale_row.reset()
        self.branding_toggle.setChecked(True)
        self.statusBar().showMessage("Parameters reset to defaults.")

    def _generate(self) -> None:
        self.generate_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.save_svg_btn.setEnabled(False)
        self.stream_btn.setEnabled(False)
        self.statusBar().showMessage("Running…")

        script_name, _mod = ALGOS[self.algo_combo.currentText()]
        script_path = HERE / script_name

        self.gen_thread = QThread(self)
        self.gen_worker = GenerateWorker(
            script_path, self._params(), self.bg_toggle.isChecked()
        )
        self.gen_worker.moveToThread(self.gen_thread)
        self.gen_thread.started.connect(self.gen_worker.run)
        self.gen_worker.done.connect(self._generate_done)
        self.gen_worker.failed.connect(self._generate_failed)
        self.gen_worker.progress.connect(self.statusBar().showMessage)
        self.gen_worker.done.connect(self.gen_thread.quit)
        self.gen_worker.failed.connect(self.gen_thread.quit)
        self.gen_thread.finished.connect(self.gen_thread.deleteLater)
        self.gen_thread.start()

    def _generate_done(self, doc: vp.Document) -> None:
        self._gcode_cache = None                # new geometry → re-render G-code on demand
        self._estimated_total_s = 0.0
        # Worker stashes the mask path it used (when bg-removal ran).
        self._last_mask_path = getattr(self.gen_worker, "_mask_path_used", None)
        try:
            self._last_canvas_mm = float(
                self.knob_widgets["canvas_mm"].value()
            )
        except Exception:
            self._last_canvas_mm = 125.0
        doc = self._apply_layout(doc)
        self.last_doc = doc
        self.viewer.set_document(doc)
        # Debug: log every layer that ended up in the doc.
        layer_summary = [
            f"L{lid}={len(lc)}p" for lid, lc in doc.layers.items()
        ]
        print(f"[generate] layers: {', '.join(layer_summary) or 'none'}", file=sys.stderr)
        lc = doc.layers.get(1)
        has = bool(lc and len(lc) > 0)
        if has:
            segs = sum(max(len(ln) - 1, 0) for ln in lc)
            length_mm = lc.length() * 25.4 / 96.0
            msg = f"{len(lc)} path(s) · {segs} segments · {length_mm:.0f} mm pen-down"
            # Render once to get a time estimate; result is cached for streaming.
            try:
                lines = self._render_gcode_lines()
                self._estimated_total_s = estimate_gcode_time(lines)
                msg += f" · {len(lines)} lines · est. {fmt_hms(self._estimated_total_s)}"
                self.estimate_label.setText(
                    f"{len(lines)} lines · est. {fmt_hms(self._estimated_total_s)}"
                )
            except Exception as exc:
                print(f"Estimate failed: {exc}", file=sys.stderr)
                self.estimate_label.setText("Estimate unavailable")
        else:
            msg = "Done (empty result — lower Gamma or raise Density)."
            self.estimate_label.setText("No G-code yet")
        self.statusBar().showMessage(msg)
        self.generate_btn.setEnabled(True)
        self.save_btn.setEnabled(has)
        self.save_svg_btn.setEnabled(has)
        self._update_stream_button()

    def _apply_layout(self, doc: vp.Document) -> vp.Document:
        """Scale the generated drawing and stamp the branding text on layer 3.

        Scale is applied around the canvas centre, so the drawing shrinks into
        the middle of the paper. Branding puts `HACKCLUB.COM` in the top-left
        and clips any stroke that would cross its reserved whitespace.
        """
        scale = float(self.scale_row.value())
        branding = self.branding_toggle.isChecked()

        if not branding:
            return doc

        try:
            tmp_doc = execute(_branding_text_cmd(scale=scale))
        except Exception as exc:
            print(f"Branding text skipped: {exc}", file=sys.stderr)
            return doc
        tlc = tmp_doc.layers.get(_BRANDING_LAYER)
        if tlc is None or len(tlc) == 0:
            return doc
        bounds = tlc.bounds()
        if bounds is None:
            return doc
        tmin_x, tmin_y, tmax_x, tmax_y = bounds
        # Shift the baseline so the glyph's top-left bbox corner lands at
        # (edge, edge) — equal top and left margin, regardless of font metrics.
        edge_px = _BRANDING_EDGE_MM * _PX_PER_MM
        shift_x = edge_px - tmin_x
        shift_y = edge_px - tmin_y
        text_max_x = tmax_x + shift_x
        text_max_y = tmax_y + shift_y
        margin_px = _BRANDING_MARGIN_MM * _PX_PER_MM
        exclusion = (0.0, 0.0, text_max_x + margin_px, text_max_y + margin_px)

        for lid in list(doc.layers.keys()):
            lc = doc.layers[lid]
            new_lines = []
            for line in lc:
                new_lines.extend(_clip_polyline_against_box(line, exclusion))
            doc.layers[lid] = vp.LineCollection(new_lines)

        doc = execute(
            _branding_text_cmd(pos_px=(shift_x, shift_y), scale=scale),
            document=doc,
        )
        # Hershey text emits one polyline per stroke and glyphs like K/H have
        # strokes joining mid-bar (not at endpoints), so linemerge can't fuse
        # them. Our optimizer splits at incidence points then greedy-walks
        # each glyph with retracing so each character becomes one pen-down.
        text_lc = doc.layers.get(_BRANDING_LAYER)
        if text_lc is not None and len(text_lc) > 0:
            optimized = _optimize_text_strokes(text_lc, tol_px=0.3 * _PX_PER_MM)
            doc.layers[_BRANDING_LAYER] = vp.LineCollection(optimized)
            doc = execute(
                f"linesort --layer {_BRANDING_LAYER} --two-opt",
                document=doc,
            )
        return doc

    def _generate_failed(self, msg: str) -> None:
        self.statusBar().showMessage(f"Error: {msg.splitlines()[0]}")
        print(msg, file=sys.stderr)
        self.generate_btn.setEnabled(True)
        self._update_stream_button()

    def _save_gcode(self) -> None:
        if self.last_doc is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save G-code", str(HERE / "out.gcode"),
            "G-code (*.gcode *.nc)"
        )
        if not path:
            return
        execute(
            f'gwrite --profile blot "{path}"',
            document=self.last_doc,
            global_opt=f'-c "{BLOT_CONFIG}"',
        )
        self.statusBar().showMessage(f"Wrote {Path(path).name}")

    def _save_svg(self) -> None:
        if self.last_doc is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save SVG", str(HERE / "out.svg"), "SVG (*.svg)"
        )
        if not path:
            return
        execute(f'write "{path}"', document=self.last_doc)
        self.statusBar().showMessage(f"Wrote {Path(path).name}")

    # ---------------------------- Control slots ---------------------------

    def _refresh_ports(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        self.port_combo.clear()
        usbmodem_idx = -1
        for i, p in enumerate(ports):
            label = f"{p.device}  —  {p.description}"
            self.port_combo.addItem(label, p.device)
            if "usbmodem" in p.device and usbmodem_idx < 0:
                usbmodem_idx = i
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
                return
        if usbmodem_idx >= 0:
            self.port_combo.setCurrentIndex(usbmodem_idx)

    def _selected_port(self) -> str:
        data = self.port_combo.currentData()
        if data:
            return str(data)
        return self.port_combo.currentText().split()[0]

    def _toggle_connect(self) -> None:
        if self.serial.is_connected:
            self.disconnect_req.emit()
        else:
            port = self._selected_port()
            if not port:
                self._append_log("No port selected.")
                return
            try:
                baud = int(self.baud_combo.currentText())
            except ValueError:
                baud = 115200
            self.connect_req.emit(port, baud)

    def _on_connected(self, ok: bool) -> None:
        self._set_connected(ok)

    def _set_connected(self, ok: bool) -> None:
        self.connect_btn.setText("Disconnect" if ok else "Connect")
        for b in self.quick_buttons:
            b.setEnabled(ok)
        self.custom_send_btn.setEnabled(ok)
        self.custom_edit.setEnabled(ok)
        self._update_stream_button()

    def _update_stream_button(self) -> None:
        have_doc = self.last_doc is not None and bool(
            self.last_doc.layers.get(1) and len(self.last_doc.layers.get(1)) > 0
        )
        streaming = self.pause_btn.isEnabled()
        self.stream_btn.setEnabled(
            self.serial.is_connected and have_doc and not streaming
        )

    def _send_line(self, cmd: str) -> None:
        if not self.serial.is_connected:
            self._append_log("Not connected.")
            return
        self.send_line_req.emit(cmd)

    def _send_custom(self) -> None:
        cmd = self.custom_edit.text().strip()
        if not cmd:
            return
        self._send_line(cmd)
        self.custom_edit.clear()

    def _start_stream(self) -> None:
        lines = self._render_gcode_lines()
        if not lines:
            self._append_log("No G-code to stream.")
            return
        self.stream_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        self.pause_btn.setChecked(False)
        self.cancel_btn.setEnabled(True)
        self.progress.setRange(0, len(lines))
        self.progress.setValue(0)
        self._stream_started = time.time()
        self.eta_label.setText("starting…")
        self.stream_req.emit(lines)

    def _toggle_pause(self, paused: bool) -> None:
        if paused:
            self.serial.pause()
            self.pause_btn.setText("Resume")
        else:
            self.serial.resume()
            self.pause_btn.setText("Pause")

    def _cancel_stream(self) -> None:
        self.serial.cancel()
        # If we were paused, unpause so the worker loop can see the cancel.
        if self.pause_btn.isChecked():
            self.pause_btn.setChecked(False)

    def _on_stream_progress(self, sent: int, total: int) -> None:
        if self.progress.maximum() != total:
            self.progress.setRange(0, max(total, 1))
        self.progress.setValue(sent)
        if self._stream_started is None or sent <= 0:
            return
        elapsed = time.time() - self._stream_started
        rate = sent / max(elapsed, 1e-3)
        remaining = max(total - sent, 0) / max(rate, 1e-3)
        self.eta_label.setText(
            f"{fmt_hms(elapsed)} elapsed · {fmt_hms(remaining)} left"
        )

    def _on_state(self, state: str) -> None:
        self.state_label.setText(state)
        color = {
            "Streaming": "#2d8",
            "Paused":    "#e90",
            "Cancelled": "#c44",
            "Error":     "#c44",
            "Idle":      "#888",
        }.get(state, "#888")
        self.state_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def _on_stream_done(self, ok: bool) -> None:
        self.pause_btn.setChecked(False)
        self.pause_btn.setText("Pause")
        self.pause_btn.setEnabled(False)
        self.cancel_btn.setEnabled(False)
        if self._stream_started is not None:
            total = time.time() - self._stream_started
            self.eta_label.setText(f"done in {fmt_hms(total)}" if ok else f"stopped after {fmt_hms(total)}")
        else:
            self.eta_label.setText("")
        self._stream_started = None
        self._update_stream_button()

    def _render_gcode_lines(self) -> list[str]:
        if self.last_doc is None:
            return []
        if self._gcode_cache is not None:
            return self._gcode_cache
        with tempfile.NamedTemporaryFile("w", suffix=".gcode", delete=False) as f:
            tmp = f.name
        try:
            execute(
                f'gwrite --profile blot "{tmp}"',
                document=self.last_doc,
                global_opt=f'-c "{BLOT_CONFIG}"',
            )
            with open(tmp) as f:
                lines = f.readlines()
            lines = self._maybe_comb(lines)
            # Park the pen off to the side so the paper is easy to lift out.
            # Explicit M5 ensures the pen is up regardless of the final state.
            lines = list(lines) + ["M5\n", "G00 X0 Y120\n"]
            self._gcode_cache = lines
            return self._gcode_cache
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    def _maybe_comb(self, lines: list[str]) -> list[str]:
        if not self.comb_toggle.isChecked():
            return lines
        if not self._last_mask_path or not Path(self._last_mask_path).exists():
            return lines
        try:
            planner = CombingPlanner(self._last_mask_path, self._last_canvas_mm)
        except Exception as exc:
            print(f"Combing planner failed: {exc}", file=sys.stderr)
            return lines
        before = len(lines)
        lines = apply_combing(lines, planner)
        added = len(lines) - before
        if added:
            self.statusBar().showMessage(
                f"{self.statusBar().currentMessage()} · combed ({added} extra moves)"
            )
        return lines

    def _append_log(self, text: str) -> None:
        self.log_view.appendPlainText(text)
        self.log_view.moveCursor(QTextCursor.End)

    # ---------------------------- lifecycle -------------------------------

    def closeEvent(self, ev) -> None:
        if getattr(self, "_current_camera", None) is not None:
            self._current_camera.stop()
        self.serial.cancel()
        if self.serial.is_connected:
            self.serial.disconnect_port()
        self.serial_thread.quit()
        self.serial_thread.wait(2000)
        super().closeEvent(ev)


def main() -> None:
    app = QApplication(sys.argv)
    # Ask macOS for camera permission before any QCamera is instantiated.
    ensure_camera_permission()
    window = Main()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
