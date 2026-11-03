"""Outils de la carte de LidarLab : chacun reçoit les clics (press / motion / release) quand il est actif.

- PoseTool    : clic = position, glisser = orientation (pose du robot, pose de départ d'une localisation)
- RectTool    : glisser un rectangle (gomme à points)
- WallTool    : retouches de la carte vectorielle : tracer, prolonger (déplacer une extrémité), supprimer
Un outil rend True quand il a « consommé » l'événement (sinon la carte fait son geste habituel).
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from matplotlib.patches import Rectangle

from ..geometry import Pose
from ..mapedit import ADD, DELETE, WallEdit
from ..segments import Segment
from . import style as S

PICK_PX = 12     # tolérance d'un clic sur un mur ou une extrémité (pixels)
SNAP_PX = 10     # une extrémité lâchée près d'une autre s'y colle


class PoseTool:
    def __init__(self, view, on_pose: Callable[[Pose], None], color: str = S.INITIAL) -> None:
        self.view, self.on_pose, self.color = view, on_pose, color
        self.start = None
        self.theta = 0.0
        (self.line,) = view.ax.plot([], [], "-", color=color, lw=3, zorder=9, solid_capstyle="round")
        (self.dot,) = view.ax.plot([], [], "o", color=color, ms=10, mec="white", mew=2, zorder=9)

    def press(self, ev) -> bool:
        if ev.button != 1:
            return False
        self.start = (ev.xdata, ev.ydata)
        self._draw(ev.xdata, ev.ydata)
        return True

    def motion(self, ev) -> bool:
        if self.start is None:
            return False
        dx, dy = ev.xdata - self.start[0], ev.ydata - self.start[1]
        if np.hypot(dx, dy) > 3 * self.view.pixel_size():
            self.theta = float(np.degrees(np.arctan2(dy, dx)))
        self._draw(*self.start)
        return True

    def release(self, _ev) -> None:
        if self.start is not None:
            x, y = self.start
            self.start = None
            self.on_pose(Pose(float(x), float(y), self.theta))

    def _draw(self, x, y) -> None:
        t = np.radians(self.theta)
        self.dot.set_data([x], [y])
        self.line.set_data([x, x + 0.5 * np.cos(t)], [y, y + 0.5 * np.sin(t)])
        self.view.redraw()

    def close(self) -> None:
        self.line.remove()
        self.dot.remove()
        self.view.redraw()


class RectTool:
    def __init__(self, view, on_box: Callable[[tuple], None]) -> None:
        self.view, self.on_box = view, on_box
        self.start = None
        self.patch = Rectangle((0, 0), 0, 0, fill=True, facecolor="#FF3B3022", edgecolor=S.DANGER, lw=1.5,
                               ls="--", zorder=9, visible=False)
        view.ax.add_patch(self.patch)

    def press(self, ev) -> bool:
        if ev.button != 1:
            return False
        self.start = (ev.xdata, ev.ydata)
        self.patch.set_bounds(ev.xdata, ev.ydata, 0, 0)
        self.patch.set_visible(True)
        return True

    def motion(self, ev) -> bool:
        if self.start is None:
            return False
        x0, y0 = self.start
        self.patch.set_bounds(min(x0, ev.xdata), min(y0, ev.ydata), abs(ev.xdata - x0), abs(ev.ydata - y0))
        self.view.redraw()
        return True

    def release(self, ev) -> None:
        if self.start is None:
            return
        x0, y0 = self.start
        self.start = None
        self.patch.set_visible(False)
        self.view.redraw()
        if ev.xdata is not None and abs(ev.xdata - x0) > 2 * self.view.pixel_size():
            self.on_box((x0, y0, ev.xdata, ev.ydata))

    def close(self) -> None:
        self.patch.remove()
        self.view.redraw()


def _point_segment_distance(p: np.ndarray, s: Segment) -> float:
    a, b = np.array([s.x1, s.y1]), np.array([s.x2, s.y2])
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


class WallTool:
    """mode = « tracer » | « prolonger » | « supprimer ». get() rend les murs affichés ; chaque geste
    produit des retouches (mapedit.WallEdit) envoyées à on_edit : elles comptent pour les deux cartes."""

    def __init__(self, view, mode: str, get: Callable[[], list[Segment]],
                 on_edit: Callable[[list[WallEdit]], None]) -> None:
        self.view, self.mode, self.get, self.on_edit = view, mode, get, on_edit
        self.start = None
        self.grab: tuple[int, int] | None = None         # (segment, extrémité 0/1) saisie pour « prolonger »
        (self.line,) = view.ax.plot([], [], "-", color=S.ACCENT, lw=3, zorder=9, solid_capstyle="round")

    def _snap(self, x: float, y: float, skip: tuple[int, int] | None = None) -> tuple[float, float]:
        tol = SNAP_PX * self.view.pixel_size()
        best, best_d = (x, y), tol
        for k, s in enumerate(self.get()):
            for e, (ex, ey) in enumerate(((s.x1, s.y1), (s.x2, s.y2))):
                d = np.hypot(ex - x, ey - y)
                if (k, e) != skip and d < best_d:
                    best, best_d = (ex, ey), d
        return best

    def press(self, ev) -> bool:
        if ev.button != 1:
            return False
        p = np.array([ev.xdata, ev.ydata])
        segs, tol = self.get(), PICK_PX * self.view.pixel_size()
        if self.mode == "supprimer":
            d = [_point_segment_distance(p, s) for s in segs]
            if d and min(d) < tol:
                self.on_edit([WallEdit(DELETE, segs[int(np.argmin(d))])])
            return True
        if self.mode == "prolonger":
            ends = [(np.hypot(s.x1 - p[0], s.y1 - p[1]), k, 0) for k, s in enumerate(segs)]
            ends += [(np.hypot(s.x2 - p[0], s.y2 - p[1]), k, 1) for k, s in enumerate(segs)]
            if ends and min(ends)[0] < tol:
                _, k, e = min(ends)
                self.grab = (k, e)
                self.start = (segs[k].x2, segs[k].y2) if e == 0 else (segs[k].x1, segs[k].y1)   # extrémité fixe
            return True
        self.start = self._snap(ev.xdata, ev.ydata)          # tracer
        return True

    def motion(self, ev) -> bool:
        if self.start is None:
            return False
        x, y = self._snap(ev.xdata, ev.ydata, self.grab)
        self.line.set_data([self.start[0], x], [self.start[1], y])
        self.view.redraw()
        return True

    def release(self, ev) -> None:
        if self.start is None:
            return
        x0, y0 = self.start
        self.start = None
        self.line.set_data([], [])
        if ev.xdata is None:
            self.view.redraw()
            return
        x, y = self._snap(ev.xdata, ev.ydata, self.grab)
        segs = self.get()
        if self.grab is not None:
            k, e = self.grab
            self.grab = None
            new = Segment(x, y, x0, y0) if e == 0 else Segment(x0, y0, x, y)
            self.on_edit([WallEdit(DELETE, segs[k]), WallEdit(ADD, new)])
        elif np.hypot(x - x0, y - y0) > 0.05:
            self.on_edit([WallEdit(ADD, Segment(x0, y0, x, y))])
        else:
            self.view.redraw()

    def close(self) -> None:
        self.line.remove()
        self.view.redraw()
