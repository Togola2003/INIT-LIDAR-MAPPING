"""La grande carte centrale : couches d'affichage + interactions souris.

Souris : molette = zoom autour du curseur ; clic gauche glissé dans le vide = déplacer la vue ;
mode règle : deux clics = distance et angle entre deux points.

Mode « déplacer les scans » (set_editor) :
  clic sur un scan            = le sélectionner (Ctrl + clic : l'ajouter / l'enlever de la sélection)
  glisser un scan             = déplacer la sélection
  clic droit glissé           = tourner la sélection autour de son centre (Maj + glisser : pareil)
  Ctrl + molette              = tourner la sélection de 1° (Maj : 5°)
Pendant le geste, les points sont seulement redessinés avec une transformation (rien n'est recalculé) :
le mouvement reste fluide même avec des milliers de points. Le calcul est fait au relâchement.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D

from ..geometry import Pose
from ..scan import Scan
from ..segments import Segment
from . import theme as T

ZOOM_STEP = 1.2
PICK_PX = 14          # un clic à moins de 14 pixels d'un point « touche » ce scan
ROTATE_MIN_PX = 60    # clic de rotation plus près du centre que ça : rotation au déplacement horizontal
DEG_PER_PX = 0.4      # ... à raison de 0,4° par pixel


def _held(ev, modifier: str) -> bool:
    """Touche de modification enfoncée pendant un événement souris (« ctrl », « shift »)."""
    key = (ev.key or "").replace("control", "ctrl")
    return modifier in key.split("+")


@dataclass
class Editor:
    """Ce que l'application fournit pour qu'on puisse déplacer ses scans à la souris."""
    pick: Callable[[float, float, float], object]           # (x, y, tolérance m) -> clé du scan ou None
    selection: Callable[[], set]                            # clés sélectionnées
    select: Callable[[object, bool], None]                  # (clé, ajouter ?) -> met à jour la sélection
    pivot: Callable[[set], tuple[float, float]]             # centre de rotation d'un groupe
    commit: Callable[[set, float, float, float, tuple], None]   # (clés, dx, dy, dθ°, pivot)


class MapView:
    def __init__(self, master, on_cursor: Callable[[str], None]) -> None:
        self.on_cursor = on_cursor
        self.fig = Figure(facecolor=T.BG)
        self.ax = self.fig.add_axes([0.055, 0.06, 0.935, 0.925], facecolor=T.MAP_BG)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, color=T.GRID, lw=0.8)
        self.ax.tick_params(colors=T.MUTED, labelsize=8)
        for sp in self.ax.spines.values():
            sp.set_color(T.BORDER)
        self.ax.set_xlabel("x (m)", color=T.MUTED, fontsize=9)
        self.ax.set_ylabel("y (m)", color=T.MUTED, fontsize=9)
        self.ax.axhline(0, color=T.BORDER, lw=1)
        self.ax.axvline(0, color=T.BORDER, lw=1)

        self.live_art = self.ax.scatter([], [], s=5, c=T.LIVE, zorder=4, linewidths=0)
        self.live_segs: list = []
        (self.robot_line,) = self.ax.plot([], [], "-", color=T.ROBOT, lw=2.5, zorder=6)
        (self.robot_dot,) = self.ax.plot([], [], "o", color=T.ROBOT, ms=8, zorder=6)
        self.capture_arts: dict[int, list] = {}
        self.analysis_arts: list = []
        (self.ruler_line,) = self.ax.plot([], [], "--o", color=T.WARN, lw=1.5, ms=5, zorder=7)
        self.ruler_text = self.ax.text(0, 0, "", color=T.WARN, fontsize=10, zorder=8,
                                       bbox=dict(facecolor=T.SURFACE, edgecolor=T.WARN, boxstyle="round,pad=0.3"))
        self.ruler_text.set_visible(False)

        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.widget = self.canvas.get_tk_widget()
        self.widget.configure(bg=T.BG, highlightthickness=0)
        self.ruler_mode = False
        self._ruler_pts: list[tuple[float, float]] = []
        self._drag = None
        self._gesture = None
        self.editor: Editor | None = None
        self.movable: dict[object, list] = {}      # clé de scan -> artistes à transformer pendant un geste
        self.robot_pose = Pose()
        self.gesture_text = self.ax.text(0.01, 0.99, "", transform=self.ax.transAxes, va="top", ha="left",
                                         color=T.WARN, fontsize=11, zorder=9, family=T.MONO,
                                         bbox=dict(facecolor=T.SURFACE, edgecolor=T.WARN, boxstyle="round,pad=0.4"))
        self.gesture_text.set_visible(False)
        for name, cb in (("scroll_event", self._on_scroll), ("button_press_event", self._on_press),
                         ("button_release_event", self._on_release), ("motion_notify_event", self._on_motion)):
            self.canvas.mpl_connect(name, cb)

    # ------------------------------------------------------------------ couches
    def draw_live(self, scan: Scan | None, color_by_intensity: bool, segments: list[Segment]) -> None:
        for art in self.live_segs:
            art.remove()
        self.live_segs = []
        if scan is None or len(scan) == 0:
            self.live_art.set_offsets(np.empty((0, 2)))
        else:
            self.live_art.set_offsets(scan.global_points())
            if color_by_intensity:
                self.live_art.set_array(scan.intensities)
                self.live_art.set_cmap("plasma")
                self.live_art.set_clim(0, 255)
            else:
                self.live_art.set_array(None)
                self.live_art.set_facecolor(T.LIVE)
        for sg in segments:
            (ln,) = self.ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", color="#00d2ff", lw=2, zorder=5)
            self.live_segs.append(ln)
        self.redraw()

    def draw_robot(self, pose: Pose) -> None:
        self.robot_pose = pose
        self.robot_dot.set_data([pose.x], [pose.y])
        self.robot_line.set_data([pose.x, pose.x + 0.3 * np.cos(pose.theta)],
                                 [pose.y, pose.y + 0.3 * np.sin(pose.theta)])

    def draw_capture(self, number: int, scan: Scan | None, color: str = "", show_points: bool = True,
                     show_pose: bool = True) -> None:
        """Points d'une capture (repère carte) + petit triangle à la pose du robot."""
        for art in self.capture_arts.pop(number, []):
            art.remove()
        arts = []
        if scan is not None and len(scan) and show_points:
            pts = scan.global_points()
            arts.append(self.ax.scatter(pts[:, 0], pts[:, 1], s=7, c=color, zorder=3, linewidths=0))
        if scan is not None and show_pose:
            p = scan.pose
            arts += self.ax.plot(p.x, p.y, "^", color=color, ms=10, mec="#000", zorder=6)
            arts += self.ax.plot([p.x, p.x + 0.25 * np.cos(p.theta)], [p.y, p.y + 0.25 * np.sin(p.theta)],
                                 "-", color=color, lw=2, zorder=6)
        self.capture_arts[number] = arts
        self.movable[number] = arts
        self.redraw()

    def draw_analysis(self, image: np.ndarray | None, extent, segments: list[Segment],
                      reference: list[Segment], beams: tuple | None) -> None:
        """Grid map (image), segments extraits, carte de référence, cellules d'un scan (pédagogie)."""
        for art in self.analysis_arts:
            art.remove()
        self.analysis_arts = []
        if image is not None:
            self.analysis_arts.append(self.ax.imshow(
                image, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent,
                alpha=0.6, zorder=1, interpolation="nearest"))
        if beams is not None:   # (image RGBA des cellules traversées / d'impact, étendue)
            rgba, beam_extent = beams
            self.analysis_arts.append(self.ax.imshow(rgba, origin="lower", extent=beam_extent, zorder=2,
                                                     interpolation="nearest"))
        for sg in reference:
            self.analysis_arts += self.ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "--", color="#a371f7", lw=1.6, zorder=2)
        for sg in segments:
            self.analysis_arts += self.ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", color="#ffffff", lw=2.4,
                                               zorder=5, solid_capstyle="round")
        self.redraw()

    def draw_overlay(self, points: list[tuple], segments: list[tuple], poses: list[tuple] = ()) -> None:
        """Couche libre : points = [(xy, couleur, taille, marqueur[, clé])], segments = [(liste, couleur, épaisseur)],
        poses = [(Pose, couleur, en_avant[, clé])]. Remplace la couche précédente.
        La clé (facultative) rattache des points / une pose à un scan déplaçable à la souris."""
        for art in getattr(self, "overlay_arts", []):
            art.remove()
        self.overlay_arts = []
        self.movable = {k: v for k, v in self.movable.items() if k in self.capture_arts}
        for item in points:
            xy, color, size, marker = item[:4]
            if len(xy):
                art = self.ax.scatter(xy[:, 0], xy[:, 1], s=size, c=color, marker=marker,
                                      linewidths=0 if marker != "x" else 1.2, zorder=3)
                self.overlay_arts.append(art)
                if len(item) > 4:
                    self.movable.setdefault(item[4], []).append(art)
        for segs, color, lw in segments:
            for sg in segs:
                self.overlay_arts += self.ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", color=color, lw=lw,
                                                  zorder=5, solid_capstyle="round")
        for item in poses:
            pose, color, strong = item[:3]
            arts = self.ax.plot(pose.x, pose.y, "^", color=color, ms=13 if strong else 9,
                                mec="#fff" if strong else "#000", mew=1.5 if strong else 1, zorder=6)
            arts += self.ax.plot([pose.x, pose.x + 0.3 * np.cos(pose.theta)],
                                 [pose.y, pose.y + 0.3 * np.sin(pose.theta)], "-", color=color, lw=2, zorder=6)
            self.overlay_arts += arts
            if len(item) > 3:
                self.movable.setdefault(item[3], []).extend(arts)
        self.redraw()

    def fit(self, points: np.ndarray, margin: float = 0.5) -> None:
        """Cadre la vue pour montrer tous les points donnés."""
        if len(points) == 0:
            return
        lo, hi = points.min(axis=0) - margin, points.max(axis=0) + margin
        c = (lo + hi) / 2
        self.frame(Pose(float(c[0]), float(c[1])), float(max(hi - lo) / 2))

    def frame(self, center: Pose, half_size: float) -> None:
        """Cadre la vue sur 'center' (± half_size sur le plus petit côté), en remplissant toute la zone."""
        w, h = self.fig.get_size_inches()
        _, _, aw, ah = self.ax.get_position(original=True).bounds
        ratio = (w * aw) / max(h * ah, 1e-6)
        hx, hy = (half_size * ratio, half_size) if ratio >= 1 else (half_size, half_size / ratio)
        self.ax.set_xlim(center.x - hx, center.x + hx)
        self.ax.set_ylim(center.y - hy, center.y + hy)
        self.redraw()

    def redraw(self) -> None:
        self.canvas.draw_idle()

    def set_ruler(self, on: bool) -> None:
        self.ruler_mode = on
        self._ruler_pts = []
        if not on:
            self.ruler_line.set_data([], [])
            self.ruler_text.set_visible(False)
            self.redraw()

    # ------------------------------------------------------------------ déplacement des scans
    def set_editor(self, editor: Editor | None) -> None:
        """Active (editor fourni) ou coupe le déplacement des scans à la souris."""
        self.editor = editor
        self._gesture = None
        self.gesture_text.set_visible(False)
        self.redraw()

    def pixel_size(self) -> float:
        """Taille d'un pixel écran en mètres (pour la tolérance d'un clic)."""
        x0, x1 = self.ax.get_xlim()
        return abs(x1 - x0) / max(self.ax.bbox.width, 1)

    def _start_gesture(self, ev, rotate: bool) -> bool:
        ed = self.editor
        key = ed.pick(ev.xdata, ev.ydata, PICK_PX * self.pixel_size())
        if key is None:
            if rotate and ed.selection():           # clic droit dans le vide : tourne quand même la sélection
                key = next(iter(ed.selection()))
            else:
                return False
        additive = _held(ev, "ctrl")
        if additive:
            ed.select(key, True)
            return True                              # Ctrl + clic : on ne fait que (dé)sélectionner
        if key not in ed.selection():
            ed.select(key, False)
        keys = set(ed.selection())
        self._gesture = dict(keys=keys, rotate=rotate, x0=ev.xdata, y0=ev.ydata, pivot=ed.pivot(keys),
                             dx=0.0, dy=0.0, dth=0.0)
        return True

    def _apply_gesture(self, ev) -> None:
        g = self._gesture
        if g["rotate"]:
            cx, cy = g["pivot"]
            if np.hypot(g["x0"] - cx, g["y0"] - cy) < ROTATE_MIN_PX * self.pixel_size():
                # clic trop près du centre : l'angle serait instable -> on tourne avec le déplacement horizontal
                g["dth"] = float((ev.xdata - g["x0"]) / self.pixel_size() * DEG_PER_PX)
            else:
                a0 = np.arctan2(g["y0"] - cy, g["x0"] - cx)
                a1 = np.arctan2(ev.ydata - cy, ev.xdata - cx)
                g["dth"] = float(np.degrees((a1 - a0 + np.pi) % (2 * np.pi) - np.pi))
            aff = Affine2D().rotate_deg_around(cx, cy, g["dth"])
            label = f"rotation  {g['dth']:+.1f}°  autour de ({cx:.2f}, {cy:.2f})"
        else:
            g["dx"], g["dy"] = ev.xdata - g["x0"], ev.ydata - g["y0"]
            aff = Affine2D().translate(g["dx"], g["dy"])
            label = f"déplacement  Δx {g['dx']:+.3f} m   Δy {g['dy']:+.3f} m"
        n = len(g["keys"])
        self.gesture_text.set_text(f"{label}   ({n} scan{'s' if n > 1 else ''})")
        self.gesture_text.set_visible(True)
        self._transform_movable(g["keys"], aff)
        self.redraw()

    def _transform_movable(self, keys, aff: Affine2D | None) -> None:
        t = self.ax.transData if aff is None else aff + self.ax.transData
        for k in keys:
            for art in self.movable.get(k, []):
                if hasattr(art, "set_offset_transform"):       # nuage de points (scatter)
                    art.set_offset_transform(t)
                else:                                          # ligne, marqueur de pose
                    art.set_transform(t)

    def _end_gesture(self) -> None:
        g, self._gesture = self._gesture, None
        self.gesture_text.set_visible(False)
        self._transform_movable(g["keys"], None)
        if g["dx"] or g["dy"] or g["dth"]:
            self.editor.commit(g["keys"], g["dx"], g["dy"], g["dth"], g["pivot"])
        else:
            self.redraw()

    # ------------------------------------------------------------------ souris
    def _on_scroll(self, ev) -> None:
        if ev.xdata is None:
            return
        if self.editor is not None and _held(ev, "ctrl") and self.editor.selection():
            keys = set(self.editor.selection())
            step = 5.0 if _held(ev, "shift") else 1.0
            self.editor.commit(keys, 0.0, 0.0, step if ev.button == "up" else -step, self.editor.pivot(keys))
            return
        f = 1 / ZOOM_STEP if ev.button == "up" else ZOOM_STEP
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        self.ax.set_xlim(ev.xdata + (x0 - ev.xdata) * f, ev.xdata + (x1 - ev.xdata) * f)
        self.ax.set_ylim(ev.ydata + (y0 - ev.ydata) * f, ev.ydata + (y1 - ev.ydata) * f)
        self.redraw()

    def _on_press(self, ev) -> None:
        if ev.xdata is None or ev.button not in (1, 3):
            return
        if self.ruler_mode and ev.button == 1:
            self._ruler_click(ev.xdata, ev.ydata)
            return
        if self.editor is not None:
            rotate = ev.button == 3 or _held(ev, "shift")
            if self._start_gesture(ev, rotate):
                return
        if ev.button == 1:
            self._drag = (ev.x, ev.y, self.ax.get_xlim(), self.ax.get_ylim())

    def _on_release(self, _ev) -> None:
        self._drag = None
        if self._gesture is not None:
            self._end_gesture()

    def _on_motion(self, ev) -> None:
        if self._gesture is not None and ev.xdata is not None:
            self._apply_gesture(ev)
            return
        if self._drag is not None and ev.x is not None:
            x, y, (xa, xb), (ya, yb) = self._drag
            bbox = self.ax.bbox
            dx = (ev.x - x) * (xb - xa) / bbox.width
            dy = (ev.y - y) * (yb - ya) / bbox.height
            self.ax.set_xlim(xa - dx, xb - dx)
            self.ax.set_ylim(ya - dy, yb - dy)
            self.redraw()
        if ev.xdata is not None:
            p = self.robot_pose
            dx, dy = ev.xdata - p.x, ev.ydata - p.y
            r = np.hypot(dx, dy)
            a = (np.degrees(np.arctan2(dy, dx)) - p.theta_deg) % 360
            self.on_cursor(f"carte  x {ev.xdata:+.2f} m   y {ev.ydata:+.2f} m      robot →  r {r:.2f} m   θ {a:.0f}°")

    def _ruler_click(self, x: float, y: float) -> None:
        if len(self._ruler_pts) >= 2:
            self._ruler_pts = []
        self._ruler_pts.append((x, y))
        xs, ys = zip(*self._ruler_pts)
        self.ruler_line.set_data(xs, ys)
        if len(self._ruler_pts) == 2:
            (xa, ya), (xb, yb) = self._ruler_pts
            d = np.hypot(xb - xa, yb - ya)
            ang = np.degrees(np.arctan2(yb - ya, xb - xa))
            self.ruler_text.set_text(f"{d:.3f} m  ·  {ang:.1f}°")
            self.ruler_text.set_position(((xa + xb) / 2, (ya + yb) / 2))
            self.ruler_text.set_visible(True)
        else:
            self.ruler_text.set_visible(False)
        self.redraw()
