"""La grande carte centrale : couches d'affichage + interactions souris.

Souris : molette = zoom autour du curseur ; clic gauche glissé = déplacer la vue ;
mode règle : deux clics = distance et angle entre deux points.
"""

from __future__ import annotations

from typing import Callable

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from ..geometry import Pose
from ..scan import Scan
from ..segments import Segment
from . import theme as T

ZOOM_STEP = 1.2


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
        self.robot_pose = Pose()
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

    # ------------------------------------------------------------------ souris
    def _on_scroll(self, ev) -> None:
        if ev.xdata is None:
            return
        f = 1 / ZOOM_STEP if ev.button == "up" else ZOOM_STEP
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        self.ax.set_xlim(ev.xdata + (x0 - ev.xdata) * f, ev.xdata + (x1 - ev.xdata) * f)
        self.ax.set_ylim(ev.ydata + (y0 - ev.ydata) * f, ev.ydata + (y1 - ev.ydata) * f)
        self.redraw()

    def _on_press(self, ev) -> None:
        if ev.xdata is None or ev.button != 1:
            return
        if self.ruler_mode:
            self._ruler_click(ev.xdata, ev.ydata)
        else:
            self._drag = (ev.x, ev.y, self.ax.get_xlim(), self.ax.get_ylim())

    def _on_release(self, _ev) -> None:
        self._drag = None

    def _on_motion(self, ev) -> None:
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
