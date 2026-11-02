"""Visualiseur de nuages de points : plusieurs scans, une couleur par scan, pose modifiable.

    python -m lidar.viewer data/scans/*.csv --poses data/poses.json

Commandes :
- cliquer un scan dans la liste (à gauche) pour le sélectionner ;
- curseurs x / y / θ, ou clavier : flèches = ±5 cm, a / e = ±2°, (Maj = pas ×5) ;
- r : remettre la pose lue dans le fichier ; s : enregistrer toutes les poses dans le JSON.
"""

from __future__ import annotations

import argparse
import glob

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons, Slider

from .geometry import Pose
from .poses import load_scans_with_poses, save_poses

STEP_M = 0.05
STEP_DEG = 2.0


class ScanViewer:
    def __init__(self, scans, poses_file: str):
        self.scans = [s.filtered() for s in scans]
        self.initial = [s.pose for s in self.scans]
        self.poses_file = poses_file
        self.sel = 0
        self._updating = False

        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_axes([0.25, 0.22, 0.72, 0.73])
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        colors = plt.get_cmap("tab10")
        self.points = []
        self.robots = []
        for k, s in enumerate(self.scans):
            c = colors(k % 10)
            pts = s.global_points()
            self.points.append(self.ax.scatter(pts[:, 0], pts[:, 1], s=3, color=c, label=s.name))
            (rob,) = self.ax.plot([], [], "-o", color=c, markersize=6, markevery=[0])
            self.robots.append(rob)
        self.ax.legend(loc="upper right", fontsize=8, markerscale=4)

        names = [s.name for s in self.scans]
        self.radio = RadioButtons(self.fig.add_axes([0.01, 0.4, 0.2, 0.5]), names)
        self.radio.on_clicked(self._on_select)
        self.sx = Slider(self.fig.add_axes([0.3, 0.12, 0.6, 0.03]), "x (m)", -10, 10, valinit=0)
        self.sy = Slider(self.fig.add_axes([0.3, 0.08, 0.6, 0.03]), "y (m)", -10, 10, valinit=0)
        self.st = Slider(self.fig.add_axes([0.3, 0.04, 0.6, 0.03]), "θ (°)", -180, 180, valinit=0)
        for sl in (self.sx, self.sy, self.st):
            sl.on_changed(self._on_slider)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.text(0.01, 0.02, "flèches/a/e : déplacer   r : reset   s : sauver poses", fontsize=8)
        self._sync_sliders()
        self._redraw_all()

    # --- interactions -------------------------------------------------------
    def _on_select(self, label: str) -> None:
        self.sel = [s.name for s in self.scans].index(label)
        self._sync_sliders()

    def _on_slider(self, _val) -> None:
        if self._updating:
            return
        self._set_pose(Pose(self.sx.val, self.sy.val, self.st.val))

    def _on_key(self, event) -> None:
        key = event.key or ""
        mult = 5 if key.startswith("shift") else 1
        key = key.replace("shift+", "")
        p = self.scans[self.sel].pose
        moves = {
            "left": (-STEP_M, 0, 0), "right": (STEP_M, 0, 0),
            "up": (0, STEP_M, 0), "down": (0, -STEP_M, 0),
            "a": (0, 0, STEP_DEG), "e": (0, 0, -STEP_DEG),
            "A": (0, 0, STEP_DEG), "E": (0, 0, -STEP_DEG),
        }
        if key in moves:
            dx, dy, dt = (v * mult for v in moves[key])
            self._set_pose(p.moved(dx, dy, dt))
            self._sync_sliders()
        elif key == "r":
            self._set_pose(self.initial[self.sel])
            self._sync_sliders()
        elif key == "s":
            path = save_poses(self.scans, self.poses_file)
            self.ax.set_title(f"Poses enregistrées dans {path}")
            self.fig.canvas.draw_idle()

    # --- rendu ----------------------------------------------------------------
    def _set_pose(self, pose: Pose) -> None:
        self.scans[self.sel] = self.scans[self.sel].with_pose(pose)
        self._redraw(self.sel)
        self.fig.canvas.draw_idle()

    def _sync_sliders(self) -> None:
        p = self.scans[self.sel].pose
        self._updating = True
        self.sx.set_val(p.x)
        self.sy.set_val(p.y)
        self.st.set_val(((p.theta_deg + 180) % 360) - 180)
        self._updating = False

    def _redraw(self, k: int) -> None:
        s = self.scans[k]
        self.points[k].set_offsets(s.global_points())
        # robot = point + trait de 20 cm indiquant son orientation
        p = s.pose
        self.robots[k].set_data(
            [p.x, p.x + 0.2 * np.cos(p.theta)], [p.y, p.y + 0.2 * np.sin(p.theta)]
        )

    def _redraw_all(self) -> None:
        for k in range(len(self.scans)):
            self._redraw(k)
        self.ax.relim()
        self.ax.autoscale_view()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="fichiers CSV (les jokers * sont acceptés)")
    ap.add_argument("--poses", default="data/poses.json", help="fichier JSON des poses (lu et écrit)")
    args = ap.parse_args()
    files = sorted({f for pattern in args.files for f in (glob.glob(pattern) or [pattern])})
    ScanViewer(load_scans_with_poses(files, args.poses), args.poses)
    plt.show()


if __name__ == "__main__":
    main()
