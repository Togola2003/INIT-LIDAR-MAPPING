"""Suivi en temps réel : localiser le robot à chaque nouveau scan qu'il envoie.

- 1er scan (ou quand on est perdu) : localisation GLOBALE dans toute la carte (≈ 0,7 s) ;
- scans suivants : le robot a peu bougé depuis le scan précédent, on cherche donc AUTOUR de la
  pose précédente (multi-résolution ± 60 cm / ± 20°, puis gradient) : ≈ 10-20 ms ;
- si trop peu de points tombent sur les murs (< 35 %), on se dit perdu et on repasse en global.
C'est le principe de la localisation relative + absolue du cours : la pose précédente sert de
prédiction (comme le ferait l'odométrie), le lidar la corrige.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..geometry import Pose
from ..gridmap import OccupancyGrid
from .globalloc import _on_map, global_localize
from .grid import LikelihoodField, build_pyramid, multiresolution_gradient

LOST_BELOW = 0.35
TRACK_WINDOW_M, TRACK_WINDOW_DEG = 0.6, 20.0


@dataclass(frozen=True)
class TrackStep:
    pose: Pose
    inliers: float          # part des points à moins de 5 cm d'un mur
    mode: str               # « global » ou « suivi »
    ambiguous: bool
    time_s: float


class Tracker:
    def __init__(self, grid: OccupancyGrid, segments=()) -> None:
        self.grid, self.segments = grid, list(segments)
        self.field = LikelihoodField.from_grid(grid)
        self.pyramid = build_pyramid(self.field)
        self.pose: Pose | None = None
        self.trail: list[Pose] = []

    def reset(self, pose: Pose | None = None) -> None:
        self.pose, self.trail = pose, []

    def update(self, points_local: np.ndarray) -> TrackStep:
        if len(points_local) < 20:
            raise ValueError("scan trop pauvre pour se localiser (moins de 20 points)")
        step = None
        if self.pose is not None:
            r = multiresolution_gradient(points_local, self.pose, self.field, pyramid=self.pyramid,
                                         window_m=TRACK_WINDOW_M, window_deg=TRACK_WINDOW_DEG)
            inl = float(_on_map(self.field, points_local, r.pose).mean())
            if inl >= LOST_BELOW:
                step = TrackStep(r.pose, inl, "suivi", False, r.time_s)
        if step is None:
            g = global_localize(points_local, self.grid, self.segments, self.field, self.pyramid)
            step = TrackStep(g.pose, g.inliers, "global", g.ambiguous, g.time_s)
        self.pose = step.pose
        self.trail = (self.trail + [step.pose])[-400:]
        return step
