"""Suivi en temps réel : localiser le robot à chaque nouveau tour du lidar, sans « sauter ».

Principe du cours (localisation relative + absolue, comme un filtre à prédiction / correction) :
1. PRÉDICTION : on part de la pose précédente, déplacée du mouvement commandé depuis (odométrie par les
   commandes, voir robot.command_motion). Le robot n'a pas de codeurs : c'est une estimation grossière.
2. CORRECTION : on cherche autour de la prédiction (multi-résolution puis gradient). La zone de recherche
   grandit avec le mouvement prévu (± 60 cm / ± 20° à l'arrêt, jusqu'à ± 1,2 m / ± 60°).
3. Si moins de 35 % des points tombent sur les murs, le tour est jugé « incertain » : on garde la
   prédiction SANS l'ajouter au trajet (un tour flou pendant un virage ne doit pas faire sauter le robot).
4. Seulement après plusieurs tours incertains de suite, on cherche dans toute la carte (global), et on
   n'accepte la réponse que si elle est nette (non ambiguë, ≥ 50 % des points sur les murs).
Le trajet est une suite de morceaux : un relocalisation global en commence un nouveau (pas de trait
qui traverse la salle).

place_scan : poser un scan (pris à l'arrêt) sur une carte, depuis une pose approchée. Sert à la
cartographie pas à pas : chaque nouveau scan est posé sur les scans déjà gardés.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..geometry import Pose
from ..gridmap import OccupancyGrid
from .globalloc import _on_map, global_localize
from .grid import LikelihoodField, build_pyramid, multiresolution_gradient

LOST_BELOW = 0.35
GLOBAL_ACCEPT = 0.50
LOST_PATIENCE = 3                    # tours incertains de suite avant de chercher partout
BASE_WINDOW_M, BASE_WINDOW_DEG = 0.6, 20.0
MAX_WINDOW_M, MAX_WINDOW_DEG = 1.2, 60.0


def compose(pose: Pose, motion: tuple[float, float, float]) -> Pose:
    """Pose déplacée d'un mouvement exprimé dans le repère du robot (dx vers l'avant, dy vers la gauche, dθ en °)."""
    dx, dy, dth = motion
    c, s = np.cos(pose.theta), np.sin(pose.theta)
    return Pose(pose.x + c * dx - s * dy, pose.y + s * dx + c * dy, pose.theta_deg + dth)


@dataclass(frozen=True)
class TrackStep:
    pose: Pose
    inliers: float          # part des points à moins de 5 cm d'un mur
    mode: str               # « suivi », « global », « incertain » ou « perdu »
    ambiguous: bool
    time_s: float


class Tracker:
    def __init__(self, grid: OccupancyGrid, segments=(), field: LikelihoodField | None = None,
                 pyramid: dict | None = None) -> None:
        self.grid, self.segments = grid, list(segments)
        self.field = field or LikelihoodField.from_grid(grid)
        self.pyramid = pyramid or build_pyramid(self.field)
        self.pose: Pose | None = None
        self.trail: list[Pose | None] = []      # None = coupure du trajet (après une relocalisation globale)
        self.lost = 0

    def reset(self, pose: Pose | None = None) -> None:
        self.pose, self.trail, self.lost = pose, [], 0

    def _accept(self, pose: Pose, new_piece: bool = False) -> None:
        if new_piece and self.trail:
            self.trail.append(None)
        self.pose = pose
        self.trail = (self.trail + [pose])[-600:]

    def update(self, points_local: np.ndarray, motion: tuple[float, float, float] | None = None) -> TrackStep:
        if len(points_local) < 20:
            raise ValueError("scan trop pauvre pour se localiser (moins de 20 points)")
        if self.pose is not None:
            m = motion or (0.0, 0.0, 0.0)
            predicted = compose(self.pose, m)
            wm = min(MAX_WINDOW_M, BASE_WINDOW_M + 0.5 * float(np.hypot(m[0], m[1])))
            wd = min(MAX_WINDOW_DEG, BASE_WINDOW_DEG + 0.5 * abs(m[2]))
            r = multiresolution_gradient(points_local, predicted, self.field, pyramid=self.pyramid,
                                         window_m=wm, window_deg=wd)
            inl = float(_on_map(self.field, points_local, r.pose).mean())
            if inl >= LOST_BELOW:
                self.lost = 0
                self._accept(r.pose)
                return TrackStep(r.pose, inl, "suivi", False, r.time_s)
            self.lost += 1
            if self.lost < LOST_PATIENCE:
                self.pose = predicted                     # on garde la prédiction, sans la tracer
                return TrackStep(predicted, inl, "incertain", False, r.time_s)
        g = global_localize(points_local, self.grid, self.segments, self.field, self.pyramid)
        if self.pose is None or (not g.ambiguous and g.inliers >= GLOBAL_ACCEPT):
            self.lost = 0
            self._accept(g.pose, new_piece=True)
            return TrackStep(g.pose, g.inliers, "global", g.ambiguous, g.time_s)
        return TrackStep(self.pose, g.inliers, "perdu", True, g.time_s)


@dataclass(frozen=True)
class Placement:
    pose: Pose
    inliers: float          # part des points du nouveau scan sur les murs déjà connus (≈ recouvrement)
    how: str
    ambiguous: bool
    time_s: float


def place_scan(points_local: np.ndarray, start: Pose | None, grid: OccupancyGrid, segments=(),
               field: LikelihoodField | None = None, pyramid: dict | None = None) -> Placement:
    """Pose d'un scan pris à l'arrêt sur une carte : autour de `start` (± 1 m, ± 40°), sinon dans toute la carte.
    On garde la réponse qui pose le plus de points sur les murs."""
    field = field or LikelihoodField.from_grid(grid)
    pyramid = pyramid or build_pyramid(field)
    cands = []
    if start is not None:
        r = multiresolution_gradient(points_local, start, field, pyramid=pyramid, window_m=1.0, window_deg=40.0)
        cands.append((float(_on_map(field, points_local, r.pose).mean()), r.pose, "autour de la pose prévue",
                      False, r.time_s))
    if not cands or cands[0][0] < GLOBAL_ACCEPT:
        g = global_localize(points_local, grid, segments, field, pyramid)
        cands.append((g.inliers, g.pose, "recherche dans toute la carte", g.ambiguous, g.time_s))
    best = max(cands, key=lambda c: c[0])
    return Placement(best[1], best[0], best[2], best[3], sum(c[4] for c in cands))
