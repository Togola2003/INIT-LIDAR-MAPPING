"""Localisation entre scans (Jour 2, référence n°3) : le scan courant est aligné sur un scan de
référence pris tout près, dont la pose est connue (ses points sont donnés dans le repère carte).

- ICP point à point (calcul direct itéré, Kabsch) : déjà utilisé au Jour 1 (matching.icp).
- ICP point à droite (gradient, Gauss-Newton) : l'erreur est la distance du point à la droite du
  mur de référence (normale locale). Un point peut glisser le long du mur sans pénalité : moins
  d'itérations, meilleure précision sur les murs droits.
- Multi-résolution + ICP : la référence est rastérisée en champ de vraisemblance pour une recherche
  grossière robuste, puis l'ICP point à point termine.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.spatial import cKDTree

from ..geometry import Pose, local_to_global
from ..matching import icp, local_normals, voxel_downsample
from .common import LocResult, Timer, gauss_newton
from .grid import LikelihoodField, multiresolution


@dataclass(frozen=True)
class ScanReference:
    points: np.ndarray        # points de référence, repère carte (allégés)
    normals: np.ndarray
    flat: np.ndarray          # points situés sur une partie droite (cibles valables pour le point à droite)
    tree: cKDTree

    @staticmethod
    def from_points(points: np.ndarray, voxel: float = 0.03) -> "ScanReference":
        pts = voxel_downsample(points, voxel)
        normals, flat = local_normals(pts)
        return ScanReference(pts, normals, flat, cKDTree(pts))

    def residual(self, points: np.ndarray):
        d, k = self.tree.query(points)
        n = self.normals[k]
        r = np.einsum("ij,ij->i", n, points - self.points[k])     # distance signée à la droite du mur
        return r, n, self.flat[k] & (d < 1.0)

    def score(self, points: np.ndarray, sigma: float = 0.05) -> float:
        d, _ = self.tree.query(points)
        return float(np.exp(-d ** 2 / (2 * sigma ** 2)).mean())


def icp_point_to_point(points_local: np.ndarray, initial: Pose, ref: ScanReference) -> LocResult:
    with Timer() as tm:
        res = icp(points_local, initial, ref.points)
        score = ref.score(local_to_global(points_local, res.pose))
    return LocResult(res.pose, score, tm.elapsed, res.iterations, "scans · ICP point à point")


def icp_point_to_line(points_local: np.ndarray, initial: Pose, ref: ScanReference) -> LocResult:
    with Timer() as tm:
        pose, n = gauss_newton(points_local, initial, ref.residual)
        score = ref.score(local_to_global(points_local, pose))
    return LocResult(pose, score, tm.elapsed, n, "scans · ICP point à droite")


def multiresolution_icp(points_local: np.ndarray, initial: Pose, ref: ScanReference,
                        field: LikelihoodField | None = None) -> LocResult:
    field = field or LikelihoodField.from_points(ref.points)
    coarse = multiresolution(points_local, initial, field)
    fine = icp_point_to_point(points_local, coarse.pose, ref)
    return replace(fine, time_s=coarse.time_s + fine.time_s, evaluations=coarse.evaluations + fine.evaluations,
                   method="scans · multi-résolution + ICP")
