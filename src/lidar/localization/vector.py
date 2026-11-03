"""Localisation sur carte vectorielle (Jour 2, référence n°2).

L'erreur d'un point est sa distance au SEGMENT de mur le plus proche (d(p, feature) du cours).
- Gauss-Newton (méthode de gradient) : on fait baisser Σ w·d² en corrigeant (x, y, θ) pas à pas ;
  le gradient de la distance d'un point à un segment est la direction « pied du segment → point ».
- Calcul direct : chaque point est apparié à son pied sur le segment le plus proche, puis la
  rotation + translation qui superpose au mieux ces paires est calculée d'un coup (Kabsch) ;
  on recommence avec de nouveaux appariements (c'est un ICP point → segment).
- Multi-résolution + Gauss-Newton : les segments sont échantillonnés en points et rastérisés en
  champ de vraisemblance, pour une recherche grossière robuste ; Gauss-Newton termine sur les segments.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from ..geometry import Pose, local_to_global
from ..matching import rigid_transform
from ..segments import Segment
from .common import LocResult, Timer, gauss_newton
from .grid import LikelihoodField, multiresolution


@dataclass(frozen=True)
class SegmentMap:
    a: np.ndarray    # (M, 2) premières extrémités
    b: np.ndarray    # (M, 2) secondes extrémités

    @staticmethod
    def from_segments(segments: list[Segment]) -> "SegmentMap":
        arr = np.array([s.as_tuple() for s in segments], dtype=float).reshape(-1, 4)
        return SegmentMap(arr[:, :2], arr[:, 2:])

    def nearest(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Pour chaque point : distance au segment le plus proche et pied (point le plus proche sur ce segment)."""
        ab = self.b - self.a                                            # (M, 2)
        l2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        ap = points[:, None, :] - self.a[None, :, :]                   # (N, M, 2)
        t = np.clip((ap * ab[None]).sum(axis=2) / l2[None], 0.0, 1.0)  # position du pied sur le segment
        feet = self.a[None] + t[..., None] * ab[None]                  # (N, M, 2)
        d = np.linalg.norm(points[:, None, :] - feet, axis=2)
        k = d.argmin(axis=1)
        idx = np.arange(len(points))
        return d[idx, k], feet[idx, k]

    def residual(self, points: np.ndarray):
        d, feet = self.nearest(points)
        grad = (points - feet) / np.maximum(d, 1e-9)[:, None]
        return d, grad, d > 1e-9

    def sample(self, step: float = 0.02) -> np.ndarray:
        """Points régulièrement répartis sur les segments (pour la recherche grossière)."""
        out = []
        for a, b in zip(self.a, self.b):
            n = max(2, int(np.ceil(np.linalg.norm(b - a) / step)) + 1)
            out.append(a + np.linspace(0, 1, n)[:, None] * (b - a))
        return np.vstack(out)

    def score(self, points: np.ndarray, sigma: float = 0.05) -> float:
        d, _ = self.nearest(points)
        return float(np.exp(-d ** 2 / (2 * sigma ** 2)).mean())


def gauss_newton_method(points_local: np.ndarray, initial: Pose, segmap: SegmentMap) -> LocResult:
    with Timer() as tm:
        pose, n = gauss_newton(points_local, initial, segmap.residual)
        score = segmap.score(local_to_global(points_local, pose))
    return LocResult(pose, score, tm.elapsed, n, "vecteur · Gauss-Newton")


def direct(points_local: np.ndarray, initial: Pose, segmap: SegmentMap,
           schedule: tuple[float, ...] = (0.5, 0.25, 0.10), iterations: int = 12, keep_ratio: float = 0.8) -> LocResult:
    """Calcul direct itéré : appariement point → pied du segment, puis transformation exacte (Kabsch)."""
    with Timer() as tm:
        pose, n = initial, 0
        for d_max in schedule:
            for _ in range(iterations):
                n += 1
                pts = local_to_global(points_local, pose)
                d, feet = segmap.nearest(pts)
                ok = d < d_max
                if ok.sum() < 10:
                    break
                ok &= d <= np.quantile(d[ok], keep_ratio)
                dth, t = rigid_transform(pts[ok], feet[ok])
                c, s = np.cos(dth), np.sin(dth)
                pose = Pose(float(c * pose.x - s * pose.y + t[0]), float(s * pose.x + c * pose.y + t[1]),
                            pose.theta_deg + float(np.degrees(dth)))
                if abs(dth) < 1e-6 and np.hypot(*t) < 1e-5:
                    break
        score = segmap.score(local_to_global(points_local, pose))
    return LocResult(pose, score, tm.elapsed, n, "vecteur · calcul direct")


def multiresolution_gauss_newton(points_local: np.ndarray, initial: Pose, segmap: SegmentMap,
                                 field: LikelihoodField | None = None, pyramid: dict | None = None) -> LocResult:
    """Recherche grossière robuste sur les segments rastérisés, puis Gauss-Newton exact sur les segments."""
    field = field or LikelihoodField.from_points(segmap.sample())
    coarse = multiresolution(points_local, initial, field, pyramid=pyramid)
    fine = gauss_newton_method(points_local, coarse.pose, segmap)
    return replace(fine, time_s=coarse.time_s + fine.time_s, evaluations=coarse.evaluations + fine.evaluations,
                   method="vecteur · multi-résolution + Gauss-Newton")
