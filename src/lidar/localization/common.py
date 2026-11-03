"""Briques communes à toutes les méthodes de localisation (Jour 2).

Problème : on connaît une référence (grid map, carte vectorielle ou scan de référence), un scan
(points dans le repère du lidar) et une pose approximative. On cherche la pose (x, y, θ) qui place
les points du scan sur la référence.

Toutes les méthodes rendent un LocResult, pour pouvoir les comparer sur un pied d'égalité.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..geometry import Pose, local_to_global


@dataclass(frozen=True)
class LocResult:
    pose: Pose
    score: float          # qualité de l'alignement propre à la méthode (plus grand = mieux, 0 à 1)
    time_s: float
    evaluations: int      # nombre de poses évaluées (ou d'itérations pour les méthodes de gradient)
    method: str


class Timer:
    def __enter__(self) -> "Timer":
        self.start = time.perf_counter()
        return self

    def __exit__(self, *_) -> None:
        self.elapsed = time.perf_counter() - self.start


def subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    """Garde au plus max_points points, régulièrement répartis (déterministe : même résultat à chaque fois)."""
    if len(points) <= max_points:
        return points
    return points[np.linspace(0, len(points) - 1, max_points).astype(int)]


def perp(v: np.ndarray) -> np.ndarray:
    """Vecteur tourné de +90° : dérivée d'un point par rapport à l'angle de rotation."""
    return np.column_stack((-v[:, 1], v[:, 0]))


# residual(points_carte) -> (r, dr/dp, valide) : erreur de chaque point, son gradient (n, 2), masque
Residual = Callable[[np.ndarray], tuple[np.ndarray, np.ndarray, np.ndarray]]


def gauss_newton(points_local: np.ndarray, initial: Pose, residual: Residual,
                 schedule: tuple[float, ...] = (0.5, 0.25, 0.10), iterations: int = 12,
                 robust_m: float = 0.05, tol: float = 1e-5) -> tuple[Pose, int]:
    """Méthode de gradient (Gauss-Newton) : minimise Σ w·r² en corrigeant (x, y, θ) pas à pas.

    À chaque itération :
      1. on place les points avec la pose courante ;
      2. chaque point donne une erreur r (distance au mur) et la direction où elle augmente ;
      3. on linéarise : r(pose + δ) ≈ r + J·δ, avec J = [∂r/∂x, ∂r/∂y, ∂r/∂θ] ;
      4. on résout (JᵀWJ)·δ = −JᵀW·r : le petit déplacement qui fait le plus baisser l'erreur.
    Les points trop loin d'un mur (> d_max : zone non cartographiée, objet nouveau) sont ignorés ;
    d_max diminue d'un palier à l'autre (grossier puis fin). Pondération de Cauchy : un point
    aberrant pèse peu. Rend la pose et le nombre d'itérations.
    """
    pose, evals = initial, 0
    for d_max in schedule:
        for _ in range(iterations):
            evals += 1
            pts = local_to_global(points_local, pose)
            r, grad, valid = residual(pts)
            ok = valid & (np.abs(r) < d_max)
            if ok.sum() < 10:
                break
            c = np.array([pose.x, pose.y])
            g = grad[ok]
            jac = np.column_stack((g, np.einsum("ij,ij->i", g, perp(pts[ok] - c))))
            w = 1.0 / (1.0 + (r[ok] / robust_m) ** 2)
            h = jac.T @ (w[:, None] * jac) + 1e-6 * np.eye(3)
            step = -np.linalg.solve(h, jac.T @ (w * r[ok]))
            pose = Pose(pose.x + float(step[0]), pose.y + float(step[1]),
                        pose.theta_deg + float(np.degrees(step[2])))
            if np.hypot(step[0], step[1]) < tol and abs(step[2]) < tol:
                break
    return pose, evals


def pose_error(found: Pose, truth: Pose) -> tuple[float, float]:
    """Erreur de position (m) et d'angle (°, ramenée dans [0, 180])."""
    da = abs((found.theta_deg - truth.theta_deg + 180.0) % 360.0 - 180.0)
    return float(np.hypot(found.x - truth.x, found.y - truth.y)), float(da)
