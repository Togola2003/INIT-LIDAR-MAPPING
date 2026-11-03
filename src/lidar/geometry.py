"""Conversions de repères : polaire -> cartésien, repère lidar -> repère carte.

Conventions (identiques aux outils du module) :
- distances en mètres, angles en degrés dans les fichiers, radians dans les calculs ;
- angles positifs dans le sens antihoraire (sens trigonométrique), 0° vers +X ;
- pose = (x, y, theta_deg) du lidar dans le repère de la carte.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta_deg: float = 0.0

    @property
    def theta(self) -> float:
        return np.deg2rad(self.theta_deg)

    def moved(self, dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0) -> "Pose":
        return Pose(self.x + dx, self.y + dy, self.theta_deg + dtheta_deg)

    @staticmethod
    def parse(text: str) -> "Pose":
        """Lit une pose écrite 'x,y,theta_deg' (ex. '0.5,1.2,90')."""
        parts = [float(v) for v in text.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Pose attendue sous la forme x,y,theta_deg : {text!r}")
        return Pose(*parts)


def polar_to_cartesian(angles_deg: np.ndarray, ranges_m: np.ndarray) -> np.ndarray:
    """(θ, r) -> points (N, 2) dans le repère du lidar."""
    a = np.deg2rad(np.asarray(angles_deg, dtype=float))
    r = np.asarray(ranges_m, dtype=float)
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def local_to_global(points: np.ndarray, pose: Pose) -> np.ndarray:
    """P_global = R(θ) · P_local + T(x, y)."""
    c, s = np.cos(pose.theta), np.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    return np.asarray(points, dtype=float) @ rot.T + np.array([pose.x, pose.y])


def move_rigid(pose: Pose, dx: float = 0.0, dy: float = 0.0, dtheta_deg: float = 0.0,
               pivot: tuple[float, float] | None = None) -> Pose:
    """Déplace une pose comme un objet solide : rotation de dtheta autour de 'pivot' (par défaut la
    pose elle-même), puis translation (dx, dy). Sert à tourner plusieurs scans ENSEMBLE autour de
    leur centre commun : chaque scan tourne sur lui-même ET autour du pivot, le groupe reste rigide."""
    cx, cy = (pose.x, pose.y) if pivot is None else pivot
    a = np.deg2rad(dtheta_deg)
    c, s = np.cos(a), np.sin(a)
    x, y = pose.x - cx, pose.y - cy
    return Pose(float(cx + c * x - s * y + dx), float(cy + s * x + c * y + dy), pose.theta_deg + dtheta_deg)
