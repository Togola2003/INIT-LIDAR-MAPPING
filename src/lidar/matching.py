"""Recalage de scans (scan matching) par ICP, Iterative Closest Point, version point à point.

Idée : on part d'une pose approximative (mesurée au mètre). À chaque itération :
  1. on place les points du scan dans la carte avec la pose courante ;
  2. on associe chaque point au point de référence le plus proche (s'il est à moins de d_max) ;
  3. on garde les meilleures paires (les 80 % les plus proches : robustesse aux zones non vues) ;
  4. on calcule la rotation + translation qui superpose au mieux ces paires (méthode de Kabsch / SVD) ;
  5. on corrige la pose, et on recommence avec un d_max de plus en plus petit (grossier -> fin).

Utilisé au Jour 1 pour corriger les poses avant de construire les cartes, et au Jour 2 pour la
localisation « entre scans ».
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import Pose, local_to_global

CHUNK = 256   # calcul des distances par paquets de lignes, pour limiter la mémoire


@dataclass(frozen=True)
class IcpResult:
    pose: Pose
    rms_m: float           # erreur quadratique moyenne des paires gardées
    inlier_ratio: float    # part des points du scan qui ont trouvé un voisin à moins de d_max final
    iterations: int
    converged: bool


def voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
    """Un seul point (la moyenne) par case de size × size : allège la référence sans la déformer."""
    if len(points) == 0 or size <= 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    sums = np.zeros((inverse.max() + 1, 2))
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse)[:, None]
    return sums / counts


def nearest(src: np.ndarray, ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pour chaque point de src : indice et distance du point de ref le plus proche (force brute)."""
    idx = np.empty(len(src), dtype=int)
    dist = np.empty(len(src))
    for k in range(0, len(src), CHUNK):
        block = src[k:k + CHUNK]
        d2 = ((block[:, None, :] - ref[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        idx[k:k + CHUNK] = j
        dist[k:k + CHUNK] = np.sqrt(d2[np.arange(len(block)), j])
    return idx, dist


def rigid_transform(a: np.ndarray, b: np.ndarray) -> tuple[float, np.ndarray]:
    """Rotation θ (rad) et translation t qui minimisent Σ |R·a + t − b|² (Kabsch 2D)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    h = (a - ca).T @ (b - cb)
    theta = np.arctan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = np.cos(theta), np.sin(theta)
    t = cb - np.array([[c, -s], [s, c]]) @ ca
    return float(theta), t


def icp(local_points: np.ndarray, initial: Pose, reference: np.ndarray,
        max_dist_schedule: tuple[float, ...] = (0.6, 0.35, 0.2, 0.1),
        iterations_per_stage: int = 15, keep_ratio: float = 0.8, tol: float = 1e-4) -> IcpResult:
    """Recale un scan (points dans le repère lidar) sur une référence (points dans le repère carte)."""
    if len(local_points) < 10 or len(reference) < 10:
        return IcpResult(initial, float("inf"), 0.0, 0, False)
    pose = initial
    total = 0
    rms, ratio = float("inf"), 0.0
    for d_max in max_dist_schedule:
        for _ in range(iterations_per_stage):
            total += 1
            src = local_to_global(local_points, pose)
            j, d = nearest(src, reference)
            ok = d < d_max
            if ok.sum() < 10:
                return IcpResult(pose, rms, ratio, total, False)
            cutoff = np.quantile(d[ok], keep_ratio)
            ok &= d <= cutoff
            dtheta, t = rigid_transform(src[ok], reference[j[ok]])
            c, s = np.cos(dtheta), np.sin(dtheta)
            x, y = c * pose.x - s * pose.y + t[0], s * pose.x + c * pose.y + t[1]
            pose = Pose(float(x), float(y), pose.theta_deg + float(np.degrees(dtheta)))
            rms = float(np.sqrt((d[ok] ** 2).mean()))
            ratio = float((d < d_max).mean())
            if abs(dtheta) < tol and np.hypot(*t) < tol:
                break
    return IcpResult(pose, rms, ratio, total, True)


def align_sequence(scans_local: list[np.ndarray], poses: list[Pose], anchor: int = 0,
                   voxel: float = 0.03) -> list[tuple[Pose, IcpResult | None]]:
    """Recale les scans un par un sur la carte formée par les scans DÉJÀ recalés.

    Le scan 'anchor' sert de référence fixe (sa pose est gardée). Les autres sont traités du plus
    proche au plus éloigné de lui, pour que chaque scan recouvre au mieux la carte déjà construite.
    """
    order = sorted(range(len(poses)), key=lambda k: np.hypot(poses[k].x - poses[anchor].x,
                                                             poses[k].y - poses[anchor].y))
    out: list[tuple[Pose, IcpResult | None]] = [(p, None) for p in poses]
    reference = voxel_downsample(local_to_global(scans_local[anchor], poses[anchor]), voxel)
    for k in order:
        if k == anchor:
            continue
        res = icp(scans_local[k], poses[k], reference)
        out[k] = (res.pose, res)
        reference = voxel_downsample(np.vstack([reference, local_to_global(scans_local[k], res.pose)]), voxel)
    return out
