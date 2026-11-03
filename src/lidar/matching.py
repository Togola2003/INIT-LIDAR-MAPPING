"""Recalage de scans (scan matching) par ICP, Iterative Closest Point, version point à point.

Idée : on part d'une pose approximative (mesurée au mètre). À chaque itération :
  1. on place les points du scan dans la carte avec la pose courante ;
  2. on associe chaque point au point de référence le plus proche (s'il est à moins de d_max) ;
  3. on garde les meilleures paires (les 80 % les plus proches : robustesse aux zones non vues) ;
  4. on calcule la rotation + translation qui superpose au mieux ces paires (méthode de Kabsch / SVD) ;
  5. on corrige la pose, et on recommence avec un d_max de plus en plus petit (grossier -> fin).

Utilisé au Jour 1 pour corriger les poses avant de construire les cartes, et au Jour 2 pour la
localisation « entre scans ».

refine_global va plus loin : au lieu de recaler les scans un par un (chaque erreur se propage au
suivant), il corrige TOUTES les poses en même temps, en version point à droite (voir sa docstring).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.spatial import cKDTree

from .geometry import Pose, local_to_global


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


def nearest(src: np.ndarray, ref) -> tuple[np.ndarray, np.ndarray]:
    """Pour chaque point de src : indice et distance du point de ref le plus proche.

    ref est un tableau de points ou un cKDTree déjà construit (arbre k-d : la recherche coûte
    log(n) au lieu de n, ce qui rend l'ICP environ 50 fois plus rapide que la force brute)."""
    tree = ref if isinstance(ref, cKDTree) else cKDTree(ref)
    dist, idx = tree.query(src)
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
    tree = cKDTree(reference)
    pose = initial
    total = 0
    rms, ratio = float("inf"), 0.0
    for d_max in max_dist_schedule:
        for _ in range(iterations_per_stage):
            total += 1
            src = local_to_global(local_points, pose)
            j, d = nearest(src, tree)
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


def agreement(local_points: np.ndarray, pose: Pose, reference: np.ndarray, d_max: float = 0.3) -> float:
    """Accord d'un scan avec une référence : part de ses points à moins de d_max d'un point de référence,
    pondérée par leur proximité (1 = parfait). Sert à vérifier qu'un recalage améliore vraiment les choses."""
    _, d = nearest(local_to_global(local_points, pose), reference)
    return float(np.clip(1 - d / d_max, 0, 1).mean())


def align_sequence(scans_local: list[np.ndarray], poses: list[Pose], anchor: int = 0,
                   voxel: float = 0.03) -> list[tuple[Pose, IcpResult | None]]:
    """Recale les scans un par un sur la carte formée par les scans DÉJÀ recalés.

    Le scan 'anchor' sert de référence fixe (sa pose est gardée). Les autres sont traités du plus
    proche au plus éloigné de lui, pour que chaque scan recouvre au mieux la carte déjà construite.
    Sécurité : la correction n'est gardée que si elle AMÉLIORE l'accord avec la carte (sinon l'ICP
    a pu « glisser » le long d'un mur droit, qui ne contraint pas la position dans sa direction).
    Un résultat refusé est rendu avec converged=False et la pose d'origine.
    """
    order = sorted(range(len(poses)), key=lambda k: np.hypot(poses[k].x - poses[anchor].x,
                                                             poses[k].y - poses[anchor].y))
    out: list[tuple[Pose, IcpResult | None]] = [(p, None) for p in poses]
    reference = voxel_downsample(local_to_global(scans_local[anchor], poses[anchor]), voxel)
    for k in order:
        if k == anchor:
            continue
        res = icp(scans_local[k], poses[k], reference)
        if agreement(scans_local[k], res.pose, reference) <= agreement(scans_local[k], poses[k], reference):
            res = IcpResult(poses[k], res.rms_m, res.inlier_ratio, res.iterations, False)
        out[k] = (res.pose, res)
        reference = voxel_downsample(np.vstack([reference, local_to_global(scans_local[k], res.pose)]), voxel)
    return out


# ====================================================================== recalage global
@dataclass(frozen=True)
class GlobalResult:
    poses: list[Pose]
    iterations: int
    gap_before_m: list[float]   # écart médian de chaque scan aux autres, avant
    gap_after_m: list[float]    # ... et après

    @property
    def median_before(self) -> float:
        return float(np.median(self.gap_before_m))

    @property
    def median_after(self) -> float:
        return float(np.median(self.gap_after_m))


def consistency(scans_local: list[np.ndarray], poses: list[Pose], voxel: float = 0.02) -> list[float]:
    """Pour chaque scan : écart médian (m) entre ses points et ceux de tous les AUTRES scans."""
    clouds = [local_to_global(p, q) for p, q in zip(scans_local, poses)]
    out = []
    for k, pts in enumerate(clouds):
        others = [c for j, c in enumerate(clouds) if j != k and len(c)]
        if not len(pts) or not others:
            out.append(float("nan"))
            continue
        out.append(float(np.median(nearest(pts, voxel_downsample(np.vstack(others), voxel))[1])))
    return out


def local_normals(points: np.ndarray, k: int = 7, max_flat: float = 0.12) -> tuple[np.ndarray, np.ndarray]:
    """Normale de chaque point (ACP sur ses k voisins) + masque des points situés sur une partie DROITE.

    max_flat : rapport petite / grande variance au-delà duquel le voisinage n'est pas une droite
    (coin, pied de chaise) ; ces points ne servent pas de cible."""
    n = len(points)
    if n < k:
        return np.zeros((n, 2)), np.zeros(n, dtype=bool)
    _, idx = cKDTree(points).query(points, k=k)
    nb = points[idx] - points[idx].mean(axis=1, keepdims=True)          # (n, k, 2)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    val, vec = np.linalg.eigh(cov)                                       # valeurs croissantes
    normals = vec[:, :, 0]                                               # direction de plus petite variance
    flat = val[:, 0] / np.maximum(val[:, 1], 1e-12)
    return normals, flat < max_flat


def _perp(v: np.ndarray) -> np.ndarray:
    return np.column_stack((-v[:, 1], v[:, 0]))


def refine_global(scans_local: list[np.ndarray], poses: list[Pose], anchor: int = 0, voxel: float = 0.03,
                  schedule: tuple[float, ...] = (0.40, 0.25, 0.15, 0.08), iterations_per_stage: int = 4,
                  sigma_point_m: float = 0.03, sigma_prior_m: float = 0.5, sigma_prior_deg: float = 8.0,
                  robust_m: float = 0.05) -> GlobalResult:
    """Corrige TOUTES les poses ensemble (ICP multi-scans, point à droite).

    1. chaque scan est allégé (une case de 3 cm = un point) et on calcule la normale locale des murs ;
    2. pour chaque paire de scans (i, j), chaque point de i est associé au point le plus proche de j ;
       l'erreur est la distance du point à la DROITE du mur de j (n·(a − b)), pas au point : un point
       peut glisser le long d'un mur sans pénalité, ce qui évite de « tirer » vers un point mal placé ;
    3. on résout un seul système de moindres carrés (Gauss-Newton) pour les 3 × N inconnues
       (x, y, θ de chaque scan), le scan ancre restant fixe ;
       - pondération robuste (Cauchy) : une paire très éloignée (porte, personne) compte peu ;
       - rappel faible vers la pose mesurée en salle (σ = 0,5 m, 8°) : un scan peu contraint
         (long couloir, faible recouvrement) reste près de sa pose au lieu de dériver ;
    4. on recommence avec un seuil d'association de plus en plus petit.
    Avantage sur align_sequence : toutes les contraintes agissent en même temps, l'erreur ne
    s'accumule pas d'un scan au suivant.
    """
    n = len(poses)
    gap_before = consistency(scans_local, poses)
    if n < 2:
        return GlobalResult(list(poses), 0, gap_before, gap_before)
    clouds = [voxel_downsample(p, voxel) for p in scans_local]
    normals, flat = zip(*(local_normals(c) for c in clouds))
    xy = np.array([[p.x, p.y] for p in poses], dtype=float)
    th = np.radians([p.theta_deg for p in poses])
    xy0, th0 = xy.copy(), th.copy()
    w_pt, w_xy, w_th = 1 / sigma_point_m ** 2, 1 / sigma_prior_m ** 2, 1 / np.radians(sigma_prior_deg) ** 2
    total = 0
    for d_max in schedule:
        for _ in range(iterations_per_stage):
            total += 1
            rot = [np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]) for t in th]
            glob = [c @ r.T + c0 for c, r, c0 in zip(clouds, rot, xy)]
            gnorm = [nm @ r.T for nm, r in zip(normals, rot)]
            trees = [cKDTree(g[f]) if f.sum() >= 10 else None for g, f in zip(glob, flat)]
            H = np.zeros((3 * n, 3 * n))
            g = np.zeros(3 * n)
            for j in range(n):
                if trees[j] is None:
                    continue
                tgt, tn = glob[j][flat[j]], gnorm[j][flat[j]]
                for i in range(n):
                    if i == j:
                        continue
                    d, idx = trees[j].query(glob[i], distance_upper_bound=d_max)
                    ok = np.isfinite(d)
                    if ok.sum() < 15:
                        continue
                    a, b, nv = glob[i][ok], tgt[idx[ok]], tn[idx[ok]]
                    r = np.einsum("ij,ij->i", nv, a - b)
                    w = w_pt / (1 + (r / robust_m) ** 2)
                    ai = np.column_stack((nv, np.einsum("ij,ij->i", nv, _perp(a - xy[i]))))
                    aj = -np.column_stack((nv, np.einsum("ij,ij->i", nv, _perp(b - xy[j]))))
                    si, sj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
                    H[si, si] += ai.T @ (w[:, None] * ai)
                    H[sj, sj] += aj.T @ (w[:, None] * aj)
                    H[si, sj] += ai.T @ (w[:, None] * aj)
                    H[sj, si] += aj.T @ (w[:, None] * ai)
                    g[si] += ai.T @ (w * r)
                    g[sj] += aj.T @ (w * r)
            for k in range(n):                       # rappel vers la pose mesurée
                H[3 * k, 3 * k] += w_xy
                H[3 * k + 1, 3 * k + 1] += w_xy
                H[3 * k + 2, 3 * k + 2] += w_th
                g[3 * k:3 * k + 2] += w_xy * (xy[k] - xy0[k])
                g[3 * k + 2] += w_th * (th[k] - th0[k])
            free = [v for v in range(3 * n) if v // 3 != anchor]
            step = np.zeros(3 * n)
            step[free] = np.linalg.solve(H[np.ix_(free, free)] + 1e-9 * np.eye(len(free)), -g[free])
            step = step.reshape(n, 3)
            xy, th = xy + step[:, :2], th + step[:, 2]
            if np.abs(step[:, :2]).max() < 1e-4 and np.abs(step[:, 2]).max() < 1e-4:
                break
    out = [Pose(float(x), float(y), float(np.degrees(t))) for (x, y), t in zip(xy, th)]
    out[anchor] = poses[anchor]
    return GlobalResult(out, total, gap_before, consistency(scans_local, out))


def align_best(scans_local: list[np.ndarray], poses: list[Pose], anchor: int = 0) -> tuple[GlobalResult, str]:
    """Recalage « complet » : on essaie deux départs pour l'optimisation globale et on garde le meilleur.

    - départ A : les poses telles quelles (mesurées en salle ou déjà corrigées à la main) ;
    - départ B : les poses après ICP scan par scan (rattrape de grosses erreurs, mais peut faire
      tomber un scan dans un mauvais minimum, en glissant le long d'un mur).
    Le critère est l'écart médian entre scans après optimisation (consistency). Rendu : le
    résultat retenu et le nom du départ choisi.
    """
    direct = refine_global(scans_local, poses, anchor)
    seq = [p for p, _ in align_sequence(scans_local, poses, anchor)]
    via_icp = refine_global(scans_local, seq, anchor)
    via_icp = replace(via_icp, gap_before_m=direct.gap_before_m)     # « avant » = poses de départ réelles
    if via_icp.median_after < direct.median_after:
        return via_icp, "ICP scan par scan puis global"
    return direct, "global depuis les poses actuelles"
