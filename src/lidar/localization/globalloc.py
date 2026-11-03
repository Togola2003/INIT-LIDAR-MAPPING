"""Localisation globale : retrouver la pose d'un scan SANS pose de départ (« où suis-je ? »).

Les méthodes locales cherchent autour d'une pose de départ (± 1 m, ± 35°). Ici on cherche dans toute
la salle, en réduisant intelligemment les poses à tester (idée reprise de l'application ld06-slam
d'un camarade de TP, réécrite en Python avec numpy) :
  1. positions candidates : les cases LIBRES de la grid map, tous les 20 cm, à plus de 10 cm d'un mur ;
  2. orientations candidates : dans une salle, les murs sont presque tous parallèles ou
     perpendiculaires. La direction dominante des murs du scan et celle de la carte donnent
     l'orientation à 90° près : 4 × 9 orientations au lieu de 360. Sans direction nette : tous les 2° ;
  3. score de chaque pose sur un champ de vraisemblance lissé (cases de 10 cm, σ = 15 cm, pour ne
     pas rater la bonne pose entre deux pas), avec 150 points du scan ;
  4. les 6 meilleures poses distinctes sont affinées (multi-résolution locale + gradient) et classées
     par la part des points du scan à moins de 5 cm d'un mur ;
     Si la meilleure a moins de 60 % de ses points sur les murs, on recommence avec TOUTES les
     orientations (tous les 3°) : plus lent (≈ 2 s) mais on ne rate plus une orientation inattendue ;
  5. confiance : on compare la 1re et la 2e candidate sur les seuls points qui les départagent.
     Si la 2e en explique presque autant, la salle est ambiguë vue d'ici (symétrie, couloir) : on le dit.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from ..geometry import Pose, local_to_global
from ..gridmap import OccupancyGrid
from ..matching import local_normals
from .common import Timer, subsample
from .grid import LikelihoodField, build_pyramid, gradient, multiresolution

STEP_M = 0.20
CLEARANCE_M = 0.10
COARSE_SIGMA = 0.15
SEARCH_POINTS = 150
CANDIDATES = 6
SEPARATION_M, SEPARATION_DEG = 0.5, 10.0
SPREAD_DEG = 4.0
ON_MAP_M = 0.05
AMBIGUITY = 0.5
SURE = 0.6            # en dessous (part des points sur les murs), on refait la recherche avec toutes les orientations
FULL_STEP_DEG = 3.0


@dataclass(frozen=True)
class GlobalResult:
    pose: Pose
    inliers: float                   # part des points du scan à moins de 5 cm d'un mur
    candidates: list                 # [(pose, inliers)] distinctes, la meilleure d'abord
    ambiguous: bool
    evidence: tuple[int, int]        # points qui départagent : (pour la 1re, pour la 2e)
    positions: int
    angles: int
    mode: str                        # « murs » (orientations tirées des murs) ou « tous »
    time_s: float


def dominant_direction(directions_deg: np.ndarray, weights: np.ndarray) -> tuple[float, float]:
    """Direction dominante modulo 90° (histogramme pondéré, 1°) et part du poids à ± 5° d'elle."""
    if not len(directions_deg) or weights.sum() <= 0:
        return 0.0, 0.0
    d = np.mod(directions_deg, 90.0)
    hist = np.bincount(np.floor(d).astype(int) % 90, weights, minlength=90)
    smooth = sum(np.roll(hist, k) for k in (-2, -1, 0, 1, 2))
    peak = float(np.argmax(smooth)) + 0.5
    gap = np.abs((d - peak + 45) % 90 - 45)
    return peak, float(weights[gap <= 5].sum() / weights.sum())


def scan_direction(points_local: np.ndarray) -> tuple[float, float]:
    """Direction des murs vus par le scan : orientation des parties droites (normales locales)."""
    normals, flat = local_normals(points_local)
    if flat.sum() < 10:
        return 0.0, 0.0
    ang = np.degrees(np.arctan2(normals[flat, 1], normals[flat, 0])) + 90.0
    return dominant_direction(ang, np.ones(int(flat.sum())))


def map_direction(segments) -> float | None:
    if not segments:
        return None
    ang = np.array([np.degrees(np.arctan2(s.y2 - s.y1, s.x2 - s.x1)) for s in segments])
    return dominant_direction(ang, np.array([s.length for s in segments]))[0]


def candidate_angles(points_local: np.ndarray, map_dir: float | None, min_support: float = 0.25):
    scan_dir, support = scan_direction(points_local)
    if map_dir is not None and support >= min_support:
        base = map_dir - scan_dir
        return np.array([base + 90 * k + d for k in range(4) for d in np.arange(-SPREAD_DEG, SPREAD_DEG + 0.5, 1.0)]), "murs"
    return np.arange(-180.0, 180.0, 2.0), "tous"


def free_positions(grid: OccupancyGrid, field: LikelihoodField, step: float = STEP_M,
                   clearance: float = CLEARANCE_M) -> np.ndarray:
    every = max(1, int(round(step / grid.resolution)))
    free = grid.free_mask() & (field.dist >= clearance)
    j, i = np.nonzero(free[::every, ::every])
    i, j = i * every, j * every
    return np.column_stack((grid.origin[0] + (i + 0.5) * grid.resolution, grid.origin[1] + (j + 0.5) * grid.resolution))


def _distinct(cands: list[tuple[float, Pose]], n: int) -> list[tuple[float, Pose]]:
    out: list[tuple[float, Pose]] = []
    for sc, p in cands:
        if all(np.hypot(p.x - q.x, p.y - q.y) >= SEPARATION_M or
               abs((p.theta_deg - q.theta_deg + 180) % 360 - 180) >= SEPARATION_DEG for _, q in out):
            out.append((sc, p))
            if len(out) == n:
                break
    return out


def _on_map(field: LikelihoodField, points_local: np.ndarray, pose: Pose) -> np.ndarray:
    d, _, valid = field.residual(local_to_global(points_local, pose))
    return valid & (d <= ON_MAP_M)


def global_localize(points_local: np.ndarray, grid: OccupancyGrid, segments=(), field: LikelihoodField | None = None,
                    pyramid: dict | None = None) -> GlobalResult:
    with Timer() as tm:
        field = field or LikelihoodField.from_grid(grid)
        pyramid = pyramid or build_pyramid(field)
        coarse = replace(field.coarser(2), sigma=COARSE_SIGMA)
        positions = free_positions(grid, field)
        angles, mode = candidate_angles(points_local, map_direction(list(segments)))
        refined = _refine(points_local, field, pyramid, coarse, positions, angles, mode)
        n_angles = len(angles)
        if mode == "murs" and max(r[0] for r in refined) < SURE:
            # les murs n'ont pas suffi à trouver une pose nette : on teste TOUTES les orientations
            all_angles = np.arange(-180.0, 180.0, FULL_STEP_DEG)
            refined += _refine(points_local, field, pyramid, coarse, positions, all_angles, "tous")
            n_angles += len(all_angles)
            mode = "murs + tous"
        refined.sort(key=lambda r: -r[0])
        distinct = _distinct(refined, CANDIDATES)
        evidence, ambiguous = (0, 0), False
        if len(distinct) > 1:
            a = _on_map(field, points_local, distinct[0][1])
            b = _on_map(field, points_local, distinct[1][1])
            evidence = (int((a & ~b).sum()), int((b & ~a).sum()))
            ambiguous = evidence[0] < 8 or evidence[1] >= AMBIGUITY * max(evidence[0], 1)
    best_in, best = distinct[0]
    return GlobalResult(best, best_in, [(p, i) for i, p in distinct], ambiguous, evidence, len(positions),
                        n_angles, mode, tm.elapsed)


def _refine(points_local, field, pyramid, coarse, positions, angles, mode) -> list[tuple[float, Pose]]:
    """Note toutes les (position, orientation) sur le champ lissé, puis affine les meilleures distinctes."""
    lik = coarse.likelihood
    pts = subsample(points_local, SEARCH_POINTS)
    ny, nx = lik.shape
    scores = np.zeros((len(angles), len(positions)))
    for a, th in enumerate(np.radians(angles)):
        c, s = np.cos(th), np.sin(th)
        rot = pts @ np.array([[c, s], [-s, c]])
        world = positions[:, None, :] + rot[None, :, :]                       # (P, N, 2)
        ij = np.floor((world - np.array(coarse.origin)) / coarse.resolution).astype(int)
        ok = (ij[..., 0] >= 0) & (ij[..., 0] < nx) & (ij[..., 1] >= 0) & (ij[..., 1] < ny)
        val = np.where(ok, lik[np.clip(ij[..., 1], 0, ny - 1), np.clip(ij[..., 0], 0, nx - 1)], 0.0)
        scores[a] = val.mean(axis=1)
    flat = scores.ravel()
    top = np.argsort(flat)[::-1][:3000]
    ranked = [(float(flat[k]), Pose(float(positions[k % len(positions), 0]), float(positions[k % len(positions), 1]),
                                    float(angles[k // len(positions)]))) for k in top]
    refined = []
    for _, p in _distinct(ranked, CANDIDATES):
        m = multiresolution(points_local, p, field, window_m=1.5 * STEP_M,
                            window_deg=SPREAD_DEG + 2 if mode == "murs" else FULL_STEP_DEG + 2, pyramid=pyramid)
        g = gradient(points_local, m.pose, field)
        best = g.pose if _on_map(field, points_local, g.pose).mean() >= _on_map(field, points_local, m.pose).mean() else m.pose
        refined.append((float(_on_map(field, points_local, best).mean()), best))
    return refined
