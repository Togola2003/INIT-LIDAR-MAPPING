"""Localisation sur grid map (Jour 2, référence n°1).

1. La carte est transformée UNE fois en « champ de vraisemblance » : pour chaque case, la distance d
   au mur (case occupée) le plus proche, et la valeur exp(−d² / 2σ²) : 1 sur un mur, 0 loin d'un mur.
2. Une pose candidate est notée en plaçant les points du scan : score = moyenne des valeurs des cases
   touchées (1 = tous les points sur des murs).
3. On cherche la pose de meilleur score :
   - exhaustive : toutes les poses d'une fenêtre autour de la pose initiale (pas 1 case, 1°) ;
   - multi-résolution : la même chose sur une carte grossière, puis affinage local sur des cartes
     de plus en plus fines (on ne garde que les meilleurs candidats d'un niveau à l'autre) ;
   - gradient : Gauss-Newton sur la distance aux murs, depuis la pose initiale (rapide et précis,
     mais il faut partir assez près) ;
   - multi-résolution + gradient : le meilleur des deux (robuste puis précis).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.ndimage import distance_transform_edt

from ..geometry import Pose, local_to_global
from ..gridmap import OccupancyGrid
from .common import LocResult, Timer, gauss_newton, subsample

MAX_DIST = 1.0        # au-delà, la distance au mur est plafonnée (zone vide ou hors carte)
SEARCH_POINTS = 180   # points utilisés pendant les recherches (le gradient final les utilise tous)


@dataclass(frozen=True)
class LikelihoodField:
    occupied: np.ndarray      # (ny, nx) bool, indexé [j, i]
    dist: np.ndarray          # distance au mur le plus proche (m)
    resolution: float
    origin: tuple[float, float]
    sigma: float

    @staticmethod
    def from_mask(occupied: np.ndarray, resolution: float, origin, sigma: float | None = None) -> "LikelihoodField":
        if occupied.any():
            dist = distance_transform_edt(~occupied) * resolution
        else:
            dist = np.full(occupied.shape, MAX_DIST)
        return LikelihoodField(occupied, np.minimum(dist, MAX_DIST), resolution, (float(origin[0]), float(origin[1])),
                               sigma or max(resolution, 0.05))

    @staticmethod
    def from_grid(grid: OccupancyGrid, sigma: float | None = None) -> "LikelihoodField":
        return LikelihoodField.from_mask(grid.occupied_mask(), grid.resolution, grid.origin, sigma)

    @staticmethod
    def from_points(points: np.ndarray, resolution: float = 0.05, margin: float = 1.5) -> "LikelihoodField":
        """Champ construit directement à partir de points (scan de référence, segments échantillonnés)."""
        lo = points.min(axis=0) - margin
        n = np.ceil((points.max(axis=0) + margin - lo) / resolution).astype(int)
        occ = np.zeros((n[1], n[0]), dtype=bool)
        ij = np.floor((points - lo) / resolution).astype(int)
        occ[ij[:, 1], ij[:, 0]] = True
        return LikelihoodField.from_mask(occ, resolution, lo)

    def coarser(self, factor: int) -> "LikelihoodField":
        """Même carte avec des cases factor fois plus grandes (une case est un mur si une de ses
        sous-cases l'est) : sert à la multi-résolution."""
        if factor == 1:
            return self
        ny, nx = self.occupied.shape
        py, px = (-ny) % factor, (-nx) % factor
        occ = np.pad(self.occupied, ((0, py), (0, px)))
        occ = occ.reshape(occ.shape[0] // factor, factor, occ.shape[1] // factor, factor).any(axis=(1, 3))
        return LikelihoodField.from_mask(occ, self.resolution * factor, self.origin, self.sigma * factor)

    @property
    def likelihood(self) -> np.ndarray:
        return np.exp(-self.dist ** 2 / (2 * self.sigma ** 2))

    def score(self, points: np.ndarray) -> float:
        """Moyenne de la vraisemblance des cases touchées (0 hors carte)."""
        i, j = self._cells(points)
        ny, nx = self.dist.shape
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        val = np.zeros(len(points))
        val[ok] = self.likelihood[j[ok], i[ok]]
        return float(val.mean()) if len(val) else 0.0

    def _cells(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        f = (points - np.array(self.origin)) / self.resolution
        return np.floor(f[:, 0]).astype(int), np.floor(f[:, 1]).astype(int)

    def residual(self, points: np.ndarray):
        """Distance au mur (interpolée) et son gradient, pour la méthode de Gauss-Newton."""
        f = (points - np.array(self.origin)) / self.resolution - 0.5      # coordonnées en cases (centres)
        ny, nx = self.dist.shape
        i0, j0 = np.floor(f[:, 0]).astype(int), np.floor(f[:, 1]).astype(int)
        valid = (i0 >= 0) & (i0 < nx - 1) & (j0 >= 0) & (j0 < ny - 1)
        i0, j0 = np.clip(i0, 0, nx - 2), np.clip(j0, 0, ny - 2)
        fx, fy = f[:, 0] - i0, f[:, 1] - j0
        d00, d10 = self.dist[j0, i0], self.dist[j0, i0 + 1]
        d01, d11 = self.dist[j0 + 1, i0], self.dist[j0 + 1, i0 + 1]
        r = (d00 * (1 - fx) + d10 * fx) * (1 - fy) + (d01 * (1 - fx) + d11 * fx) * fy
        gx = ((d10 - d00) * (1 - fy) + (d11 - d01) * fy) / self.resolution
        gy = ((d01 - d00) * (1 - fx) + (d11 - d10) * fx) / self.resolution
        return r, np.column_stack((gx, gy)), valid & (r < MAX_DIST)


# ---------------------------------------------------------------------- recherche exhaustive
def _search(field: LikelihoodField, points: np.ndarray, center: Pose, half_cells: int, half_deg: float,
            step_deg: float, keep: int = 1) -> tuple[list[tuple[float, Pose]], int]:
    """Teste toutes les poses (x0 ± half_cells cases, y0 ± …, θ0 ± half_deg) et rend les meilleures.

    Astuce de calcul : pour un angle donné, déplacer le scan d'un nombre ENTIER de cases revient à
    décaler les indices des cases : on calcule les indices une fois par angle, puis toutes les
    translations d'un coup (numpy), sans reprojeter les points."""
    lik = np.pad(field.likelihood, half_cells + 1)               # bord de zéros : rien hors carte
    offsets = np.arange(-half_cells, half_cells + 1)
    angles = center.theta_deg + np.arange(-half_deg, half_deg + 1e-9, step_deg)
    best: list[tuple[float, Pose]] = []
    for a in angles:
        t = np.radians(a)
        c, s = np.cos(t), np.sin(t)
        world = points @ np.array([[c, s], [-s, c]]) + np.array([center.x, center.y])
        i, j = field._cells(world)
        i, j = i + half_cells + 1, j + half_cells + 1
        ny, nx = lik.shape
        ok = (i >= half_cells) & (i < nx - half_cells - 1) & (j >= half_cells) & (j < ny - half_cells - 1)
        if not ok.any():
            continue
        sc = lik[j[ok, None, None] + offsets[None, :, None], i[ok, None, None] + offsets[None, None, :]].sum(axis=0)
        sc = sc / len(points)
        for flat in np.argsort(sc, axis=None)[-keep:]:
            ky, kx = np.unravel_index(flat, sc.shape)
            pose = Pose(center.x + offsets[kx] * field.resolution, center.y + offsets[ky] * field.resolution, float(a))
            best.append((float(sc[ky, kx]), pose))
    best.sort(key=lambda b: -b[0])
    return best[:keep], len(angles) * len(offsets) ** 2


def exhaustive(points_local: np.ndarray, initial: Pose, field: LikelihoodField, window_m: float = 1.1,
               window_deg: float = 35.0, step_deg: float = 1.0) -> LocResult:
    """Recherche exhaustive : pas = 1 case (5 cm) et step_deg, sur ± window_m et ± window_deg."""
    with Timer() as tm:
        pts = subsample(points_local, SEARCH_POINTS)
        found, n = _search(field, pts, initial, int(round(window_m / field.resolution)), window_deg, step_deg)
        score, pose = found[0] if found else (0.0, initial)
    return LocResult(pose, score, tm.elapsed, n, "grid · exhaustive")


def multiresolution(points_local: np.ndarray, initial: Pose, field: LikelihoodField, window_m: float = 1.1,
                    window_deg: float = 35.0, levels: tuple[int, ...] = (4, 2, 1), keep: int = 4,
                    pyramid: dict | None = None) -> LocResult:
    """Multi-résolution : recherche complète sur la carte la plus grossière (cases de 20 cm, pas de 4°),
    puis, pour les `keep` meilleurs candidats, recherche locale sur la carte 2× plus fine, etc."""
    with Timer() as tm:
        pts = subsample(points_local, SEARCH_POINTS)
        pyramid = pyramid if pyramid is not None else build_pyramid(field, levels)
        step_deg = 4.0
        coarse = pyramid[levels[0]]
        cands, n = _search(coarse, pts, initial, int(np.ceil(window_m / coarse.resolution)), window_deg, step_deg, keep)
        for f in levels[1:]:
            fine = pyramid[f]
            nxt = []
            for _, pose in cands:
                found, k = _search(fine, pts, pose, 2, step_deg, step_deg / 2, keep=2)
                nxt += found
                n += k
            step_deg /= 2
            cands = sorted(nxt, key=lambda b: -b[0])[:keep]
        score, pose = cands[0] if cands else (0.0, initial)
    return LocResult(pose, score, tm.elapsed, n, "grid · multi-résolution")


def build_pyramid(field: LikelihoodField, levels: tuple[int, ...] = (4, 2, 1)) -> dict[int, LikelihoodField]:
    return {f: field.coarser(f) for f in levels}


def gradient(points_local: np.ndarray, initial: Pose, field: LikelihoodField) -> LocResult:
    """Gauss-Newton sur la distance aux murs, depuis la pose initiale."""
    with Timer() as tm:
        pose, n = gauss_newton(points_local, initial, field.residual)
        score = field.score(local_to_global(points_local, pose))
    return LocResult(pose, score, tm.elapsed, n, "grid · gradient")


def multiresolution_gradient(points_local: np.ndarray, initial: Pose, field: LikelihoodField,
                             pyramid: dict | None = None, **kw) -> LocResult:
    """Multi-résolution (robuste à une mauvaise pose initiale) puis gradient (précision < 1 case)."""
    coarse = multiresolution(points_local, initial, field, pyramid=pyramid, **kw)
    fine = gradient(points_local, coarse.pose, field)
    return replace(fine, time_s=coarse.time_s + fine.time_s, evaluations=coarse.evaluations + fine.evaluations,
                   method="grid · multi-résolution + gradient")
