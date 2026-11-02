"""Grid map 2D (carte d'occupation probabiliste) à partir de scans lidar.

Choix techniques (détaillés dans docs/jour1.md) :
- chaque cellule stocke le log-odds  L = ln(p / (1 - p))  : 0 = inconnu (p = 0,5),
  L < 0 = libre, L > 0 = occupé. La mise à jour bayésienne devient une simple addition ;
- cellules traversées par le faisceau : L += L_FREE (< 0), tracé avec Bresenham ;
- cellule d'impact : L += L_OCC (> 0) ;
- L est borné à [L_MIN, L_MAX] pour que la carte puisse encore changer (obstacles mobiles) ;
- export en uint8 (0 = libre, 127 = inconnu, 255 = occupé) comme suggéré dans le cours.

    python -m lidar.gridmap build data/simu/*.csv --res 0.05 --out out/grid.npz
    python -m lidar.gridmap update out/grid.npz data/simu/simu_03.csv --out out/grid.npz
    python -m lidar.gridmap show out/grid.npz
"""

from __future__ import annotations

import argparse
import glob
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from .scan import Scan

L_FREE = -0.4   # ≈ p(occupé | faisceau traverse) = 0,40
L_OCC = 0.85    # ≈ p(occupé | impact) = 0,70
L_MIN, L_MAX = -4.0, 4.0
# Seuils de classement : une seule observation (p = 0,40 ou 0,70) suffit à sortir de « inconnu »
OCC_THRESHOLD = 0.55
FREE_THRESHOLD = 0.45


@dataclass(frozen=True)
class OccupancyGrid:
    logodds: np.ndarray        # forme (ny, nx), indexée [j, i] (ligne = y, colonne = x)
    resolution: float          # taille d'une cellule (m)
    origin: tuple[float, float]  # coordonnées monde du coin bas-gauche de la cellule (0, 0)

    @property
    def shape(self) -> tuple[int, int]:
        return self.logodds.shape

    def world_to_cell(self, x, y) -> tuple[np.ndarray, np.ndarray]:
        i = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        j = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return i, j

    def cell_center(self, i: int, j: int) -> tuple[float, float]:
        return (self.origin[0] + (i + 0.5) * self.resolution, self.origin[1] + (j + 0.5) * self.resolution)

    def inside(self, i, j) -> np.ndarray:
        ny, nx = self.shape
        return (i >= 0) & (i < nx) & (j >= 0) & (j < ny)

    def extent(self) -> tuple[float, float, float, float]:
        ny, nx = self.shape
        x0, y0 = self.origin
        return (x0, x0 + nx * self.resolution, y0, y0 + ny * self.resolution)

    def probability(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    def to_uint8(self) -> np.ndarray:
        return np.round(self.probability() * 255).astype(np.uint8)

    def occupied_mask(self) -> np.ndarray:
        return self.probability() > OCC_THRESHOLD

    def free_mask(self) -> np.ndarray:
        return self.probability() < FREE_THRESHOLD


def create_grid(width_m: float, height_m: float, resolution: float, origin=(0.0, 0.0)) -> OccupancyGrid:
    """Grille vide, tout « inconnu » (L = 0). Ex. 2 m x 3 m à 0,1 m -> 30 lignes x 20 colonnes."""
    if resolution <= 0 or width_m <= 0 or height_m <= 0:
        raise ValueError("dimensions et résolution doivent être > 0")
    nx = int(np.ceil(width_m / resolution))
    ny = int(np.ceil(height_m / resolution))
    return OccupancyGrid(np.zeros((ny, nx)), resolution, (float(origin[0]), float(origin[1])))


def bresenham(i0: int, j0: int, i1: int, j1: int) -> list[tuple[int, int]]:
    """Cellules traversées de (i0, j0) à (i1, j1) inclus (algorithme de Bresenham entier)."""
    cells = []
    di, dj = abs(i1 - i0), -abs(j1 - j0)
    si, sj = (1 if i0 < i1 else -1), (1 if j0 < j1 else -1)
    err = di + dj
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if i == i1 and j == j1:
            return cells
        e2 = 2 * err
        if e2 >= dj:
            err += dj
            i += si
        if e2 <= di:
            err += di
            j += sj


def update_grid(grid: OccupancyGrid, scan: Scan, min_intensity: int = 0) -> OccupancyGrid:
    """Retourne une NOUVELLE grille mise à jour avec un scan (la grille d'entrée n'est pas modifiée).

    1. filtrage (pas d'impact, hors portée, intensité faible) ;
    2. passage repère lidar -> repère carte avec la pose du scan ;
    3. cellules traversées (sauf la dernière) -> plus libres ; cellule d'impact -> plus occupée.
    Les cellules hors de la grille sont ignorées (la grille ne s'agrandit pas).
    """
    s = scan.filtered(min_intensity=min_intensity)
    pts = s.global_points()
    ri, rj = grid.world_to_cell(s.pose.x, s.pose.y)
    hi, hj = grid.world_to_cell(pts[:, 0], pts[:, 1])

    free = np.zeros_like(grid.logodds)
    occ = np.zeros_like(grid.logodds)
    for i1, j1 in zip(hi, hj):
        ray = np.array(bresenham(int(ri), int(rj), int(i1), int(j1))[:-1])
        if len(ray):
            ok = grid.inside(ray[:, 0], ray[:, 1])
            # une même cellule traversée par plusieurs faisceaux du même scan n'est comptée qu'une fois
            free[ray[ok, 1], ray[ok, 0]] = L_FREE
    ok = grid.inside(hi, hj)
    occ[hj[ok], hi[ok]] = L_OCC
    free[occ > 0] = 0.0  # une cellule d'impact n'est pas « libérée » par un autre faisceau du même scan

    return replace(grid, logodds=np.clip(grid.logodds + free + occ, L_MIN, L_MAX))


def grid_from_scan(scan: Scan, resolution: float = 0.05, margin: float = 0.3) -> OccupancyGrid:
    """Crée une grille dimensionnée pour contenir le scan (+ marge), puis l'y intègre."""
    return update_grid(grid_for_scans([scan], resolution, margin), scan)


def grid_for_scans(scans: list[Scan], resolution: float = 0.05, margin: float = 0.3) -> OccupancyGrid:
    pts = np.vstack([s.filtered().global_points() for s in scans] + [[[s.pose.x, s.pose.y] for s in scans]])
    lo = pts.min(axis=0) - margin
    hi = pts.max(axis=0) + margin
    return create_grid(hi[0] - lo[0], hi[1] - lo[1], resolution, origin=(lo[0], lo[1]))


def decay(grid: OccupancyGrid, factor: float = 0.95) -> OccupancyGrid:
    """Extension « durée de vie » : rapproche chaque cellule de l'inconnu (L -> 0)."""
    return replace(grid, logodds=grid.logodds * factor)


def save_grid(grid: OccupancyGrid, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, logodds=grid.logodds, resolution=grid.resolution, origin=np.array(grid.origin))
    return path


def load_grid(path: str | Path) -> OccupancyGrid:
    d = np.load(path)
    return OccupancyGrid(d["logodds"], float(d["resolution"]), tuple(float(v) for v in d["origin"]))


def plot_grid(grid: OccupancyGrid, ax=None, scans: list[Scan] | None = None):
    import matplotlib.pyplot as plt

    ax = ax or plt.gca()
    # gris : inconnu, blanc : libre, noir : occupé ; origin="lower" car la ligne 0 est en bas (y min)
    ax.imshow(1.0 - grid.probability(), cmap="gray", vmin=0, vmax=1, origin="lower", extent=grid.extent())
    for s in scans or []:
        ax.plot(s.pose.x, s.pose.y, "r^", markersize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    return ax


def _expand(patterns: list[str]) -> list[str]:
    return sorted({f for p in patterns for f in (glob.glob(p) or [p])})


def main() -> None:
    from .poses import load_scans_with_poses

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="scan(s) + pose(s) -> nouvelle carte")
    b.add_argument("scans", nargs="+")
    b.add_argument("--res", type=float, default=0.05)
    u = sub.add_parser("update", help="carte existante + scan(s) -> carte mise à jour")
    u.add_argument("grid")
    u.add_argument("scans", nargs="+")
    for p in (b, u):
        p.add_argument("--poses", default="data/poses.json")
        p.add_argument("--min-intensity", type=int, default=0)
        p.add_argument("--out", default="out/grid.npz")
    sh = sub.add_parser("show")
    sh.add_argument("grid")
    args = ap.parse_args()

    import matplotlib.pyplot as plt

    if args.cmd == "show":
        plot_grid(load_grid(args.grid))
        plt.show()
        return
    scans = load_scans_with_poses(_expand(args.scans), args.poses)
    grid = grid_for_scans(scans, args.res) if args.cmd == "build" else load_grid(args.grid)
    for s in scans:
        grid = update_grid(grid, s, args.min_intensity)
    out = save_grid(grid, args.out)
    plot_grid(grid, scans=scans)
    plt.title(f"{len(scans)} scan(s), résolution {grid.resolution} m, {grid.shape[1]}x{grid.shape[0]} cellules")
    png = out.with_suffix(".png")
    plt.savefig(png, dpi=150)
    print(f"carte : {out}  image : {png}")
    plt.show()


if __name__ == "__main__":
    main()
