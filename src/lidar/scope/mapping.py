"""Construction des cartes à partir des captures, avec les réglages du cours de cartographie.

Grid map (doc « Construction d'une grid map 2D ») :
  1. grille : résolution + état initial (inconnu / libre / occupé)
  2. prétraitement : filtres portée / intensité
  3. cellules traversées : p(occupé | faisceau traverse) = p_free  -> L += logit(p_free)
  4. cellule d'impact   : p(occupé | impact)           = p_occ   -> L += logit(p_occ)
  5. nettoyage : suppression des cellules occupées isolées
  Modèle d'affichage : probabiliste (niveaux de gris) ou binaire (occupé / libre / inconnu).

Carte vectorielle (doc « Construction d'une carte vectorielle 2D ») :
  segmentation (saut de distance) -> split-and-merge -> ajustement -> association / fusion -> absences.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from ..extraction import ExtractionParams
from ..gridmap import INITIAL_STATES, OccupancyGrid, grid_for_scans, p_to_logodds, remove_isolated, ternary, update_grid
from ..scan import Scan
from ..segments import Segment
from ..vectormap import VectorMapParams, build_vector_map
from .processing import Filters


@dataclass(frozen=True)
class GridParams:
    enabled: bool = True
    resolution_m: float = 0.05
    initial_state: str = "inconnu"     # inconnu / libre / occupé
    model: str = "probabiliste"        # probabiliste / binaire
    p_occ: float = 0.70                # confiance « obstacle » à chaque impact
    p_free: float = 0.40               # confiance « occupé » quand un faisceau traverse (< 0,5 = libre)
    remove_isolated: bool = False


@dataclass(frozen=True)
class VectorParams:
    enabled: bool = True
    jump_m: float = 0.05               # saut de distance qui sépare deux objets
    split_tol_m: float = 0.03          # écart max point-droite avant de couper
    min_length_m: float = 0.10
    merge_angle_deg: float = 10.0

    def to_vectormap(self) -> VectorMapParams:
        ext = ExtractionParams(jump_base_m=self.jump_m, split_tol_m=self.split_tol_m,
                               min_length_m=self.min_length_m, merge_angle_deg=self.merge_angle_deg)
        return VectorMapParams(extraction=ext, assoc_angle_deg=self.merge_angle_deg, min_length_m=self.min_length_m)


@dataclass(frozen=True)
class MapParams:
    filters: Filters = field(default_factory=Filters)
    grid: GridParams = field(default_factory=GridParams)
    vector: VectorParams = field(default_factory=VectorParams)

    def as_dict(self) -> dict:
        return {"filters": self.filters.as_dict(), "grid": asdict(self.grid), "vector": asdict(self.vector)}


@dataclass(frozen=True)
class MapResult:
    grid: OccupancyGrid | None
    segments: list[Segment]
    stats: dict

    def grid_image(self, model: str) -> np.ndarray | None:
        """Image à afficher (1 = blanc = libre, 0 = noir = occupé, 0,5 = gris = inconnu)."""
        if self.grid is None:
            return None
        if model == "binaire":
            return np.select([ternary(self.grid) == 1, ternary(self.grid) == 0], [0.0, 1.0], 0.5)
        return 1.0 - self.grid.probability()


def beam_image(scan: Scan, params: MapParams) -> tuple[np.ndarray, tuple] | None:
    """Image RGBA des cellules TRAVERSÉES (bleu) et d'IMPACT (rouge) d'un seul scan :
    visualise les étapes 3 et 4 de la construction d'une grid map."""
    from ..gridmap import beam_cells

    s = params.filters.apply(scan)
    if not len(s):
        return None
    grid = grid_for_scans([s], params.grid.resolution_m)
    free, hit = beam_cells(grid, s)
    rgba = np.zeros(grid.shape + (4,))
    rgba[free[:, 1], free[:, 0]] = (0.20, 0.60, 0.86, 0.45)
    rgba[hit[:, 1], hit[:, 0]] = (0.90, 0.33, 0.29, 0.90)
    return rgba, grid.extent()


def build_maps(scans: list[Scan], params: MapParams) -> MapResult:
    scans = [params.filters.apply(s) for s in scans]
    scans = [s for s in scans if len(s)]
    if not scans:
        return MapResult(None, [], {})
    stats: dict = {"scans": len(scans), "points": int(sum(len(s) for s in scans))}
    grid = None
    if params.grid.enabled:
        g = params.grid
        grid = grid_for_scans(scans, g.resolution_m, initial_logodds=INITIAL_STATES[g.initial_state])
        for s in scans:
            grid = update_grid(grid, s, l_free=p_to_logodds(g.p_free), l_occ=p_to_logodds(g.p_occ))
        if g.remove_isolated:
            grid = remove_isolated(grid)
        t = ternary(grid)
        ny, nx = grid.shape
        stats["grid"] = {"cells": f"{nx} x {ny}", "resolution_m": g.resolution_m,
                         "occupied": int((t == 1).sum()), "free": int((t == 0).sum()),
                         "unknown": int((t == -1).sum()),
                         "explored_area_m2": round(float((t != -1).sum()) * g.resolution_m ** 2, 2)}
    segments: list[Segment] = []
    if params.vector.enabled:
        segments = build_vector_map(scans, params.vector.to_vectormap()).segments
        stats["vector"] = {"segments": len(segments),
                           "total_length_m": round(sum(s.length for s in segments), 2),
                           "longest_m": round(max((s.length for s in segments), default=0.0), 2)}
    return MapResult(grid, segments, stats)
