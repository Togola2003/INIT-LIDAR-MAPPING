"""Retouches à la main de la carte : ajouter ou supprimer un mur, appliquées aux DEUX cartes.

Une retouche est enregistrée comme une opération (« ajout » ou « suppression » d'un segment), pas
comme une image modifiée. On peut donc les rejouer sur n'importe quelle carte recalculée :
- carte vectorielle : un ajout ajoute le segment ; une suppression retire les segments posés
  le long du segment supprimé ;
- grid map : un ajout rend « occupées » les cases traversées par le segment (comme un mur vu par
  le lidar) ; une suppression les rend « libres » (sur une case d'épaisseur de chaque côté).
Ainsi un mur tracé à la main compte comme un vrai mur pour l'affichage, l'export ET la localisation.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .gridmap import L_MAX, L_MIN, OccupancyGrid
from .segments import Segment

ADD, DELETE = "ajout", "suppression"
ON_SEGMENT_M = 0.10     # un segment est « sur » le segment supprimé si ses extrémités en sont à moins de 10 cm


@dataclass(frozen=True)
class WallEdit:
    kind: str           # ADD ou DELETE
    segment: Segment

    def as_dict(self) -> dict:
        return {"operation": self.kind, "x1": self.segment.x1, "y1": self.segment.y1,
                "x2": self.segment.x2, "y2": self.segment.y2}

    @staticmethod
    def from_dict(d: dict) -> "WallEdit":
        if d["operation"] not in (ADD, DELETE):
            raise ValueError(f"retouche inconnue : {d['operation']!r}")
        return WallEdit(d["operation"], Segment(float(d["x1"]), float(d["y1"]), float(d["x2"]), float(d["y2"])))


def _distance_to_segment(p: np.ndarray, s: Segment) -> float:
    a, b = np.array([s.x1, s.y1]), np.array([s.x2, s.y2])
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def lies_on(s: Segment, deleted: Segment, tol: float = ON_SEGMENT_M) -> bool:
    return (_distance_to_segment(np.array([s.x1, s.y1]), deleted) <= tol and
            _distance_to_segment(np.array([s.x2, s.y2]), deleted) <= tol)


def apply_to_segments(segments: list[Segment], edits) -> list[Segment]:
    out = list(segments)
    for e in edits:
        if e.kind == ADD:
            out.append(e.segment)
        else:
            out = [s for s in out if not lies_on(s, e.segment)]
    return out


def segment_cells(grid: OccupancyGrid, s: Segment, thickness: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Cases (i, j) traversées par le segment (échantillonné tous les tiers de case), élargies de `thickness`."""
    n = max(2, int(np.ceil(s.length / (grid.resolution / 3))) + 1)
    t = np.linspace(0, 1, n)
    i, j = grid.world_to_cell(s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1))
    cells = set(zip(np.asarray(i).tolist(), np.asarray(j).tolist()))
    if thickness:
        cells = {(a + di, b + dj) for a, b in cells
                 for di in range(-thickness, thickness + 1) for dj in range(-thickness, thickness + 1)}
    arr = np.array(sorted(cells), dtype=int).reshape(-1, 2)
    ok = grid.inside(arr[:, 0], arr[:, 1])
    return arr[ok, 0], arr[ok, 1]


def apply_to_grid(grid: OccupancyGrid, edits) -> OccupancyGrid:
    """Nouvelle grille où chaque mur ajouté est occupé et chaque mur supprimé est libre."""
    if not edits:
        return grid
    lo = grid.logodds.copy()
    for e in edits:
        if e.kind == ADD:
            i, j = segment_cells(grid, e.segment)
            lo[j, i] = L_MAX
        else:
            i, j = segment_cells(grid, e.segment, thickness=1)
            lo[j, i] = L_MIN
    return replace(grid, logodds=lo)
