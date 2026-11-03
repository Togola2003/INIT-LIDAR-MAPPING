"""Carte de référence : la carte de la salle, VALIDÉE puis figée, sur laquelle on se localise.

Quand la carte est jugée bonne (scans recalés, nettoyés, murs retouchés), on la valide : la grid map
et la carte vectorielle, retouches comprises, sont copiées dans une carte de référence qui ne bouge
plus. La localisation (scans du projet ou robot en direct) se fait toujours sur elle : on sait
exactement sur quelle carte on se repère, et toucher aux scans ensuite ne la modifie pas.

Elle s'enregistre dans le projet (.lidar.json) : grid map en valeurs 0-255 compressées par plages
de valeurs identiques, segments en x1,y1,x2,y2.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime

import numpy as np

from .geometry import Pose
from .gridmap import OccupancyGrid, grid_for_scans, update_grid
from .mapedit import apply_to_grid, apply_to_segments
from .scan import Scan
from .segments import Segment
from .vectormap import VectorMapParams, build_vector_map


@dataclass(frozen=True)
class ReferenceMap:
    grid: OccupancyGrid
    segments: tuple[Segment, ...]
    name: str = "Carte de référence"
    created: str = ""
    scans: tuple[tuple[str, Pose], ...] = ()     # scans qui ont servi à la construire (traçabilité)
    edits: int = 0                               # nombre de retouches à la main intégrées

    def describe(self) -> str:
        ny, nx = self.grid.shape
        return (f"{len(self.scans)} scans · grid {nx} × {ny} ({self.grid.resolution * 100:.0f} cm) · "
                f"{len(self.segments)} murs · {self.edits} retouche(s) · {self.created}")


def build_reference(scans: list[Scan], labels: list[str], edits=(), resolution: float = 0.05,
                    vector_params: VectorMapParams | None = None, name: str = "Carte de référence") -> ReferenceMap:
    """scans : scans DÉJÀ filtrés, avec leurs poses. edits : retouches (mapedit.WallEdit)."""
    if not scans:
        raise ValueError("il faut au moins un scan pour construire la carte de référence")
    grid = grid_for_scans(scans, resolution)
    for s in scans:
        grid = update_grid(grid, s)
    segs = build_vector_map(scans, vector_params or VectorMapParams()).segments
    return ReferenceMap(apply_to_grid(grid, edits), tuple(apply_to_segments(segs, edits)), name,
                        datetime.now().strftime("%d/%m/%Y %H:%M"), tuple(zip(labels, (s.pose for s in scans))),
                        len(edits))


def _runs(values: np.ndarray) -> list[int]:
    v = values.ravel()
    if not len(v):
        return []
    starts = np.flatnonzero(np.r_[True, v[1:] != v[:-1]])
    lengths = np.diff(np.r_[starts, len(v)])
    return np.column_stack((v[starts], lengths)).ravel().astype(int).tolist()


def _unruns(runs: list[int], size: int) -> np.ndarray:
    arr = np.asarray(runs, dtype=int).reshape(-1, 2)
    out = np.repeat(arr[:, 0], arr[:, 1]).astype(np.uint8)
    if len(out) != size:
        raise ValueError("grid map de référence incomplète")
    return out


def to_dict(ref: ReferenceMap) -> dict:
    ny, nx = ref.grid.shape
    return {
        "description": "Carte de référence validée : grid map (0 = libre, 255 = occupé, 127 = inconnu, lignes "
                       "du bas vers le haut, compressée par plages [valeur, nombre]) + segments x1,y1,x2,y2 (m)",
        "name": ref.name, "created": ref.created, "edits": ref.edits,
        "grid": {"width": nx, "height": ny, "resolution_m": ref.grid.resolution,
                 "origin_m": list(ref.grid.origin), "values_rle": _runs(ref.grid.to_uint8())},
        "segments": [list(s.as_tuple()) for s in ref.segments],
        "scans": [{"label": lab, "pose": [p.x, p.y, p.theta_deg]} for lab, p in ref.scans],
    }


def from_dict(d: dict) -> ReferenceMap:
    g = d["grid"]
    nx, ny = int(g["width"]), int(g["height"])
    p = np.clip(_unruns(g["values_rle"], nx * ny).reshape(ny, nx) / 255.0, 0.02, 0.98)
    grid = OccupancyGrid(np.log(p / (1 - p)), float(g["resolution_m"]), tuple(g["origin_m"]))
    return ReferenceMap(grid, tuple(Segment(*map(float, s)) for s in d["segments"]), d.get("name", "Carte de référence"),
                        d.get("created", ""), tuple((s["label"], Pose(*s["pose"])) for s in d.get("scans", [])),
                        int(d.get("edits", 0)))
