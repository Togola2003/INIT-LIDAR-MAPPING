"""Localisation par scan matching (Jour 2) : trois types de référence, plusieurs méthodes chacune.

    from lidar.localization import References, METHODS
    refs = References.build(scans_carte, scan_voisin_points)
    result = METHODS["grid · multi-résolution + gradient"](points_local, pose_initiale, refs)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..gridmap import grid_for_scans, update_grid
from ..scan import Scan
from ..segments import Segment
from ..vectormap import VectorMapParams, build_vector_map
from . import grid as G
from .common import LocResult, pose_error


@dataclass(frozen=True)
class References:
    """Les trois références, préparées une fois (champ de vraisemblance, pyramide, index des segments)."""
    field: G.LikelihoodField | None = None
    pyramid: dict | None = None

    @staticmethod
    def build(map_scans: list[Scan] | None = None, segments: list[Segment] | None = None,
              reference_points: np.ndarray | None = None, resolution: float = 0.05,
              vector_params: VectorMapParams | None = None) -> "References":
        """map_scans (avec leurs poses) -> grid map ; segments (ou, à défaut, carte vectorielle construite
        à partir de map_scans) -> carte vectorielle ; reference_points -> scan de référence."""
        kw = {}
        if map_scans:
            grid = grid_for_scans(map_scans, resolution)
            for s in map_scans:
                grid = update_grid(grid, s)
            kw["field"] = G.LikelihoodField.from_grid(grid)
            kw["pyramid"] = G.build_pyramid(kw["field"])
            if segments is None:
                segments = build_vector_map(map_scans, vector_params or VectorMapParams()).segments
        return References(**kw)


Method = Callable[[np.ndarray, "object", References], LocResult]

# nom -> (référence nécessaire, fonction). Mêmes entrées pour toutes : points du scan (repère lidar),
# pose initiale, références.
METHODS: dict[str, tuple[str, Method]] = {
    "grid · exhaustive": ("field", lambda p, i, r: G.exhaustive(p, i, r.field)),
    "grid · multi-résolution": ("field", lambda p, i, r: G.multiresolution(p, i, r.field, pyramid=r.pyramid)),
    "grid · gradient": ("field", lambda p, i, r: G.gradient(p, i, r.field)),
    "grid · multi-résolution + gradient": ("field", lambda p, i, r: G.multiresolution_gradient(
        p, i, r.field, pyramid=r.pyramid)),
}


def localize(method: str, points_local: np.ndarray, initial, refs: References) -> LocResult:
    need, fn = METHODS[method]
    if getattr(refs, need) is None:
        raise ValueError(f"la méthode « {method} » a besoin de la référence {need}")
    return fn(points_local, initial, refs)


__all__ = ["METHODS", "LocResult", "References", "localize", "pose_error"]
