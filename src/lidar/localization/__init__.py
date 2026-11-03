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
from . import scans as S
from . import vector as V
from .common import LocResult, pose_error
from .globalloc import GlobalResult, global_localize


@dataclass(frozen=True)
class References:
    """Les trois références, préparées une fois (champ de vraisemblance, pyramide, index des segments)."""
    field: G.LikelihoodField | None = None
    pyramid: dict | None = None
    segmap: V.SegmentMap | None = None
    seg_field: G.LikelihoodField | None = None
    seg_pyramid: dict | None = None
    scan_ref: S.ScanReference | None = None
    scan_field: G.LikelihoodField | None = None

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
        if segments:
            kw["segmap"] = V.SegmentMap.from_segments(segments)
            kw["seg_field"] = G.LikelihoodField.from_points(kw["segmap"].sample(), resolution)
            kw["seg_pyramid"] = G.build_pyramid(kw["seg_field"])
        if reference_points is not None and len(reference_points):
            kw["scan_ref"] = S.ScanReference.from_points(reference_points)
            kw["scan_field"] = G.LikelihoodField.from_points(kw["scan_ref"].points, resolution)
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
    "vecteur · Gauss-Newton": ("segmap", lambda p, i, r: V.gauss_newton_method(p, i, r.segmap)),
    "vecteur · calcul direct": ("segmap", lambda p, i, r: V.direct(p, i, r.segmap)),
    "vecteur · multi-résolution + Gauss-Newton": ("segmap", lambda p, i, r: V.multiresolution_gauss_newton(
        p, i, r.segmap, r.seg_field, r.seg_pyramid)),
    "scans · ICP point à point": ("scan_ref", lambda p, i, r: S.icp_point_to_point(p, i, r.scan_ref)),
    "scans · ICP point à droite": ("scan_ref", lambda p, i, r: S.icp_point_to_line(p, i, r.scan_ref)),
    "scans · multi-résolution + ICP": ("scan_ref", lambda p, i, r: S.multiresolution_icp(
        p, i, r.scan_ref, r.scan_field)),
}


def localize(method: str, points_local: np.ndarray, initial, refs: References) -> LocResult:
    need, fn = METHODS[method]
    if getattr(refs, need) is None:
        raise ValueError(f"la méthode « {method} » a besoin de la référence {need}")
    return fn(points_local, initial, refs)


__all__ = ["METHODS", "GlobalResult", "LocResult", "References", "global_localize", "localize", "pose_error"]
