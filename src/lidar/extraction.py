"""Transformation d'un nuage de points en segments (étape « extraction de primitives »).

Pipeline (cf. « Construction d'une carte vectorielle 2D », §3) :
1. segmentation : les points sont parcourus dans l'ordre angulaire ; un saut de distance
   entre deux points successifs > seuil adaptatif ouvre un nouveau cluster ;
2. split : dans chaque cluster, on coupe récursivement au point le plus éloigné de la corde
   tant que cet écart dépasse ``split_tol`` (algorithme « split-and-merge » / Douglas-Peucker) ;
3. ajustement : régression orthogonale (ACP) sur les points de chaque morceau, extrémités =
   projection des points extrêmes sur la droite obtenue ;
4. merge : deux segments consécutifs « proches » (segments.are_close) sont fusionnés.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scan import Scan
from .segments import Segment, are_close, merge


@dataclass(frozen=True)
class ExtractionParams:
    jump_base_m: float = 0.05     # saut minimal entre deux points d'un même objet
    jump_ratio: float = 0.05      # + 5 % de la distance (les points s'écartent avec la portée)
    split_tol_m: float = 0.03     # écart max point-corde avant découpe
    min_points: int = 6           # points minimum pour accepter un segment
    min_length_m: float = 0.10    # longueur minimum d'un segment
    merge_angle_deg: float = 8.0
    merge_offset_m: float = 0.04
    merge_gap_m: float = 0.10


def cluster_points(points: np.ndarray, ranges: np.ndarray, p: ExtractionParams) -> list[np.ndarray]:
    """Découpe une liste de points ordonnée angulairement en clusters contigus."""
    n = len(points)
    if n == 0:
        return []
    gaps = np.linalg.norm(np.diff(points, axis=0, append=points[:1]), axis=1)  # gaps[k] = |P(k+1) - P(k)|
    thresholds = p.jump_base_m + p.jump_ratio * ranges
    breaks = gaps > thresholds
    if not breaks.any():
        return [points]
    # on démarre juste après une rupture pour ne pas couper un objet à cheval sur 0°/360°
    start = int(np.argmax(breaks)) + 1
    order = np.roll(np.arange(n), -start)
    clusters, current = [], []
    for k in order:
        current.append(k)
        if breaks[k]:
            clusters.append(points[current])
            current = []
    if current:
        clusters.append(points[current])
    return clusters


def _split(points: np.ndarray, tol: float) -> list[np.ndarray]:
    if len(points) < 3:
        return [points]
    a, b = points[0], points[-1]
    chord = b - a
    norm = np.linalg.norm(chord)
    if norm < 1e-9:
        d = np.linalg.norm(points - a, axis=1)
    else:
        d = np.abs(chord[0] * (points[:, 1] - a[1]) - chord[1] * (points[:, 0] - a[0])) / norm
    k = int(np.argmax(d))
    if d[k] <= tol or k in (0, len(points) - 1):
        return [points]
    return _split(points[: k + 1], tol) + _split(points[k:], tol)


def fit_segment(points: np.ndarray) -> Segment:
    """Régression orthogonale : droite qui minimise la somme des distances perpendiculaires²."""
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    u = vt[0]
    t = (points - c) @ u
    return Segment.from_points(c + t.min() * u, c + t.max() * u)


def points_to_segments(points: np.ndarray, ranges: np.ndarray | None = None,
                       params: ExtractionParams = ExtractionParams()) -> list[Segment]:
    """Nuage de points ordonné angulairement -> liste de segments."""
    points = np.asarray(points, dtype=float)
    if ranges is None:
        ranges = np.zeros(len(points))
    segments: list[Segment] = []
    for cluster in cluster_points(points, ranges, params):
        for part in _split(cluster, params.split_tol_m):
            if len(part) < params.min_points:
                continue
            seg = fit_segment(part)
            if seg.length < params.min_length_m:
                continue
            if segments and are_close(segments[-1], seg, params.merge_angle_deg,
                                      params.merge_offset_m, params.merge_gap_m):
                segments[-1] = merge(segments[-1], seg)
            else:
                segments.append(seg)
    return segments


@dataclass(frozen=True)
class ExtractionStages:
    """Résultat de chaque étape du document « carte vectorielle », pour les afficher une par une."""
    clusters: list[np.ndarray]      # 1) segmentation : groupes de points contigus
    pieces: list[np.ndarray]        # 2) split : morceaux à peu près rectilignes
    rejected: list[np.ndarray]      #    morceaux écartés (trop peu de points / trop courts)
    fitted: list[Segment]           # 3) ajustement : une droite par morceau
    merged: list[Segment]           # 4) fusion des segments voisins alignés


def extraction_stages(points: np.ndarray, ranges: np.ndarray | None = None,
                      params: ExtractionParams = ExtractionParams()) -> ExtractionStages:
    """Même algorithme que points_to_segments, mais en gardant les résultats intermédiaires."""
    points = np.asarray(points, dtype=float)
    ranges = np.zeros(len(points)) if ranges is None else ranges
    clusters = cluster_points(points, ranges, params)
    pieces, rejected, fitted, merged = [], [], [], []
    for cluster in clusters:
        for part in _split(cluster, params.split_tol_m):
            seg = fit_segment(part) if len(part) >= 2 else None
            if len(part) < params.min_points or seg is None or seg.length < params.min_length_m:
                rejected.append(part)
                continue
            pieces.append(part)
            fitted.append(seg)
            if merged and are_close(merged[-1], seg, params.merge_angle_deg, params.merge_offset_m, params.merge_gap_m):
                merged[-1] = merge(merged[-1], seg)
            else:
                merged.append(seg)
    return ExtractionStages(clusters, pieces, rejected, fitted, merged)


def scan_to_segments(scan: Scan, params: ExtractionParams = ExtractionParams(),
                     min_intensity: int = 0) -> list[Segment]:
    """Scan -> segments dans le repère CARTE (utilise la pose du scan)."""
    s = scan.filtered(min_intensity=min_intensity).sorted_by_angle()
    return points_to_segments(s.global_points(), s.ranges_m, params)
