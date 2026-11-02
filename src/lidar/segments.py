"""Opérations géométriques sur les segments 2D.

Tolérance numérique : ``EPS = 1e-6`` m (1 µm). Elle sert à décider qu'une intersection est
« sur » un segment, qu'elle tombe « sur une extrémité », que deux segments sont parallèles,
ou que deux longueurs sont « égales ». Elle est très inférieure au bruit du LD06 (~1 cm)
mais bien supérieure aux erreurs d'arrondi des flottants (~1e-15 relatif).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-6


@dataclass(frozen=True)
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def p1(self) -> np.ndarray:
        return np.array([self.x1, self.y1])

    @property
    def p2(self) -> np.ndarray:
        return np.array([self.x2, self.y2])

    @property
    def vector(self) -> np.ndarray:
        return self.p2 - self.p1

    @property
    def length(self) -> float:
        return float(np.hypot(self.x2 - self.x1, self.y2 - self.y1))

    @property
    def midpoint(self) -> np.ndarray:
        return (self.p1 + self.p2) / 2

    @property
    def direction_deg(self) -> float:
        """Orientation de la droite support dans [0, 180[ (un segment n'a pas de sens)."""
        return float(np.degrees(np.arctan2(self.y2 - self.y1, self.x2 - self.x1)) % 180.0)

    @staticmethod
    def from_points(p: np.ndarray, q: np.ndarray) -> "Segment":
        return Segment(float(p[0]), float(p[1]), float(q[0]), float(q[1]))

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.x1, self.y1, self.x2, self.y2)


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


# --------------------------------------------------------------------------- angle
def angle_between(s1: Segment, s2: Segment) -> float:
    """Angle (degrés) entre les droites support, dans [0, 90]. 0 = parallèles, 90 = perpendiculaires."""
    d = abs(s1.direction_deg - s2.direction_deg) % 180.0
    return min(d, 180.0 - d)


# --------------------------------------------------------------------------- distances
def point_line_distance(p: np.ndarray, s: Segment) -> float:
    """Distance d'un point à la DROITE support du segment."""
    if s.length < EPS:
        return float(np.linalg.norm(p - s.p1))
    return abs(_cross(s.vector, p - s.p1)) / s.length


def point_segment_distance(p: np.ndarray, s: Segment) -> float:
    """Distance d'un point au SEGMENT (projection bornée aux extrémités)."""
    L2 = s.length ** 2
    if L2 < EPS ** 2:
        return float(np.linalg.norm(p - s.p1))
    t = np.clip(np.dot(p - s.p1, s.vector) / L2, 0.0, 1.0)
    return float(np.linalg.norm(p - (s.p1 + t * s.vector)))


def _gap_along(s1: Segment, s2: Segment) -> float:
    """Écart longitudinal entre les deux segments projetés sur la direction de s1 (0 s'ils se recouvrent)."""
    u = s1.vector / max(s1.length, EPS)
    a = sorted((0.0, float(np.dot(s1.p2 - s1.p1, u))))
    b = sorted((float(np.dot(s2.p1 - s1.p1, u)), float(np.dot(s2.p2 - s1.p1, u))))
    return max(0.0, max(a[0], b[0]) - min(a[1], b[1]))


# --------------------------------------------------------------------------- proximité
def are_close(
    s1: Segment,
    s2: Segment,
    max_angle_deg: float = 10.0,
    max_offset_m: float = 0.05,
    max_gap_m: float = 0.10,
) -> bool:
    """Deux segments sont « proches » si :
    - angulairement : angle entre droites ≤ max_angle_deg ;
    - en position : les extrémités du plus court sont à ≤ max_offset_m de la droite du plus long
      (ils sont quasiment sur la même droite ; on mesure sur le plus long car sa direction est
      la plus fiable) ;
    - et ils se recouvrent ou sont séparés de moins de max_gap_m dans le sens de la longueur.
    """
    if angle_between(s1, s2) > max_angle_deg:
        return False
    longest, other = (s1, s2) if s1.length >= s2.length else (s2, s1)
    offset = max(point_line_distance(other.p1, longest), point_line_distance(other.p2, longest))
    if offset > max_offset_m:
        return False
    return _gap_along(longest, other) <= max_gap_m


# --------------------------------------------------------------------------- fusion
def merge(s1: Segment, s2: Segment) -> Segment:
    """Fusionne deux segments proches en un seul.

    Direction = moyenne des directions pondérée par les longueurs (en doublant les angles
    pour que 1° et 179° donnent bien ~0°) ; la droite passe par le barycentre pondéré des milieux ;
    les extrémités sont les projections extrêmes des 4 extrémités sur cette droite.
    """
    w1, w2 = s1.length, s2.length
    if w1 + w2 < EPS:
        return s1
    a1, a2 = np.radians(2 * s1.direction_deg), np.radians(2 * s2.direction_deg)
    theta = 0.5 * np.arctan2(w1 * np.sin(a1) + w2 * np.sin(a2), w1 * np.cos(a1) + w2 * np.cos(a2))
    u = np.array([np.cos(theta), np.sin(theta)])
    c = (w1 * s1.midpoint + w2 * s2.midpoint) / (w1 + w2)
    ts = [float(np.dot(p - c, u)) for p in (s1.p1, s1.p2, s2.p1, s2.p2)]
    return Segment.from_points(c + min(ts) * u, c + max(ts) * u)


# --------------------------------------------------------------------------- intersection
def _intersection_params(s1: Segment, s2: Segment) -> tuple[float, float] | None:
    """Paramètres (t, u) tels que s1.p1 + t·v1 = s2.p1 + u·v2, None si parallèles/colinéaires."""
    v1, v2 = s1.vector, s2.vector
    den = _cross(v1, v2)
    # test de parallélisme relatif aux longueurs (sin(angle) < EPS)
    if abs(den) <= EPS * max(s1.length * s2.length, EPS):
        return None
    w = s2.p1 - s1.p1
    return _cross(w, v2) / den, _cross(w, v1) / den


def intersection(s1: Segment, s2: Segment) -> np.ndarray | None:
    """Point d'intersection des deux SEGMENTS (extrémités incluses, à EPS près), sinon None.

    Les segments parallèles ou colinéaires retournent None (pas de point unique).
    """
    params = _intersection_params(s1, s2)
    if params is None:
        return None
    t, u = params
    tol1 = EPS / max(s1.length, EPS)
    tol2 = EPS / max(s2.length, EPS)
    if -tol1 <= t <= 1 + tol1 and -tol2 <= u <= 1 + tol2:
        return s1.p1 + t * s1.vector
    return None


# --------------------------------------------------------------------------- découpe
def cut(target: Segment, cutter: Segment) -> Segment:
    """Découpe ``target`` traversé par ``cutter`` et garde le plus long des deux morceaux.

    Règles :
    - il faut une intersection située sur les DEUX segments ;
    - et STRICTEMENT à l'intérieur de ``target`` (à plus de EPS de ses extrémités) ;
      une intersection à une extrémité de target ne change rien ;
    - segments colinéaires (ou parallèles) : pas de découpe ;
    - longueurs égales à EPS près : on garde le côté qui contient le premier point (p1).
    Retourne toujours un Segment (``target`` inchangé si aucune découpe).
    """
    p = intersection(target, cutter)
    if p is None:
        return target
    d1 = float(np.linalg.norm(p - target.p1))
    d2 = float(np.linalg.norm(target.p2 - p))
    if d1 <= EPS or d2 <= EPS:
        return target
    if d2 > d1 + EPS:
        return Segment.from_points(p, target.p2)
    return Segment.from_points(target.p1, p)
