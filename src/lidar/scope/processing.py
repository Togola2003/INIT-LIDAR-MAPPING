"""Traitements utilisés par LidarScope : filtres, moyenne de plusieurs tours, statistiques."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from ..scan import Scan

DIRECTIONS = (("Devant", 0), ("Gauche", 90), ("Derrière", 180), ("Droite", 270))


@dataclass(frozen=True)
class Filters:
    """Filtres d'AFFICHAGE et d'analyse. Les exports gardent toujours les mesures brutes."""
    min_range_m: float = 0.05
    max_range_m: float = 12.0
    min_intensity: int = 0

    def apply(self, scan: Scan) -> Scan:
        return scan.filtered(self.min_range_m, self.max_range_m, self.min_intensity)

    def as_dict(self) -> dict:
        return {"min_range_m": self.min_range_m, "max_range_m": self.max_range_m, "min_intensity": self.min_intensity}


def _angle_gap(a: np.ndarray, b: float | np.ndarray) -> np.ndarray:
    return np.abs((a - b + 180.0) % 360.0 - 180.0)


def average_scans(scans: list[Scan]) -> Scan:
    """Moyenne de plusieurs tours pris au même endroit (réduit le bruit).

    Pour chaque point du tour le plus récent, on prend la MÉDIANE des distances mesurées
    à moins d'un demi-pas angulaire dans tous les tours (les points sans écho sont ignorés).
    La médiane résiste mieux qu'une moyenne à un passant ou à un reflet ponctuel.
    """
    if len(scans) == 1:
        return scans[0]
    ref = scans[-1]
    step = float(np.median(np.diff(np.sort(ref.angles_deg)))) if len(ref) > 2 else 1.0
    pool_a = np.concatenate([s.angles_deg for s in scans])
    pool_r = np.concatenate([s.ranges_m for s in scans])
    pool_i = np.concatenate([s.intensities for s in scans])
    valid = pool_r > 0
    pool_a, pool_r, pool_i = pool_a[valid], pool_r[valid], pool_i[valid]
    ranges = np.zeros(len(ref))
    inten = np.full(len(ref), np.nan)
    for k, a in enumerate(ref.angles_deg):
        near = _angle_gap(pool_a, a) <= step / 2
        if near.any():
            ranges[k] = np.median(pool_r[near])
            vals = pool_i[near]
            if not np.isnan(vals).all():          # intensité inconnue (Teleplot) : reste NaN
                inten[k] = np.nanmedian(vals)
    return replace(ref, ranges_m=ranges, intensities=inten)


def scan_stats(raw: Scan, filters: Filters) -> dict:
    s = filters.apply(raw)
    out = {
        "points": len(raw),
        "kept": len(s),
        "no_echo_pct": 100.0 * (1 - len(s) / max(len(raw), 1)),
    }
    if len(s):
        k = int(np.argmin(s.ranges_m))
        out.update(
            nearest=(float(s.ranges_m[k]), float(s.angles_deg[k])),
            max_range=float(s.ranges_m.max()),
            mean_intensity=None if np.isnan(s.intensities).all() else float(np.nanmean(s.intensities)),
            directions={
                name: (float(np.median(s.ranges_m[_angle_gap(s.angles_deg, ang) <= 10]))
                       if (_angle_gap(s.angles_deg, ang) <= 10).any() else None)
                for name, ang in DIRECTIONS
            },
        )
    return out
