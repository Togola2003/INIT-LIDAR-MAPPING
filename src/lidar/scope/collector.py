"""Récolte de plusieurs tours pour une capture : on démarre, on accumule N tours, on s'arrête.

Pourquoi : un seul tour (ou le dernier reçu au moment du clic) est bruité et peut être incomplet.
En récoltant N tours au même endroit puis en les combinant, on obtient un scan plus propre et plus
dense, et on peut juger de sa qualité (bruit, tours reçus) AVANT de le garder.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace

import numpy as np

from ..geometry import Pose
from ..scan import Scan
from .processing import average_scans
from .sources import Frame

COMBINE_MODES = ("Médiane", "Tous les points")
NO_DATA_TIMEOUT_S = 5.0     # plus aucun tour reçu depuis ce temps : la récolte s'arrête


@dataclass(frozen=True)
class Collection:
    target: int                       # nombre de tours voulu
    pose: Pose                        # pose du robot au moment du clic
    frames: tuple[Frame, ...] = ()
    started: float = field(default_factory=time.monotonic)
    last_frame: float = field(default_factory=time.monotonic)

    def add(self, frame: Frame) -> "Collection":
        return replace(self, frames=self.frames + (frame,), last_frame=time.monotonic())

    @property
    def count(self) -> int:
        return len(self.frames)

    @property
    def done(self) -> bool:
        return self.count >= self.target

    @property
    def progress(self) -> float:
        return min(1.0, self.count / max(self.target, 1))

    def stalled(self, now: float | None = None) -> bool:
        return ((now or time.monotonic()) - self.last_frame) > NO_DATA_TIMEOUT_S


def merge_all(scans: list[Scan]) -> Scan:
    """Tous les points de tous les tours, triés par angle (nuage plus dense, bruit conservé)."""
    ref = scans[-1]
    a = np.concatenate([s.angles_deg for s in scans])
    r = np.concatenate([s.ranges_m for s in scans])
    i = np.concatenate([s.intensities for s in scans])
    keep = r > 0
    order = np.argsort(a[keep], kind="stable")
    return replace(ref, angles_deg=a[keep][order], ranges_m=r[keep][order], intensities=i[keep][order])


def range_spread(scans: list[Scan], bin_deg: float = 1.0) -> float | None:
    """Bruit des distances : écart-type médian, par tranche de 1°, des distances mesurées dans tous
    les tours (m). Petit = scan stable. None si moins de 2 tours."""
    if len(scans) < 2:
        return None
    a = np.concatenate([s.angles_deg for s in scans])
    r = np.concatenate([s.ranges_m for s in scans])
    keep = r > 0
    b = (np.floor(a[keep] / bin_deg).astype(int)) % int(round(360 / bin_deg))
    r = r[keep]
    n = np.bincount(b)
    s1, s2 = np.bincount(b, r), np.bincount(b, r * r)
    ok = n >= 3
    if not ok.any():
        return None
    var = s2[ok] / n[ok] - (s1[ok] / n[ok]) ** 2
    return float(np.median(np.sqrt(np.maximum(var, 0))))


def finish(collection: Collection, mode: str = "Médiane") -> tuple[Scan, dict]:
    """Combine les tours récoltés en UN scan (à la pose du clic) + un petit bilan de qualité."""
    scans = [f.scan for f in collection.frames]
    if not scans:
        raise ValueError("aucun tour reçu")
    combined = merge_all(scans) if mode == "Tous les points" else average_scans(scans)
    combined = combined.with_pose(collection.pose)
    pts = [int((s.ranges_m > 0).sum()) for s in scans]
    info = {
        "tours": len(scans),
        "voulus": collection.target,
        "duree_s": round(collection.last_frame - collection.started, 1),
        "points": int((combined.ranges_m > 0).sum()),
        "points_par_tour": int(np.median(pts)),
        "bruit_m": range_spread(scans),
        "mode": mode,
    }
    return combined, info
