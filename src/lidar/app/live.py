"""Localisation en direct, en ARRIÈRE-PLAN : l'interface ne gèle jamais pendant un calcul.

Le robot envoie environ 5 tours par seconde ; une localisation dure de 10 ms (suivi) à 2 s (recherche
globale). Le calcul se fait donc dans un fil séparé, toujours sur le DERNIER tour reçu (les tours plus
anciens sont sautés, ils ne servent plus) ; le mouvement commandé entre-temps (odométrie) sert de
prédiction. L'interface récupère le dernier résultat à son rythme avec poll().
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np

from ..geometry import Pose
from ..localization.tracking import Tracker, TrackStep
from ..robot import Odometry
from ..segments import Segment


def trail_segments(trail) -> list[Segment]:
    """Trajet (Pose ou None = coupure) -> segments à dessiner, sans relier deux morceaux séparés."""
    return [Segment(a.x, a.y, b.x, b.y) for a, b in zip(trail, trail[1:]) if a is not None and b is not None]


@dataclass(frozen=True)
class LiveResult:
    step: TrackStep
    points: np.ndarray          # le tour localisé (repère lidar)
    trail: tuple                # trajet (Pose ou None = coupure)


class LiveLocalizer:
    def __init__(self, odometry: Odometry) -> None:
        self.odometry = odometry
        self.tracker: Tracker | None = None
        self._latest: np.ndarray | None = None
        self._result: LiveResult | None = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.busy = False

    @property
    def running(self) -> bool:
        """Un suivi est en cours (une carte a été donnée par start)."""
        return self.tracker is not None

    def start(self, grid, segments, pose: Pose | None = None, field=None, pyramid=None) -> None:
        """(Re)démarre le suivi sur une carte. pose : position connue au départ (sinon recherche globale)."""
        tracker = Tracker(grid, segments, field, pyramid)
        tracker.reset(pose)
        if pose is not None:
            tracker.trail = [pose]
        self.odometry.take()                       # on oublie le mouvement commandé avant le départ
        with self._lock:
            self.tracker = tracker
            self._latest, self._result = None, None
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        """Arrête le suivi. Le fil reste en attente (prêt pour le prochain start) ; close() le termine."""
        with self._lock:
            self.tracker, self._latest, self._result = None, None, None

    def close(self) -> None:
        self.stop()
        self._stop.set()
        self._wake.set()

    def relocalize(self) -> None:
        with self._lock:
            if self.tracker is not None:
                self.tracker.reset()

    def submit(self, points: np.ndarray) -> None:
        with self._lock:
            self._latest = points
        self._wake.set()

    def poll(self) -> LiveResult | None:
        with self._lock:
            out, self._result = self._result, None
        return out

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(0.5)
            self._wake.clear()
            with self._lock:
                pts, tracker = self._latest, self.tracker
                self._latest = None
            if pts is None or tracker is None:
                continue
            self.busy = True
            try:
                step = tracker.update(pts, self.odometry.take())
            except ValueError:
                step = None
            finally:
                self.busy = False
            if step is not None:
                with self._lock:
                    if tracker is self.tracker:
                        self._result = LiveResult(step, pts, tuple(tracker.trail))
