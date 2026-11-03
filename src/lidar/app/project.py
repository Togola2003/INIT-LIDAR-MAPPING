"""Le projet de LidarLab : tout ce que l'utilisateur fabrique, séparé de l'interface.

Un projet = des captures (scans + poses), une carte de référence facultative et, si l'utilisateur a
retouché la carte vectorielle à la main, la liste de ses segments. Le projet est IMMUABLE : chaque
action rend un nouveau projet, ce qui donne l'annulation (Ctrl+Z) presque gratuitement.

Ce module ne dépend d'aucune bibliothèque graphique : il est testé seul et pourra servir tel quel
à une future version de l'application (autre interface, web, mobile).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable

import numpy as np

from ..geometry import Pose, move_rigid
from ..scan import Scan
from ..scope.session import Capture, load_session, now_utc, read_any, save_session
from ..segments import Segment

# couleurs « système » (lisibles sur fond clair), une par capture
PALETTE = ["#007AFF", "#34C759", "#FF9500", "#FF2D55", "#AF52DE", "#30B0C7", "#5856D6", "#FFCC00",
           "#FF3B30", "#00C7BE", "#A2845E", "#32ADE6"]
HISTORY = 80


@dataclass(frozen=True)
class Project:
    captures: tuple[Capture, ...] = ()
    reference: tuple[Segment, ...] = ()            # carte importée (ex. MapTool), pour comparer
    manual_segments: tuple[Segment, ...] | None = None   # carte vectorielle retouchée à la main
    name: str = "Nouveau projet"
    path: str | None = None

    # ------------------------------------------------------------------ lecture
    def find(self, number: int | None) -> Capture | None:
        return next((c for c in self.captures if c.number == number), None)

    @property
    def visible(self) -> list[Capture]:
        return [c for c in self.captures if c.visible]

    @property
    def is_empty(self) -> bool:
        return not self.captures

    def poses(self) -> dict[int, Pose]:
        return {c.number: c.scan.pose for c in self.captures}


# ---------------------------------------------------------------------- opérations (pures)
def add_scans(project: Project, items: list[tuple[str, Scan]], source: str = "fichier",
              raw: tuple = ()) -> tuple[Project, list[int]]:
    n0 = max((c.number for c in project.captures), default=0)
    new = []
    for k, (label, scan) in enumerate(items, start=1):
        n = n0 + k
        new.append(Capture(n, scan, PALETTE[(n - 1) % len(PALETTE)], label, True, now_utc(), source, raw))
    return replace(project, captures=project.captures + tuple(new)), [c.number for c in new]


def add_captures(project: Project, captures: list[Capture]) -> tuple[Project, list[int]]:
    """Ajoute des captures déjà complètes (relues d'une session) en leur donnant de nouveaux numéros."""
    n0 = max((c.number for c in project.captures), default=0)
    new = [replace(c, number=n0 + k, color=PALETTE[(n0 + k - 1) % len(PALETTE)], visible=True)
           for k, c in enumerate(captures, start=1)]
    return replace(project, captures=project.captures + tuple(new)), [c.number for c in new]


def remove(project: Project, numbers: set[int]) -> Project:
    return replace(project, captures=tuple(c for c in project.captures if c.number not in numbers))


def set_visible(project: Project, numbers: set[int], visible: bool) -> Project:
    return replace(project, captures=tuple(replace(c, visible=visible) if c.number in numbers else c
                                           for c in project.captures))


def set_poses(project: Project, poses: dict[int, Pose]) -> Project:
    return replace(project, captures=tuple(replace(c, scan=c.scan.with_pose(poses[c.number]))
                                           if c.number in poses else c for c in project.captures))


def move(project: Project, numbers: set[int], dx: float, dy: float, dth: float, pivot) -> Project:
    return set_poses(project, {c.number: move_rigid(c.scan.pose, dx, dy, dth, pivot)
                               for c in project.captures if c.number in numbers})


def erase_points(project: Project, numbers: set[int], box: tuple[float, float, float, float]) -> tuple[Project, int]:
    """Efface (distance -> 0 = « pas d'écho ») les points des captures `numbers` situés dans le rectangle
    box = (x0, y0, x1, y1), en coordonnées carte. Les angles sont gardés : le scan reste un vrai scan."""
    x0, x1 = sorted(box[0::2])
    y0, y1 = sorted(box[1::2])
    caps, erased = [], 0
    for c in project.captures:
        if c.number in numbers and c.visible:
            g = c.scan.global_points()
            inside = (c.scan.ranges_m > 0) & (g[:, 0] >= x0) & (g[:, 0] <= x1) & (g[:, 1] >= y0) & (g[:, 1] <= y1)
            if inside.any():
                r = np.where(inside, 0.0, c.scan.ranges_m)
                c = replace(c, scan=replace(c.scan, ranges_m=r))
                erased += int(inside.sum())
        caps.append(c)
    return replace(project, captures=tuple(caps)), erased


def set_manual_segments(project: Project, segments: list[Segment] | None) -> Project:
    return replace(project, manual_segments=None if segments is None else tuple(segments))


def pivot_of(project: Project, numbers: set[int]) -> tuple[float, float]:
    poses = [c.scan.pose for c in project.captures if c.number in numbers]
    if not poses:
        return (0.0, 0.0)
    return float(np.mean([p.x for p in poses])), float(np.mean([p.y for p in poses]))


# ---------------------------------------------------------------------- fichiers
def open_files(project: Project, paths: list[str]) -> tuple[Project, list[int], list[str]]:
    """Ajoute le contenu de fichiers (session .json, CSV du module, tableau LidarScope). Rend aussi les erreurs."""
    added, errors = [], []
    for path in paths:
        try:
            if Path(path).suffix.lower() == ".json":
                project, nums = add_captures(project, load_session(path))
                segs = json.loads(Path(path).read_text(encoding="utf-8")).get("processing", {}).get(
                    "carte_vectorielle_retouchee")
                if segs and project.manual_segments is None:
                    project = set_manual_segments(project, [Segment(*s) for s in segs])
            else:
                project, nums = add_scans(project, read_any(path))
            added += nums
        except (ValueError, OSError, KeyError) as exc:
            errors.append(f"{Path(path).name} : {exc}")
    return project, added, errors


def save(project: Project, path: str, extra: dict | None = None) -> Project:
    info = {"projet": project.name}
    if project.manual_segments is not None:
        info["carte_vectorielle_retouchee"] = [list(s.as_tuple()) for s in project.manual_segments]
    save_session(list(project.captures), path, {**info, **(extra or {})})
    return replace(project, path=path, name=Path(path).name.split(".")[0])


# ---------------------------------------------------------------------- historique
@dataclass
class Store:
    """Le projet courant + l'historique pour annuler / rétablir. Prévient l'interface à chaque changement."""
    project: Project = field(default_factory=Project)
    past: list = field(default_factory=list)
    future: list = field(default_factory=list)
    saved: Project | None = None
    listeners: list = field(default_factory=list)

    def subscribe(self, fn: Callable[[Project], None]) -> None:
        self.listeners.append(fn)

    def apply(self, new: Project, remember: bool = True) -> None:
        if new is self.project:
            return
        if remember:
            self.past = (self.past + [self.project])[-HISTORY:]
            self.future = []
        self.project = new
        for fn in self.listeners:
            fn(new)

    def undo(self) -> bool:
        if not self.past:
            return False
        self.future.append(self.project)
        prev = self.past.pop()
        self.apply(prev, remember=False)
        return True

    def redo(self) -> bool:
        if not self.future:
            return False
        self.past.append(self.project)
        self.apply(self.future.pop(), remember=False)
        return True

    def reset(self) -> None:
        """« Nouveau projet » : tout est vidé, l'historique aussi."""
        self.past, self.future, self.saved = [], [], None
        self.apply(Project(), remember=False)

    def mark_saved(self) -> None:
        self.saved = self.project

    @property
    def dirty(self) -> bool:
        return self.project is not self.saved and not (self.saved is None and self.project.is_empty)
