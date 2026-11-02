"""Fichier de poses partagé entre les outils : {"nom_du_scan": [x, y, theta_deg], ...}.

Le visualiseur l'écrit après recalage manuel ; les outils de cartographie le relisent.
"""

from __future__ import annotations

import json
from pathlib import Path

from .geometry import Pose
from .scan import Scan, load_scan


def load_poses(path: str | Path | None) -> dict[str, Pose]:
    if not path or not Path(path).exists():
        return {}
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {name: Pose(*vals) for name, vals in raw.items()}


def save_poses(scans: list[Scan], path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {s.name: [round(s.pose.x, 4), round(s.pose.y, 4), round(s.pose.theta_deg, 2)] for s in scans}
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return path


def load_scans_with_poses(files: list[str], poses_file: str | None = None) -> list[Scan]:
    """Charge les scans ; une pose du fichier JSON remplace celle écrite dans le CSV."""
    poses = load_poses(poses_file)
    scans = [load_scan(f) for f in files]
    return [s.with_pose(poses[s.name]) if s.name in poses else s for s in scans]
