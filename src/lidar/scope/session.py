"""Import / export fidèle des données de LidarScope.

Format principal : **JSON auto-décrit** (``*.lidar.json``). Le fichier contient, en plus des
points, la description complète de ses unités, de ses repères et de chaque colonne : un autre
programme, un tableur, un site web ou une IA peut l'interpréter sans autre documentation.

- les mesures BRUTES (angle, distance, intensité) sont toujours exportées sans filtrage ;
- les colonnes x/y sont des valeurs dérivées, recalculables à partir de la pose ;
- l'import relit les mesures brutes et la pose : un export puis un import redonnent les mêmes données.
"""

from __future__ import annotations

import collections
import csv
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from ..geometry import Pose
from ..scan import Scan, load_scan

FORMAT_NAME = "lidarscope.session"
FORMAT_VERSION = 1
COLUMNS = ["angle_deg", "distance_m", "intensity", "x_lidar_m", "y_lidar_m", "x_map_m", "y_map_m"]

DESCRIPTION = {
    "summary": "Scans 2D d'un lidar LD06 (360°) acquis par un robot, avec la pose du robot pour chaque scan.",
    "units": {"distance": "mètre", "angle": "degré", "intensity": "sans unité, 0-255 (réflectivité, ~200 = blanc à < 6 m)"},
    "angle_convention": "sens trigonométrique (antihoraire) vu de dessus ; 0° = avant du robot (+X lidar), 90° = sa gauche",
    "frames": {
        "lidar": "origine au centre du lidar, +X vers l'avant du robot, +Y vers sa gauche",
        "map": "repère de la salle, origine au coin de référence choisi par l'opérateur",
    },
    "pose": "pose = (x_m, y_m, theta_deg) du lidar dans le repère map ; "
            "x_map = x_m + cos(theta)*x_lidar - sin(theta)*y_lidar ; y_map = y_m + sin(theta)*x_lidar + cos(theta)*y_lidar",
    "no_echo": "distance_m = 0 signifie « pas d'écho » (rien mesuré), PAS un obstacle à l'origine",
    "columns": {
        "angle_deg": "angle du point dans le repère lidar [0, 360[ (mesure brute convertie en sens trigonométrique)",
        "distance_m": "distance mesurée (mesure brute)",
        "intensity": "intensité du retour (mesure brute)",
        "x_lidar_m": "dérivé : distance_m * cos(angle_deg)",
        "y_lidar_m": "dérivé : distance_m * sin(angle_deg)",
        "x_map_m": "dérivé : position dans la salle, via la pose",
        "y_map_m": "dérivé : position dans la salle, via la pose",
    },
    "points_layout": "chaque capture a 'columns' (noms) et 'points' (liste de lignes, une ligne = un point)",
}


@dataclass(frozen=True)
class Capture:
    number: int
    scan: Scan
    color: str
    label: str = ""
    visible: bool = True
    captured_utc: str = ""

    @property
    def title(self) -> str:
        return self.label or f"Scan {self.number}"


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _point_rows(scan: Scan) -> list[list[float]]:
    loc, glob = scan.local_points(), scan.global_points()
    return [
        [round(float(a), 3), round(float(r), 4), int(i),
         round(float(lx), 4), round(float(ly), 4), round(float(gx), 4), round(float(gy), 4)]
        for a, r, i, (lx, ly), (gx, gy) in zip(scan.angles_deg, scan.ranges_m, scan.intensities, loc, glob)
    ]


def session_to_dict(captures: list[Capture], extra: dict | None = None) -> dict:
    return {
        "format": FORMAT_NAME,
        "version": FORMAT_VERSION,
        "created_utc": now_utc(),
        "sensor": {"model": "LDROBOT LD06", "range_m": [0.02, 12.0], "points_per_second": 4500},
        "description": DESCRIPTION,
        "captures": [
            {
                "id": c.number,
                "label": c.title,
                "color": c.color,
                "captured_utc": c.captured_utc,
                "pose": {"x_m": c.scan.pose.x, "y_m": c.scan.pose.y, "theta_deg": c.scan.pose.theta_deg},
                "n_points": len(c.scan),
                "n_points_with_echo": int((c.scan.ranges_m > 0).sum()),
                "columns": COLUMNS,
                "points": _point_rows(c.scan),
            }
            for c in captures
        ],
        "processing": extra or {},
    }


def save_session(captures: list[Capture], path: str | Path, extra: dict | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(session_to_dict(captures, extra), ensure_ascii=False, indent=1), encoding="utf-8")
    return path


def load_session(path: str | Path) -> list[Capture]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("format") != FORMAT_NAME:
        raise ValueError(f"ce fichier n'est pas une session LidarScope (format={data.get('format')!r})")
    out = []
    for c in data["captures"]:
        cols = c["columns"]
        pts = np.array(c["points"], dtype=float).reshape(-1, len(cols))
        col = {name: pts[:, k] for k, name in enumerate(cols)}
        p = c["pose"]
        scan = Scan(col["angle_deg"], col["distance_m"], col["intensity"],
                    Pose(p["x_m"], p["y_m"], p["theta_deg"]), name=c.get("label", ""))
        out.append(Capture(int(c["id"]), scan, c.get("color", "#2ecc71"), c.get("label", ""),
                           True, c.get("captured_utc", "")))
    return out


# ------------------------------------------------------------------ CSV (tableur)
TABLE_HEADER = ["scan", "x_robot_m", "y_robot_m", "theta_robot_deg"] + COLUMNS


def save_table(captures: list[Capture], path: str | Path) -> Path:
    """Tableau plat, une ligne par point (pour Excel / LibreOffice / pandas)."""
    path = Path(path)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(TABLE_HEADER)
        for c in captures:
            p = c.scan.pose
            for row in _point_rows(c.scan):
                w.writerow([c.number, p.x, p.y, p.theta_deg] + row)
    return path


def read_any(path: str | Path) -> list[tuple[str, Scan]]:
    """Lit n'importe quel fichier connu : session JSON, tableau LidarScope, ou scan CSV du module."""
    path = Path(path)
    if path.suffix.lower() == ".json":
        return [(c.title, c.scan) for c in load_session(path)]
    with open(path, encoding="utf-8", errors="replace") as f:
        first = next((line for line in f if line.strip() and not line.startswith("#")), "")
    if not first.startswith("scan,"):
        return [(path.stem, load_scan(path))]
    groups: dict[str, list[dict]] = collections.defaultdict(list)
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            groups[row["scan"]].append(row)
    out = []
    for num, rows in groups.items():
        r0 = rows[0]
        pose = Pose(float(r0["x_robot_m"]), float(r0["y_robot_m"]), float(r0["theta_robot_deg"]))
        inten_key = "intensity" if "intensity" in r0 else "intensite"
        arr = {k: np.array([float(r[k]) for r in rows]) for k in ("angle_deg", "distance_m", inten_key)}
        out.append((f"{path.stem} #{num}", Scan(arr["angle_deg"], arr["distance_m"], arr[inten_key], pose)))
    return out
