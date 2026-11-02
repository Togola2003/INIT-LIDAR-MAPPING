"""Représentation d'un scan et lecture/écriture des différents formats CSV.

Formats reconnus automatiquement (d'après l'en-tête) :
- notre format / RobotControl : ``angle_deg,distance_m,intensity`` (+ ligne ``# Robot position: ...``)
- MapTool                   : ``angle_deg,r_m,x_rel_m,y_rel_m``
- printScanCSV de la lib LD06 : ``N,Angle(°),Distance(mm),Intensity[,x(mm),y(mm)]``
Une distance nulle signifie « pas d'impact » : ces points sont retirés au filtrage.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from .geometry import Pose, local_to_global, polar_to_cartesian

POSE_RE = re.compile(
    r"x\s*=\s*(-?[\d.]+)\s*m?\s*,\s*y\s*=\s*(-?[\d.]+)\s*m?\s*,\s*theta\s*=\s*(-?[\d.]+)",
    re.IGNORECASE,
)

# Limites physiques du LD06 (datasheet : 0,02 m à 12 m)
LD06_MIN_RANGE_M = 0.02
LD06_MAX_RANGE_M = 12.0


@dataclass(frozen=True)
class Scan:
    angles_deg: np.ndarray
    ranges_m: np.ndarray
    intensities: np.ndarray
    pose: Pose = Pose()
    name: str = "scan"

    def __len__(self) -> int:
        return len(self.ranges_m)

    def with_pose(self, pose: Pose) -> "Scan":
        return replace(self, pose=pose)

    def local_points(self) -> np.ndarray:
        return polar_to_cartesian(self.angles_deg, self.ranges_m)

    def global_points(self, pose: Pose | None = None) -> np.ndarray:
        return local_to_global(self.local_points(), pose or self.pose)

    def filtered(
        self,
        min_range: float = LD06_MIN_RANGE_M,
        max_range: float = LD06_MAX_RANGE_M,
        min_intensity: int = 0,
    ) -> "Scan":
        """Retire les points sans impact, hors portée ou de trop faible intensité.

        Une intensité inconnue (NaN, ex. données Teleplot) ne fait pas rejeter le point."""
        keep = (
            (self.ranges_m >= min_range)
            & (self.ranges_m <= max_range)
            & ((self.intensities >= min_intensity) | np.isnan(self.intensities))
        )
        return replace(
            self,
            angles_deg=self.angles_deg[keep],
            ranges_m=self.ranges_m[keep],
            intensities=self.intensities[keep],
        )

    def sorted_by_angle(self) -> "Scan":
        order = np.argsort(self.angles_deg % 360.0)
        return replace(
            self,
            angles_deg=self.angles_deg[order] % 360.0,
            ranges_m=self.ranges_m[order],
            intensities=self.intensities[order],
        )


def _parse_header(header: str) -> tuple[int, int, int | None, float]:
    """Retourne (col_angle, col_distance, col_intensité ou None, facteur vers mètres)."""
    cols = [c.strip().lower() for c in header.split(",")]
    angle_col = next(i for i, c in enumerate(cols) if c.startswith("angle"))
    dist_col = next(i for i, c in enumerate(cols) if c.startswith(("distance", "r_m")))
    inten_col = next((i for i, c in enumerate(cols) if c.startswith("intensity")), None)
    scale = 0.001 if "mm" in cols[dist_col] else 1.0
    return angle_col, dist_col, inten_col, scale


def load_scan(path: str | Path, pose: Pose | None = None) -> Scan:
    """Charge un scan CSV. La pose vient de l'argument, sinon du commentaire, sinon (0,0,0)."""
    path = Path(path)
    file_pose = Pose()
    header = None
    rows: list[list[str]] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = POSE_RE.search(line)
            if m:
                file_pose = Pose(*(float(v) for v in m.groups()))
            continue
        if header is None and any(ch.isalpha() for ch in line):
            header = line
            continue
        rows.append(line.split(","))
    if header is None:
        raise ValueError(f"{path}: en-tête CSV introuvable")
    a_col, d_col, i_col, scale = _parse_header(header)
    try:
        # champ vide (intensité inconnue) -> NaN
        data = np.array([[float(v) if v.strip() else np.nan for v in r] for r in rows if len(r) > max(a_col, d_col)])
    except ValueError as exc:
        raise ValueError(f"{path}: valeur non numérique ({exc})") from exc
    if data.size == 0:
        raise ValueError(f"{path}: aucun point")
    intens = data[:, i_col] if i_col is not None else np.full(len(data), 255.0)
    return Scan(
        angles_deg=data[:, a_col],
        ranges_m=data[:, d_col] * scale,
        intensities=intens,
        pose=pose or file_pose,
        name=path.stem,
    )


def save_scan(scan: Scan, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    p = scan.pose
    lines = [
        f"# Robot position: x={p.x:.4f}m, y={p.y:.4f}m, theta={p.theta_deg:.2f}deg",
        "angle_deg,distance_m,intensity",
    ]
    lines += [
        f"{a:.3f},{r:.4f},{'' if np.isnan(i) else int(i)}"   # intensité inconnue -> champ vide
        for a, r, i in zip(scan.angles_deg, scan.ranges_m, scan.intensities)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
