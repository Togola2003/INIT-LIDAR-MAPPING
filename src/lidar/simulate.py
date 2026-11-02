"""Générateur de scans synthétiques (pièce rectangulaire + obstacle), pour tester sans robot.

Ces scans ne remplacent PAS les scans réels demandés par le TP : ils servent aux tests
automatiques et à valider les outils avant la séance en salle.

    python -m lidar.simulate --out data/simu --poses "0.8,0.8,0" "2.5,1.5,90"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .geometry import Pose
from .scan import Scan, save_scan

# Pièce 4 m x 3 m avec une caisse de 0,5 m x 0,4 m
ROOM_WALLS = [
    (0.0, 0.0, 4.0, 0.0), (4.0, 0.0, 4.0, 3.0), (4.0, 3.0, 0.0, 3.0), (0.0, 3.0, 0.0, 0.0),
    (2.0, 2.0, 2.5, 2.0), (2.5, 2.0, 2.5, 2.4), (2.5, 2.4, 2.0, 2.4), (2.0, 2.4, 2.0, 2.0),
]


def _ray_hit(ox: float, oy: float, dx: float, dy: float, walls) -> float:
    """Distance du premier mur touché par le rayon (ox,oy)+t(dx,dy), 0 si rien."""
    best = np.inf
    for x1, y1, x2, y2 in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        if t > 1e-9 and 0.0 <= u <= 1.0:
            best = min(best, t)
    return 0.0 if np.isinf(best) else best


def simulate_scan(
    pose: Pose,
    walls=ROOM_WALLS,
    step_deg: float = 0.8,
    noise_m: float = 0.01,
    max_range: float = 12.0,
    seed: int | None = 0,
) -> Scan:
    rng = np.random.default_rng(seed)
    angles = np.arange(0.0, 360.0, step_deg)
    ranges = np.empty_like(angles)
    for k, a in enumerate(angles):
        g = np.deg2rad(a + pose.theta_deg)
        r = _ray_hit(pose.x, pose.y, np.cos(g), np.sin(g), walls)
        ranges[k] = 0.0 if r == 0.0 or r > max_range else r + rng.normal(0.0, noise_m)
    intens = np.where(ranges > 0, 200, 0)
    return Scan(angles, ranges, intens.astype(float), pose, name="simu")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="data/simu", help="dossier de sortie")
    ap.add_argument("--poses", nargs="+", default=["0.8,0.8,0", "2.8,1.2,90", "1.2,2.3,-45"])
    ap.add_argument("--noise", type=float, default=0.01, help="bruit gaussien (m)")
    args = ap.parse_args()
    for k, txt in enumerate(args.poses, start=1):
        scan = simulate_scan(Pose.parse(txt), noise_m=args.noise, seed=k)
        path = save_scan(scan, Path(args.out) / f"simu_{k:02d}.csv")
        print(f"écrit {path} ({len(scan)} points, pose {txt})")


if __name__ == "__main__":
    main()
