"""Acquisition de scans réels LD06 via le port série de l'ESP32.

Le firmware ``firmware/ESP32_LD06_CSV`` envoie chaque scan 360° au format ``printScanCSV``
de la bibliothèque LD06 (une ligne par point, une ligne vide entre deux scans) :

    N,Angle(°),Distance(mm),Intensity[,x(mm),y(mm)]

Les angles sont déjà convertis par la lib dans le sens trigonométrique (360 - θ_LD06).

Exemple (Windows, port COM5, 3 scans, robot posé à x=0,5 m y=1 m orienté à 90°) :

    python -m lidar.acquire --port COM5 --count 3 --pose 0.5,1,90 --out data/scans/salle_pos1
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .geometry import Pose
from .scan import Scan, save_scan

MIN_POINTS_PER_SCAN = 100  # un tour complet du LD06 fait ~450 points : on rejette les scans tronqués


def parse_point_line(line: str) -> tuple[float, float, float] | None:
    """'12,54.66,2000,43,...' -> (angle_deg, distance_m, intensité) ; None si ligne invalide."""
    parts = line.split(",")
    if len(parts) < 4:
        return None
    try:
        _, angle, dist_mm, inten = (float(p) for p in parts[:4])
    except ValueError:
        return None  # en-tête ou ligne corrompue
    return angle, dist_mm / 1000.0, inten


def read_scans(lines, count: int, pose: Pose):
    """Regroupe les lignes en scans (séparés par une ligne vide). Générateur de Scan."""
    points: list[tuple[float, float, float]] = []
    produced = 0
    for raw in lines:
        line = raw.strip()
        if line:
            pt = parse_point_line(line)
            if pt is not None:
                points.append(pt)
            continue
        if len(points) >= MIN_POINTS_PER_SCAN:
            arr = np.array(points)
            yield Scan(arr[:, 0], arr[:, 1], arr[:, 2], pose)
            produced += 1
            if produced >= count:
                return
        points = []


def _serial_lines(port: str, baud: int):
    import serial  # import local : pyserial n'est nécessaire que pour l'acquisition

    with serial.Serial(port, baud, timeout=2) as ser:
        ser.reset_input_buffer()
        skipped_first = False
        while True:
            line = ser.readline().decode("utf-8", errors="replace")
            if not skipped_first:
                # le premier scan reçu peut être incomplet (connexion en cours de tour)
                if line.strip() == "":
                    skipped_first = True
                continue
            yield line


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="port série, ex. COM5 ou /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=1, help="nombre de scans à enregistrer")
    ap.add_argument("--pose", default="0,0,0", help="pose mesurée du robot x,y,theta_deg")
    ap.add_argument("--out", required=True, help="préfixe des fichiers, ex. data/scans/pos1")
    args = ap.parse_args()

    pose = Pose.parse(args.pose)
    prefix = Path(args.out)
    for k, scan in enumerate(read_scans(_serial_lines(args.port, args.baud), args.count, pose), 1):
        path = save_scan(scan, prefix.parent / f"{prefix.name}_{k:02d}.csv")
        print(f"scan {k}/{args.count} : {len(scan)} points -> {path}")


if __name__ == "__main__":
    main()
