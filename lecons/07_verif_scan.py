"""Leçon 7 : vérifier un scan réel juste après l'acquisition (en salle).

    .venv/Scripts/python lecons/07_verif_scan.py data/scans/pos1_01.csv

Affiche : nombre de points, part sans écho, distance mesurée devant / à gauche / derrière / à droite,
obstacle le plus proche. Sert au TEST D'ORIENTATION : on pose un objet devant puis à gauche du robot,
et on vérifie qu'il apparaît du bon côté.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from lidar.scan import load_scan

DIRECTIONS = {"devant": 0, "gauche": 90, "derrière": 180, "droite": 270}   # angles « maths »
DEMI_SECTEUR = 10   # on regarde ±10° autour de chaque direction


def ecart_angulaire(a: np.ndarray, b: float) -> np.ndarray:
    return np.abs((a - b + 180) % 360 - 180)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fichier")
    ap.add_argument("--sans-graphique", action="store_true")
    args = ap.parse_args()

    brut = load_scan(args.fichier)
    scan = brut.filtered()
    print(f"Fichier {args.fichier}")
    print(f"  {len(brut)} points reçus, {len(brut) - len(scan)} sans écho ou hors portée "
          f"({100 * (len(brut) - len(scan)) / len(brut):.0f} %)")
    if len(brut) < 300:
        print("  ⚠ moins de 300 points : tour incomplet ? (attendu ≈ 450 à 10 tours/s)")

    a, r = scan.angles_deg, scan.ranges_m
    print("  Distance médiane par direction (repère du robot) :")
    for nom, angle in DIRECTIONS.items():
        sel = ecart_angulaire(a, angle) <= DEMI_SECTEUR
        txt = f"{np.median(r[sel]):.2f} m ({sel.sum()} points)" if sel.any() else "aucun écho"
        print(f"    {nom:9s} ({angle:3d}°) : {txt}")
    k = int(np.argmin(r))
    print(f"  Obstacle le plus proche : {r[k]:.2f} m à {a[k]:.1f}°")

    if not args.sans_graphique:
        pts = scan.local_points()
        plt.scatter(pts[:, 0], pts[:, 1], s=3)
        plt.arrow(0, 0, 0.3, 0, width=0.02, color="r")       # flèche = avant du robot (+X)
        plt.text(0.35, 0, "avant", color="r")
        plt.text(0, 0.3, "gauche (+Y)", color="g")
        plt.gca().set_aspect("equal")
        plt.grid(alpha=0.3)
        plt.title(f"{scan.name} vu par le robot")
        plt.show()


if __name__ == "__main__":
    main()
