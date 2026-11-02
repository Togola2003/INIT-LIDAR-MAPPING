"""Leçon 8 : afficher plusieurs scans et les recaler à la main (exercice).

Comme en salle, les poses mesurées au mètre sont un peu fausses. Ce script fabrique 3 scans
de la pièce simulée dont les poses écrites dans les fichiers sont volontairement FAUSSES
(quelques cm et quelques degrés). À toi de les corriger dans le visualiseur.

1. Préparer l'exercice :      .venv/Scripts/python lecons/08_recalage.py preparer
2. Recaler à la main :        .venv/Scripts/python -m lidar.viewer "data/exercice8/*.csv" --poses data/exercice8/poses.json
   (choisir un scan dans la liste, flèches / a / e pour le bouger, s pour enregistrer)
3. Vérifier ton résultat :    .venv/Scripts/python lecons/08_recalage.py verifier
"""

import sys
from pathlib import Path

from lidar.geometry import Pose
from lidar.poses import load_poses
from lidar.scan import save_scan
from lidar.simulate import simulate_scan

DOSSIER = Path("data/exercice8")
# (vraie pose, erreur ajoutée dans le fichier). Le scan 1 sert de référence : sa pose est juste.
CAS = {
    "ex_01": (Pose(0.8, 0.8, 0), Pose(0, 0, 0)),
    "ex_02": (Pose(2.8, 1.2, 90), Pose(0.15, -0.10, 6)),
    "ex_03": (Pose(1.2, 2.3, -45), Pose(-0.20, 0.10, -8)),
}


def preparer() -> None:
    for k, (nom, (vraie, err)) in enumerate(CAS.items(), start=1):
        scan = simulate_scan(vraie, seed=k)
        fausse = vraie.moved(err.x, err.y, err.theta_deg)
        save_scan(scan.with_pose(fausse), DOSSIER / f"{nom}.csv")
        print(f"{nom} : pose écrite dans le fichier = ({fausse.x:.2f}, {fausse.y:.2f}, {fausse.theta_deg:.0f}°)")
    print("\nOuvre le visualiseur : les murs ne se superposent pas. Déplace ex_02 et ex_03 pour les aligner sur ex_01.")


def verifier() -> None:
    poses = load_poses(DOSSIER / "poses.json")
    if not poses:
        sys.exit("Pas de data/exercice8/poses.json : as-tu appuyé sur « s » dans le visualiseur ?")
    for nom, (vraie, _) in CAS.items():
        p = poses.get(nom)
        if p is None:
            print(f"{nom} : absent du fichier")
            continue
        dpos = ((p.x - vraie.x) ** 2 + (p.y - vraie.y) ** 2) ** 0.5
        dang = abs((p.theta_deg - vraie.theta_deg + 180) % 360 - 180)
        note = "✅ très bien" if dpos < 0.05 and dang < 2 else ("🙂 correct" if dpos < 0.10 and dang < 4 else "❌ à reprendre")
        print(f"{nom} : écart position {100 * dpos:4.1f} cm, écart angle {dang:4.1f}°  {note}")


if __name__ == "__main__":
    {"preparer": preparer, "verifier": verifier}.get(sys.argv[1] if len(sys.argv) > 1 else "", lambda: print(__doc__))()
