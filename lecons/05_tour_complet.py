"""Leçon 5 : assembler les trames en un tour complet (un « scan » 360°).

Une trame = 12 points ≈ 10°. Un tour ≈ 37 trames. Il faut savoir quand un tour finit.
On reproduit la méthode de la bibliothèque LD06 (computeData dans ld06.cpp) :
- un nouveau tour commence quand l'angle d'un point est PLUS PETIT que celui du point d'avant
  (passage 359,x° -> 0,x°), ce qui peut arriver au MILIEU d'une trame ;
- le tout premier tour est ignoré (on a commencé à écouter en cours de route) ;
- un tour n'est publié que s'il couvre plus de 340°.

Lancer :  .venv/Scripts/python lecons/05_tour_complet.py
"""

import math

import matplotlib.pyplot as plt

VITESSE = 3600.0          # °/s -> 10 tours/s
POINTS_PAR_S = 4500
PAS = VITESSE / POINTS_PAR_S        # 0,8° entre deux points
ANGLE_TRAME = 12 * PAS              # 9,6° couverts par une trame


def distance_piece(angle_ld06: float) -> float:
    """Distance (mm) vers le mur d'une pièce carrée de 4 m, lidar au centre (pour avoir des données)."""
    a = math.radians(angle_ld06)
    return 2000 / max(abs(math.cos(a)), abs(math.sin(a)))


def generer_trames(debut: float, nb: int, trames_perdues: set[int]):
    """Fabrique nb trames successives (angle début, angle fin, 12 distances) ; certaines sont perdues."""
    for k in range(nb):
        d = (debut + k * ANGLE_TRAME) % 360
        f = (d + ANGLE_TRAME) % 360
        if k in trames_perdues:
            continue
        pas = ((f - d + 360) % 360) / 12
        dist = [distance_piece((d + (n + 0.5) * pas) % 360) for n in range(12)]
        yield d, f, dist


class Assembleur:
    """Même logique que la bibliothèque : un tour en cours de remplissage + le dernier tour complet."""

    def __init__(self):
        self.initialise = False
        self.dernier_angle = 0.0
        self.angle_depart = 0.0
        self.en_cours = []        # tampon rempli point par point
        self.tours_publies = []   # tours complets disponibles pour l'utilisateur

    def ajouter_trame(self, debut, fin, distances):
        pas = ((fin - debut + 360) % 360) / 12
        for n, dist in enumerate(distances):
            angle = (debut + (n + 0.5) * pas) % 360          # angle du capteur (sens horaire)
            if angle < self.dernier_angle:                    # l'angle a « redescendu » : nouveau tour
                if not self.initialise:
                    self.initialise = True
                    print(f"   passage par 0° détecté : premier tour PARTIEL ignoré, on commence à remplir")
                else:
                    couverture = self.dernier_angle - self.angle_depart
                    if couverture > 340:
                        self.tours_publies.append(self.en_cours)
                        print(f"   tour terminé : {len(self.en_cours)} points, couverture {couverture:.1f}° -> PUBLIÉ")
                    else:
                        print(f"   tour terminé : {len(self.en_cours)} points, couverture {couverture:.1f}° -> REJETÉ (< 340°)")
                self.en_cours = []
                self.angle_depart = angle
            self.dernier_angle = angle
            if self.initialise:
                self.en_cours.append(((360 - angle) % 360, dist))   # stocké en angle « maths »


# Écoute commencée à 123,4°. Deux pertes de trames (parasites, buffer plein...) :
# - trames 60 à 62 : autour de 0°  (fin du 2e tour complet)
# - trames 80 à 84 : vers 170°     (en plein milieu du 3e tour complet)
perdues = {60, 61, 62, 80, 81, 82, 83, 84}
trames = list(generer_trames(debut=123.4, nb=150, trames_perdues=perdues))
print(f"{len(trames)} trames reçues ({len(perdues)} perdues), chaque trame couvre {ANGLE_TRAME:.1f}°\n")

asm = Assembleur()
for d, f, dist in trames:
    asm.ajouter_trame(d, f, dist)

print(f"\nTours complets publiés : {len(asm.tours_publies)}")
print(f"Points non publiés encore en cours de remplissage : {len(asm.en_cours)}")

# Affichage du premier tour publié, en cartésien (aperçu de la leçon 6)
tour = asm.tours_publies[0]
xs = [d / 1000 * math.cos(math.radians(a)) for a, d in tour]
ys = [d / 1000 * math.sin(math.radians(a)) for a, d in tour]
plt.scatter(xs, ys, s=4)
plt.plot(0, 0, "r^")
plt.gca().set_aspect("equal")
plt.title(f"Un tour complet : {len(tour)} points (pièce carrée 4 m simulée)")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.show()
