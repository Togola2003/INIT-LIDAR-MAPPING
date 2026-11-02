"""Leçon 4 : calculer les 12 angles d'une trame.

Une trame donne seulement l'angle de DÉBUT et l'angle de FIN. Les 12 angles des points
se déduisent par interpolation (le lidar tourne à vitesse à peu près constante).

Lancer :  .venv/Scripts/python lecons/04_angles.py
"""

import math


def angles_datasheet(debut: float, fin: float) -> list[float]:
    """Datasheet p. 7 : step = (fin - début) / (12 - 1) ; angle_i = début + step * i.
    Le 1er point tombe pile sur 'début', le 12e pile sur 'fin'."""
    pas = ((fin - debut + 360) % 360) / 11
    return [(debut + pas * i) % 360 for i in range(12)]


def angles_bibliotheque(debut: float, fin: float) -> list[float]:
    """Bibliothèque LD06 et cours : pas = (fin - début) / 12 ; angle_n = début + (n + 0,5) * pas.
    Chaque point est placé au MILIEU de sa tranche de 1/12."""
    pas = ((fin - debut + 360) % 360) / 12
    return [(debut + (n + 0.5) * pas) % 360 for n in range(12)]


def vers_math(angle_ld06: float) -> float:
    """Le LD06 compte dans le sens HORAIRE, les maths dans le sens ANTIHORAIRE."""
    return (360 - angle_ld06) % 360


# ------------------------------------------------------------------ 1. la trame de la datasheet
debut, fin = 324.27, 334.70   # lus dans la trame : AB 7E -> 32427, BE 82 -> 33470 (centièmes de degré)
print("1) Trame de la datasheet : début 324,27°, fin 334,70°")
print("   point | datasheet | bibliothèque")
for n, (a, b) in enumerate(zip(angles_datasheet(debut, fin), angles_bibliotheque(debut, fin))):
    print(f"   {n:5d} | {a:8.2f}° | {b:8.2f}°")
print("   -> écart < 0,5° : les deux conventions sont valables, mais il faut en garder UNE seule.")

# ------------------------------------------------------------------ 2. le passage par 0°
debut, fin = 355.00, 5.40
print(f"\n2) Trame à cheval sur 0° : début {debut}°, fin {fin}°")
print(f"   calcul naïf : fin - début = {fin - debut:.2f}°  (négatif : faux !)")
print(f"   avec (fin - début + 360) % 360 = {(fin - debut + 360) % 360:.2f}°  (juste)")
print("   angles obtenus :", ", ".join(f"{a:.1f}" for a in angles_bibliotheque(debut, fin)))

# ------------------------------------------------------------------ 3. horaire -> antihoraire
print("\n3) Pourquoi 360 - angle ? (lidar vu de dessus, 0° = devant le robot)")
print("   angle LD06 | angle maths | côté réel  | x, y d'un obstacle à 1 m")
for a in (0, 90, 180, 270):
    m = vers_math(a)
    cote = {0: "devant", 90: "à DROITE", 180: "derrière", 270: "à GAUCHE"}[a]
    x, y = math.cos(math.radians(m)), math.sin(math.radians(m))
    print(f"   {a:9d}° | {m:10.0f}° | {cote:10s} | x={x:+.1f}  y={y:+.1f}")
print("   En maths, y > 0 = à gauche. Sans la conversion, la carte serait inversée gauche/droite (effet miroir).")
