"""Leçon 6 : d'un point (r, θ) vu par le lidar à un point (x, y) sur la carte de la salle.

Deux étapes :
1. polaire -> cartésien dans le repère du LIDAR :  x = r·cos θ,  y = r·sin θ
2. repère du lidar -> repère de la CARTE avec la pose du robot (xr, yr, θr) :
       X = xr + x·cos θr − y·sin θr
       Y = yr + x·sin θr + y·cos θr          (rotation de θr, puis translation)

Ce sont exactement les fonctions de src/lidar/geometry.py, utilisées par tous les outils.
Lancer :  .venv/Scripts/python lecons/06_reperes.py
"""

import math

import matplotlib.pyplot as plt

from lidar.geometry import Pose
from lidar.scan import load_scan

# ------------------------------------------------------------------ 1. un seul point, à la main
r, theta = 2.0, 30.0              # le lidar voit un obstacle à 2 m, à 30° (sens maths)
pose = Pose(1.0, 0.5, 90.0)       # robot en (1 m ; 0,5 m), tourné de 90° (il regarde vers +Y)

x = r * math.cos(math.radians(theta))
y = r * math.sin(math.radians(theta))
print("1) Un point, étape par étape")
print(f"   vu par le lidar : r = {r} m, θ = {theta}°")
print(f"   repère lidar    : x = {r}·cos {theta}° = {x:.3f} m ; y = {r}·sin {theta}° = {y:.3f} m")
c, s = math.cos(pose.theta), math.sin(pose.theta)
X = pose.x + x * c - y * s
Y = pose.y + x * s + y * c
print(f"   pose du robot   : ({pose.x}, {pose.y}) orienté à {pose.theta_deg}°")
print(f"   repère carte    : X = {pose.x} + {x:.3f}·cos90 − {y:.3f}·sin90 = {X:.3f} m")
print(f"                     Y = {pose.y} + {x:.3f}·sin90 + {y:.3f}·cos90 = {Y:.3f} m")
print("   Vérification intuitive : le robot regarde vers +Y, l'obstacle est 30° à sa gauche,")
print("   donc à 120° dans la salle : X = 1 + 2·cos120 = 0, Y = 0,5 + 2·sin120 = 2,232. ✔")

# ------------------------------------------------------------------ 2. un scan entier
scan = load_scan("data/simu/simu_02.csv")   # pièce simulée de 4 m × 3 m, pose écrite dans le fichier
print(f"\n2) Scan {scan.name} : {len(scan)} points, pose lue dans le fichier = {scan.pose}")

local = scan.filtered().local_points()
bonne = scan.filtered().global_points()
fausse = scan.filtered().global_points(scan.pose.moved(dtheta_deg=5))   # pose fausse de 5° seulement

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
titres = ["Repère du LIDAR (robot au centre)", "Repère de la CARTE (bonne pose)", "Carte avec une pose fausse de 5°"]
for ax, pts, titre in zip(axes, (local, bonne, fausse), titres):
    ax.scatter(pts[:, 0], pts[:, 1], s=3)
    ax.set_title(titre)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
axes[0].plot(0, 0, "r^", markersize=10)
for ax in axes[1:]:
    ax.plot([0, 4, 4, 0, 0], [0, 0, 3, 3, 0], "k--", lw=1, label="vrais murs")
    ax.plot(scan.pose.x, scan.pose.y, "r^", markersize=10)
    ax.legend(loc="lower right")
print("   À gauche : ce que voit le robot. Au milieu : posé sur la carte, les points tombent sur les murs.")
print("   À droite : 5° d'erreur sur l'angle suffisent à décaler un mur à 3 m de 3·sin5° ≈ 26 cm !")
plt.tight_layout()
plt.show()
