# TP Jour 2 : localisation (scan matching)

**Question à résoudre :** un scan et une pose approximative étant donnés, retrouver la vraie pose (x, y, θ) du robot
en alignant le scan sur une **référence**, puis comparer les méthodes avec des chiffres.

## D'où on part (fin du Jour 1, 03/11)

| Disponible | Où | Utilité pour le Jour 2 |
|---|---|---|
| Grid map (log-odds, export MapTool) | `src/lidar/gridmap.py` | référence n°1 |
| Carte vectorielle (segments) | `src/lidar/vectormap.py`, `segments.py` | référence n°2 |
| ICP point à point (Kabsch, d_max décroissant) | `src/lidar/matching.py` : `icp` | localisation entre scans : **déjà une méthode** |
| ICP multi-scans point à droite (Gauss-Newton) | `matching.py` : `refine_global` | base de la localisation sur carte vectorielle |
| Simulateur à vérité connue | `src/lidar/simulate.py` | poses connues exigées par l'énoncé |
| Scans réels 02/11 (16, sans intensité) | `data/salle/` | 2ᵉ jeu d'essai |
| **Scans réels 03/11 (13 dont 9 avec intensité, poses recalées)** | `data/salle_jour2/session_salle_2026-11-03.lidar.json` | **jeu d'essai principal** : écart médian entre scans 3,0 cm |

## Où il faut arriver (attendus de l'énoncé)

| # | Attendu | Méthodes retenues | État |
|---|---|---|---|
| 1 | Localisation sur **grid map** | champ de vraisemblance (distance aux cellules occupées) : **recherche exhaustive**, **multi-résolution**, puis raffinement **par gradient** | ⬜ |
| 2 | Localisation sur **carte vectorielle** | distance point → segment : **Gauss-Newton** (gradient), **calcul direct** (Kabsch sur les projections) | ⬜ |
| 3 | Localisation **entre scans** | **ICP point à point** (calcul direct itéré, existe), ICP point à droite, recherche exhaustive en référence | 🟡 |
| — | Mêmes scans, mêmes poses initiales pour toutes les méthodes | banc d'essai commun `TP/Jour2/banc_essai.py` | ⬜ |
| — | Erreur de position (m), erreur angulaire (°), temps, taux de réussite | réussite = erreur < **5 cm et < 2°** (≈ 1 cellule de grid map, bruit LD06 ≈ 1-2 cm) | ⬜ |
| — | Sensibilité à l'initialisation | pose initiale faussée de 0,1 à 1 m et de 5 à 30°, 8 directions | ⬜ |
| — | Poses connues en simulation + incertitude des références réelles | simulation : vérité exacte ; réel : référence = poses recalées, incertitude ≈ écart entre scans (2-3 cm, ~0,5°) | ⬜ |
| bonus | Méthode probabiliste | filtre de particules ou recuit simulé sur la grid map | facultatif |

## Protocole d'évaluation (le même pour toutes les méthodes)

1. **Simulation :** salle simulée, scans à poses exactes. La référence est la carte construite **sans** le scan testé.
2. **Réel :** pour chaque scan de `salle_jour2`, référence = carte des **12 autres** scans (laisser-un-dehors).
   La « vérité » est la pose recalée, avec une incertitude de quelques cm, qu'on indique dans les résultats.
3. Pour chaque scan : les mêmes poses initiales faussées (graine aléatoire fixe) pour toutes les méthodes.
4. Mesures : erreur de position, erreur d'angle, temps, réussite (seuils ci-dessus), courbe « taux de réussite
   en fonction de l'erreur initiale ».

## Plus tard (Jour 3), préparé mais pas développé maintenant

- **Pilotage Wi-Fi :** le firmware RobotControl (`/forward?time=…`, `/rotateleft`, `/scan` binaire, `/pose`)
  sera piloté par un client Python (`src/lidar/robot/`), puis par un onglet *Pilotage* dans LidarScope, avec une
  source `WifiSource` à côté de `SerialSource`.
- **Odométrie :** le robot n'a **pas de codeurs de roues**. L'odométrie sera donc estimée à partir des commandes
  (vitesse étalonnée × durée), puis corrigée à chaque scan par la localisation du Jour 2 (prédiction, puis correction).
  C'est la base du SLAM du Jour 3.
