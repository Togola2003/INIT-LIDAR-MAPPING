# TP Jour 2 : localisation (scan matching)

**Minimum attendu :** une méthode fonctionnelle pour chacun des trois types de référence, avec une évaluation chiffrée.

| # | Attendu | Piste retenue | État |
|---|---|---|---|
| 1 | Localisation sur **grid map** (le scan doit être dans une zone couverte) | recherche exhaustive puis multi-résolution sur la carte de vraisemblance | ⬜ à faire |
| 2 | Localisation sur **carte vectorielle** | distance point → segment + descente de gradient / ICP point-à-ligne | ⬜ à faire |
| 3 | Localisation **entre scans** (scan pris près du scan de référence) | ICP point à point (`src/lidar/matching.py`, déjà validé au Jour 1) | 🟡 base prête |
| — | Comparer les méthodes : mêmes scans, mêmes poses initiales | banc d'essai commun | ⬜ |
| — | Mesurer : erreur de position (m), erreur angulaire (°), temps de calcul, taux de réussite (seuils explicités) | | ⬜ |
| — | Faire varier la pose initiale (sensibilité à l'initialisation) | | ⬜ |
| — | Poses connues en simulation + incertitude des poses mesurées sur le robot | simulateur `lidar.simulate`, MapTool ; incertitude estimée au Jour 1 : ±10 cm, ±5° | 🟡 |

Données disponibles : `data/salle/session_salle_recalee.lidar.json` (16 scans, poses corrigées),
cartes du Jour 1 dans `TP/Jour1/resultats/`.
