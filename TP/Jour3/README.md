# TP Jour 3 : intégration SLAM et cartographie de la salle

| Attendu | État |
|---|---|
| Finaliser les jours 1 et 2 | ⬜ |
| Combiner localisation + cartographie (début de SLAM) | ⬜ |
| Produire une **carte complète de la salle de classe** | 🟡 première carte au Jour 1 (`TP/Jour1/figures/fig8_carte_finale.png`) |
| Bonus : déplacer le robot automatiquement et construire la carte en temps réel | ⬜ |

Piste identifiée au Jour 1 : un recalage scan par scan ne corrige pas les erreurs globales (scans 3, 4, 7 et 10).
Il faudra une optimisation globale des poses (graphe de poses) ou des acquisitions plus rapprochées.
