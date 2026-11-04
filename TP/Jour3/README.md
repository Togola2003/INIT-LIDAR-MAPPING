# TP Jour 3 : intégration SLAM et cartographie de la salle

| Attendu | État |
|---|---|
| Finaliser les jours 1 et 2 | ✅ tags `jour1` et `jour2` |
| Combiner localisation + cartographie (début de SLAM) | 🟡 cartographie pas à pas dans LidarLab, validée en simulation (scans posés à 2 cm près) ; à essayer en salle |
| Produire une **carte complète de la salle de classe** | 🟡 cartes des 02/11 (16 scans) et 03/11 (13 scans), recalées à 2-3 cm |
| Bonus : déplacer le robot et construire la carte en temps réel | 🟡 robot Wi-Fi piloté + suivi en direct prêts (non testés sur le vrai robot) |

## D'où on part

- **Localisation dans une carte fixe :** elle marche hors ligne (10 méthodes) et en direct (`localization/tracking.py` :
  global au début, puis autour de la pose précédente, ≈ 15 ms par tour).
- **Robot en Wi-Fi :** lecture de `/scan` et pilotage (`robot.py`), testés contre un faux robot seulement.
- **Ce qui manque pour le SLAM :** la carte ne s'agrandit pas encore. Le robot se localise, mais ses nouveaux points ne
  sont pas ajoutés à la carte.

## 1. Avant toute chose : moteurs et Wi-Fi du vrai robot

Constat du 04/11 : avec n'importe quel programme téléversé, **les moteurs tournaient tout seuls**.
- **Cause 1 :** les 8 fils moteurs sont branchés directement sur l'ESP32 (D15, D2, D4, D16, D17, D5, D18, D19), et
  une broche non réglée flotte ou démarre à l'état haut.
- **Cause 2 :** le firmware RobotControl du prof est prévu pour une carte moteur en I2C. Il utilise D19, qui est ici
  le fil m2b.

Procédure complète, avec le tableau à remplir : **[firmware/README.md](../../firmware/README.md)**
1. `firmware/01_Moteurs_Test` : moteurs arrêtés au démarrage, puis identification de chaque fil (roue, sens).
2. `firmware/02_LidarRobot_WiFi` : lidar + moteurs + Wi-Fi, même interface que RobotControl (donc compatible LidarLab).
3. Téléphone : rejoindre le Wi-Fi du robot, puis ouvrir http://192.168.4.1. La télécommande et le scan sont en direct.
4. LidarLab : Wi-Fi `192.168.4.1` → Connecter, puis Localiser → Robot en direct → Suivre.

**À noter pour le compte rendu :**
- le tableau fil → roue ;
- le taux de tours bien localisés et le temps de calcul ;
- les cas où le robot se perd ;
- la vitesse réelle du robot pour une commande donnée (utile pour l'odométrie).

## 2. Le SLAM « stop and go » (fait le 04/11)

Méthode choisie : le robot **s'arrête** pour chaque scan. Un scan pris à l'arrêt (10 tours combinés) est bien plus
net qu'un tour pris en roulant, donc plus facile à poser.

1. **1er scan = origine** de la carte.
2. Entre deux scans, le robot est **suivi en direct** sur la carte déjà construite (`localization/tracking.py`) :
   - **prédiction :** le mouvement commandé (odométrie par les commandes, `robot.command_motion`) ;
   - **correction :** scan matching autour de la prédiction ;
   - un tour douteux est ignoré (le trajet ne saute plus) ; après 3 tours douteux, recherche dans toute la carte ;
   - calcul dans un fil à part (`app/live.py`) : l'interface ne gèle plus.
3. **Nouveau scan :** posé sur les scans gardés (`place_scan`), à partir de la position suivie, avec son
   **recouvrement** affiché. Gardé, il agrandit la carte, et le suivi repart sur la carte agrandie.
4. **Fermeture de boucle :** « Optimiser toute la carte » (`matching.align_best`) recale toutes les poses ensemble.

**Essai simulé** (robot factice dans une salle de 6 × 4 m, 3 scans) :
- scan posé après un déplacement suivi de 63 cm / 20° : erreur **2 cm, 0,1°** ;
- scan posé après un déplacement « à la main » de 2,1 m / 70°, sans suivi : erreur **2 cm, 0°** ;
- suivi en direct : environ 20 ms par tour.

**Procédure en salle :**
1. départ dans un coin ;
2. petits pas (≤ 1 m, ≤ 45°), toujours avec une bonne partie commune avec les scans précédents ;
3. tour de la salle, puis le milieu ;
4. revenir près du 1er scan à la fin, puis optimiser : c'est la fermeture de boucle.

**Mesures à faire en salle :**
- qualité de la carte finale (murs fins, écart entre scans) ;
- dérive avec et sans correction ;
- comparaison avec la carte faite à la main aux Jours 1 et 2.

Piste du Jour 1, toujours valable : un recalage scan par scan accumule les erreurs. L'optimisation globale de
toutes les poses les corrige.
