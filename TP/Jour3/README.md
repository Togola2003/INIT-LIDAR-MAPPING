# TP Jour 3 : intégration SLAM et cartographie de la salle

| Attendu | État |
|---|---|
| Finaliser les jours 1 et 2 | ✅ tags `jour1` et `jour2` |
| Combiner localisation + cartographie : SLAM pas à pas | 🟡 fait dans LidarLab, validé en simulation (18 scans, ≤ 1 cm) ; à faire en salle |
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

## 2. Le SLAM pas à pas (consigne du prof, fait le 04/11)

Consigne :
1. scanner une 1re fois : c'est la référence ;
2. déplacer un peu le robot et rescanner : le scan se localise grâce aux points communs ;
3. ajouter sa partie nouvelle : référence + scan = nouvelle référence ;
4. continuer par petits pas jusqu'à la salle complète.

**Méthode, choix grid / vectoriel, références :** [docs/SLAM.md](../../docs/SLAM.md).

| Ce qui est fait | Où |
|---|---|
| Localiser un scan sur la référence : grid map (grossier), puis ICP (fin) | `src/lidar/slam.py` |
| Points communs / partie nouvelle, détection de boucle | `src/lidar/slam.py` |
| Interface : Scanner → aperçu orange / vert → Garder | LidarLab, étape 1 |
| Suivi en direct entre deux scans, sans figer l'interface | `src/lidar/app/live.py` |
| Optimisation de toutes les poses (fermeture de boucle) | `matching.refine_global` |
| Tests : tour complet de la salle simulée (18 scans, ≤ 1 cm) | `tests/test_slam.py` |

**Procédure en salle :**
1. départ dans un coin ;
2. petits pas (≤ 1 m, ≤ 45°), toujours avec une bonne partie commune avec les scans précédents ;
3. tour de la salle, sous les tables, puis le milieu ;
4. revenir près du 1er scan, puis Optimiser.

**Mesures à faire en salle :**
- qualité de la carte finale (murs fins, écart entre scans) ;
- dérive avec et sans correction ;
- comparaison avec la carte faite à la main aux Jours 1 et 2.

Piste du Jour 1, toujours valable : un recalage scan par scan accumule les erreurs. L'optimisation globale de
toutes les poses les corrige.
