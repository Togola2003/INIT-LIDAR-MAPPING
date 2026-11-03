# TP Jour 3 : intégration SLAM et cartographie de la salle

| Attendu | État |
|---|---|
| Finaliser les jours 1 et 2 | ✅ tags `jour1` et `jour2` |
| Combiner localisation + cartographie (début de SLAM) | ⬜ |
| Produire une **carte complète de la salle de classe** | 🟡 cartes des 02/11 (16 scans) et 03/11 (13 scans), recalées à 2-3 cm |
| Bonus : déplacer le robot et construire la carte en temps réel | 🟡 robot Wi-Fi piloté + suivi en direct prêts (non testés sur le vrai robot) |

## D'où on part

- **Localisation dans une carte fixe :** elle marche hors ligne (10 méthodes) et en direct (`localization/tracking.py` :
  global au début, puis autour de la pose précédente, ≈ 15 ms par tour).
- **Robot en Wi-Fi :** lecture de `/scan` et pilotage (`robot.py`), testés contre un faux robot seulement.
- **Ce qui manque pour le SLAM :** la carte ne s'agrandit pas encore. Le robot se localise, mais ses nouveaux points ne
  sont pas ajoutés à la carte.

## 1. Avant toute chose : tester le vrai robot en Wi-Fi (procédure)

1. Téléverser `RESSOURCES/Tools/RobotControl/ESP32RobotControl/ESP32RobotControl.ino` sur l'ESP32. Il crée le
   réseau Wi-Fi « RobotESP32 », mot de passe `12345678`.
2. Connecter le PC à ce réseau. Le PC n'a alors plus Internet, c'est normal.
3. `LidarLab.bat` → étape **1 Récolter** → **Wi-Fi** → adresse `192.168.4.1` → **Connecter**.
   - **Attendu :** le scan en direct s'affiche en noir et la pastille en haut devient verte.
   - **Sinon :** vérifier le réseau Wi-Fi du PC, et ouvrir `http://192.168.4.1/ping` dans un navigateur (il doit
     répondre `OK`).
4. Petits mouvements avec le pavé ou le clavier (I K J L U O, Espace = stop), durée ≈ 300-400 ms.
5. Ouvrir le projet, puis étape **4 Cartes** → **Valider**, puis étape **5 Localiser** → **Robot en direct** →
   **Suivre le robot**.
6. **À noter pour le compte rendu :**
   - le taux de tours bien localisés ;
   - le temps de calcul ;
   - les cas où il se perd (trop près d'un mur, meubles) ;
   - la vitesse réelle du robot pour une commande de 300 ms.

## 2. Ensuite : le SLAM (plan)

1. **Localiser** chaque nouveau tour sur la carte actuelle, comme aujourd'hui.
2. Si la localisation est sûre (≥ 50 % des points sur les murs), **ajouter** ce tour à la carte : mise à jour de la
   grid map du Jour 1 (`update_grid`), à la pose trouvée.
3. Quand on revient à un endroit déjà vu, corriger l'erreur accumulée (fermeture de boucle) : c'est le recalage
   global déjà écrit (`matching.refine_global`).
4. **Mesures :**
   - qualité de la carte finale (murs fins, écart entre scans) ;
   - dérive avec et sans correction ;
   - comparaison avec la carte faite à la main aux Jours 1 et 2.

Piste du Jour 1, toujours valable : un recalage scan par scan accumule les erreurs. L'optimisation globale de
toutes les poses les corrige.
