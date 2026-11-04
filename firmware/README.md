# Robot : moteurs, lidar et Wi-Fi, étape par étape

| Dossier | Rôle |
|---|---|
| `01_Moteurs_Test/` | **Étape 1.** Arrête les moteurs au démarrage et fait tourner une broche à la fois, pour savoir quel fil commande quelle roue |
| `02_LidarRobot_WiFi/` | **Étape 2.** Le robot complet : lidar + 4 moteurs + Wi-Fi + télécommande sur téléphone, compatible LidarLab |
| `ESP32_LD06_CSV/` | Jour 1 : scans envoyés en CSV sur le câble USB (sans moteurs) |

Aucune bibliothèque à installer : seulement la carte **ESP32** (déjà installée, carte « ESP32 Dev Module ») et la
bibliothèque **LD06** (déjà installée). Les deux programmes ont été compilés avec succès (Arduino ESP32 3.3.12).

## Pourquoi les moteurs tournaient tout seuls

Les 8 fils des moteurs (m1a … m4b) arrivent **directement** sur des broches de l'ESP32 :

| Fil | m3a | m3b | m4a | m4b | m1a | m1b | m2a | m2b |
|---|---|---|---|---|---|---|---|---|
| Broche ESP32 | D15 | D2 | D4 | D16 | D17 | D5 | D18 | D19 |

- **Une broche que le programme ne règle pas « flotte ».** Certaines sont même à l'état HAUT pendant le démarrage
  de l'ESP32 (D15, D5). Le pont en H du moteur croit alors qu'on lui demande de tourner.
- **Le firmware RobotControl du prof est fait pour une autre carte moteur,** pilotée en I2C. Il utilise **D19** pour
  cette liaison, or D19 est ici le fil **m2b** : le moteur 2 reçoit donc des impulsions.

**Remède, dans nos deux programmes :** la toute première ligne de `setup()` met les 8 broches en sortie, à l'état BAS.

**Limite matérielle :** pendant les ~0,3 s du démarrage, avant que le programme tourne, une roue peut encore
tressaillir (D15 et D5 sont imposées par la puce). Pour l'éviter complètement, il faudrait des résistances de
rappel à la masse (10 kΩ) sur ces deux fils. Ce n'est pas nécessaire pour le TP.

## Étape 0 : sécurité

- **Robot posé sur une boîte, roues dans le vide,** pour tous les essais de l'étape 1.
- **Interrupteur de la batterie accessible.** En cas de problème, on coupe.
- **Si le moniteur série d'Arduino est ouvert, LidarLab ne peut pas utiliser le câble.** Pas de souci pour le Wi-Fi.

## Étape 1 : arrêter les moteurs et identifier les fils

1. Arduino IDE → ouvrir `firmware/01_Moteurs_Test/01_Moteurs_Test.ino`.
2. Outils → Carte → **ESP32 Dev Module** ; Port → celui de l'ESP32 ; **Téléverser**.
   - **Attendu :** les moteurs s'arrêtent dès que le programme démarre.
3. Outils → **Moniteur série**, à **115200** bauds. Le menu s'affiche.
4. Tape `1` puis Entrée : le fil m3a (D15) est mis à l'état haut pendant 0,8 s. Note **quelle roue** tourne et
   **dans quel sens** (vers l'avant ou l'arrière du robot). Recommence avec `2` … `8`.
5. Remplis ce tableau, il sert à l'étape 2 :

| Touche | Fil | Broche | Roue qui tourne | Sens |
|---|---|---|---|---|
| 1 | m3a | D15 | arrière gauche | avance |
| 2 | m3b | D2 | arrière gauche | recule |
| 3 | m4a | D4 | arrière droite | recule |
| 4 | m4b | D16 | arrière droite | avance |
| 5 | m1a | D17 | avant droite | recule |
| 6 | m1b | D5 | avant droite | avance |
| 7 | m2a | D18 | avant gauche | avance |
| 8 | m2b | D19 | avant gauche | recule |

*Relevé fait sur notre robot le 04/11. Résultat : M1 = avant droite, M2 = avant gauche, M3 = arrière gauche, M4 =
arrière droite. Sur les roues de droite, c'est le fil « b » qui fait avancer. Ce tableau est déjà recopié dans
`02_LidarRobot_WiFi/moteurs.h`.*

- **Normalement,** chaque paire (m1a/m1b, …) commande **la même roue**, une fois dans un sens et une fois dans l'autre.
- **Vérification :** `a` `b` `c` `d` font tourner M1 M2 M3 M4 dans le sens « A », et `A` `B` `C` `D` dans l'autre sens.
- **`0`** arrête tout.

## Étape 2 : le robot complet

1. Ouvrir `firmware/02_LidarRobot_WiFi/moteurs.h` et **remplir le tableau `WHEELS`** avec l'étape 1 :
   - pour chaque position (avant gauche, avant droite, arrière gauche, arrière droite), mettre la broche A et la
     broche B de la roue qui s'y trouve ;
   - mettre `invert = true` si, quand on demande « avancer », cette roue tourne vers l'arrière.

   Pour notre robot, c'est déjà fait : relevé du 04/11 ci-dessus.
2. Dans `02_LidarRobot_WiFi.ino`, **donner un nom unique au Wi-Fi** (`SSID`) s'il y a plusieurs robots dans la salle,
   par exemple `"LD06-Togola"`.
3. Ouvrir `02_LidarRobot_WiFi.ino` → **Téléverser**. Moniteur série à 115200 : `Wi-Fi "…" pret : http://192.168.4.1`.
4. **Premier essai, roues dans le vide :** sur le téléphone, maintenir ↑. Les 4 roues doivent tourner vers l'avant.
   Sinon, corriger `WHEELS` (broches ou `invert`), puis téléverser à nouveau.
5. **Régler une roue à part,** depuis le navigateur du PC ou du téléphone : `http://192.168.4.1/wheel?w=0&v=150`
   fait tourner la roue avant gauche (w = 0 AVG, 1 AVD, 2 ARG, 3 ARD ; v négatif = sens inverse).

## Étape 3 : piloter depuis le téléphone (rien à télécharger)

1. Sur le téléphone : Réglages Wi-Fi → rejoindre **RobotESP32** (ou ton nom), mot de passe `12345678`. Si le téléphone
   dit « pas d'Internet », choisir de **rester connecté**.
2. Ouvrir le navigateur (Chrome, Safari…) à l'adresse **http://192.168.4.1**.
3. La télécommande s'affiche, avec le scan du lidar en direct en haut :
   - **maintenir** une flèche fait rouler le robot, **lever le doigt** l'arrête ;
   - ⟲ ⟳ le font tourner sur place, ← → le font glisser de côté (roues Mecanum) ;
   - le curseur règle la vitesse.
   - **Sécurité :** chaque ordre ne dure que 0,4 s. Si le téléphone perd le Wi-Fi, le robot s'arrête tout seul.

## Étape 4 : avec LidarLab sur le PC

1. Le PC rejoint le même Wi-Fi que le robot. Le téléphone peut rester connecté en même temps.
2. `LidarLab.bat` → étape **1 Récolter** → **Wi-Fi** → `192.168.4.1` → **Connecter** : le scan en direct s'affiche.
3. Le pavé de pilotage (flèches, I K J L U O, Espace = stop, durée et vitesse) commande le robot.
4. Étape **5 Localiser** → **Robot en direct** → **Suivre le robot** : il se replace sur la carte de référence à
   chaque tour du lidar.

## Interface HTTP (pour aller plus loin)

| Adresse | Effet |
|---|---|
| `/` | télécommande pour téléphone |
| `/forward` `/backward` `/left` `/right` `/rotateleft` `/rotateright` `?time=ms` | un mouvement (2 s au plus), puis arrêt automatique |
| `/stop` | arrêt immédiat |
| `/speed?v=60..255` | vitesse des mouvements |
| `/wheel?w=0..3&v=-255..255&time=ms` | une seule roue (réglages) |
| `/scan` | dernier tour du lidar, en binaire (même format que RobotControl) |
| `/status` | JSON : points, tours par minute, vitesse, en mouvement ou non |
| `/pose?x=mm&y=mm&z=cdeg`, `/ping` | comme RobotControl |
