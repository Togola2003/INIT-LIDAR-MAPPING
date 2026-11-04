# Journal de bord du projet

Traçabilité demandée par l'énoncé : ce qui a été fait, quand, dans quel commit, et ce qui n'a pas marché du
premier coup. Liste complète des commits : `git log --oneline`. Versions de fin de journée : `git tag`
(`jour1`, `jour2`).

## Jour 1 : lundi 02/11/2026

| Heure | Étape | Commit |
|---|---|---|
| 11:25 | Dépôt créé. Premiers outils : acquisition série, visualiseur multi-scans, grid map, segments, carte vectorielle, 46 tests | `1e839dd` |
| 12:01 → 13:30 | **Initiation** (`lecons/`) : octets et trame LD06, UART et débits, recherche de trames dans le flux, 12 angles, tour complet, repères, procédure en salle, exercice de recalage | `0078880` → `4018fd6` |
| 15:55 | **LidarScope** v1 : visualisation en direct (style Teleplot), captures colorées | `069ba35`, `0f5941a` |
| 16:21 | LidarScope v2 : interface moderne, filtres, format `.lidar.json` auto-décrit | `793c93b` |
| 17:07 | LidarScope v3 : onglet Cartographie selon le cours, export au choix (chaque scan / ensemble), fidélité Teleplot | `108eaf3` |
| après-midi | **Acquisition en salle** : 16 scans réels avec LidarScope (export du 02/11 à 21:02) | données ajoutées le 03/11 |

## Jour 1 (fin) : mardi 03/11/2026, matin

| Heure | Étape | Commit |
|---|---|---|
| 07:07 | Scans réels ajoutés (`data/salle/`), recalage **ICP**, filtre de voisinage, étapes intermédiaires de la carte vectorielle | `afff321` |
| 07:12 | **LidarStudio** : cartographie hors ligne en 6 étapes du cours | `75ad0ec` |
| matin | Compte rendu du Jour 1 avec les figures des scans réels, dossier `TP/`, ce journal, tag `jour1` | `2ca54fe` |
| 10:47 | Outils plus rapides : arbre k-d (ICP 13,7 s → 0,17 s), **recalage global** de toutes les poses (6,4 → 2,2 cm) | `045ec34` |
| 10:47 | Scans déplacés et tournés à la souris (plusieurs à la fois, Ctrl+Z), capture par récolte de N tours avec aperçu | `09732c4` |
| 10:47 | Dépôt organisé en deux parties : outils / réalisation du TP | `efd841c` |

## Jour 2 : mardi 03/11/2026, localisation

**Démarche suivie (celle du prof) :** une méthode d'abord, on regarde ses limites, puis on améliore, avec un commit
par étape.

| Heure | Étape | Commit |
|---|---|---|
| 10:00 → 13:45 | Nouvelle acquisition en salle (`printScanCSV`, avec intensité), recalage à la main ; export de 13:45 retenu (écart entre scans 5,2 → 3,0 cm) | — |
| 14:10 | Nettoyage (doublons `aa/`, sorties régénérables), jeu de données `data/salle_jour2/`, plan du Jour 2 | `f68ace9` |
| 15:13 | **Étape 1, grid map :** champ de vraisemblance. Gradient seul (précis mais coincé quand on part loin), puis exhaustive et multi-résolution (robustes), puis la combinaison | `9f24a06` |
| 15:14 | **Étape 2, carte vectorielle :** distance point → segment, Gauss-Newton et calcul direct | `f503a05` |
| 15:14 | **Étape 3, entre scans :** ICP point à point et point à droite | `e90f6d9` |
| 15:14 | **Étape 4, banc d'essai :** 6 240 localisations, mêmes scans et mêmes départs, simulation + réel, critère de zone couverte | `6239762` |
| 15:14 → 15:55 | Compte rendu du Jour 2, puis précision : localisation faite hors ligne sur les scans enregistrés | `51d94de`, `beeac38` |
| 16:07 → 16:17 | **LidarLab**, l'application unique en 6 étapes (récolter, nettoyer, recaler, cartes, localiser, exporter) | `b30a1a4` → `6d80563` |
| 17:33 | Retouches des murs appliquées aux **deux** cartes ; carte de référence validée et figée | `e39c228` |
| 17:34 | **Localisation globale** sans pose de départ (idée de l'appli ld06-slam d'un camarade, réécrite en Python) | `c20aafb` |
| 17:35 | **Suivi en temps réel** + robot de démo qui roule dans la carte | `20c2a58` |
| 17:41 | Outils toujours visibles au-dessus de la carte ; étape Localiser refaite | `287f548` → `99abc50` |
| 18:52 → 18:56 | **Robot en Wi-Fi** (protocole RobotControl : `/scan` binaire, `/forward?time=…`) : lecture et pilotage | `b0a74b1` → `911e26b` |
| 19:49 | Localiser **comme demandé** : la carte reste fixe, un scan libre est glissé / tourné, puis « Localiser » le ramène dessus ; recherche globale plus sûre | `e4f27e7` → `1caf243` |
| soir | Tag `jour2` | — |

### Problèmes rencontrés et solutions (Jour 2)

| Problème | Cause | Solution | Commit |
|---|---|---|---|
| Le gradient seul échoue dès 50 cm / 20° d'erreur de départ (0 % à 1 m) | il suit la pente depuis le départ et s'arrête sur un **minimum local** (un autre mur) | multi-résolution d'abord, gradient ensuite : 99 % | `9f24a06` |
| Les scans réels 1, 2 et 5 ne se localisent pas (≈ 77 % de réussite au total) | ils voient surtout des **meubles proches** ; sans eux, leur coin de salle n'est pas dans la carte | critère explicite de **zone couverte** (≥ 35 % des points près d'un mur), résultats donnés pour les deux cas | `6239762` |
| Dans l'appli, « Localiser » laissait un scan vert « à côté » de la carte | la pose de départ se déplaçait **à chaque clic sur la carte** ; départ à 5-10 m, alors que les méthodes cherchent à ± 1 m | localisation **globale** sans pose de départ, plus de clic piège sur la carte | `c20aafb`, `f6a822a` |
| Un mur tracé à la main n'existait que dans la carte vectorielle | les retouches remplaçaient la liste de segments | une retouche = une **opération** rejouée sur la grid map **et** les murs | `e39c228` |
| Un scan importé posé à l'envers mais annoncé « fiable » | la recherche globale ne testait que les orientations tirées des murs | si moins de 60 % des points sont sur les murs, on teste **toutes** les orientations | `e4f27e7` |
| L'utilisateur ne comprenait pas l'enchaînement de l'étape Localiser | l'interaction ne correspondait pas à son idée : carte fixe + scan libre qu'on déplace puis qui revient | étape refaite : scan libre orange → ◎ Localiser → animation, vert brillant | `e6ca97d` |

### Reste à faire, et à tester en salle

- **Robot réel en Wi-Fi :** testé seulement contre un faux robot qui répond comme l'ESP32. À essayer le 04/11.
- **Méthode probabiliste** (filtre de particules) : non faite (facultative).

## Jour 3 : mercredi 04/11/2026

| Heure | Étape | Commit |
|---|---|---|
| matin | Problème : les moteurs tournent tout seuls avec n'importe quel programme. Causes : fils moteurs en direct sur l'ESP32 (broches flottantes ou hautes au démarrage), et D19 utilisée par l'I2C du firmware RobotControl. **Firmware 01** (moteurs arrêtés + identification des fils) et **firmware 02** (lidar + moteurs + Wi-Fi + télécommande téléphone, compatible LidarLab), compilés tous les deux | `6458404` → `4abdfad` |
| matin | Test 01 sur le robot : relevé fil → roue (M1 avant droite, M2 avant gauche, M3 arrière gauche, M4 arrière droite ; sur les roues de droite, c'est le fil « b » qui fait avancer), recopié dans `moteurs.h` | voir `git log` |
| midi | Rotation inversée sur le vrai robot (« avancer » bon, « tourner à gauche » tourne à droite) : le relevé avait été fait robot vu de face, gauche et droite échangés. `moteurs.h` corrigé | `409a6e3` |
| après-midi | Suivi en direct robuste : prédiction par les commandes (odométrie), fenêtre de recherche qui grandit avec le mouvement, tours douteux ignorés, trajet coupé aux relocalisations ; `place_scan` | `e99c449` |
| après-midi | **Cartographie pas à pas** dans Récolter (chaque scan posé sur les précédents, recouvrement affiché, une couleur par scan, optimisation finale) ; suivi en arrière-plan (`app/live.py`), aussi dans Localiser. Essai simulé : scans posés à 2 cm près | `db6204c` |
| fin d'après-midi | Consigne du prof : **SLAM pas à pas** (scan = référence, petit déplacement, rescan localisé par les points communs, partie nouvelle ajoutée). Module `slam.py` (grid map puis ICP fin, commun / nouveau, boucles), interface centrée dessus, doc [docs/SLAM.md](docs/SLAM.md). Simulation : tour complet de 18 scans, chaque pose à ≤ 1 cm | `4128361`, `e4e1550` |
| 13 h 09 → 14 h 07 | ✅ **Essai en salle réussi** : SLAM pas à pas sur le vrai robot. 1er scan dans un coin, puis petits déplacements : chaque scan se localise et ajoute sa partie nouvelle. **93 scans**, toute la salle (≈ 12 × 9 m). Session : [data/salle_jour3/slam_salle_04-11.lidar.json](data/salle_jour3/slam_salle_04-11.lidar.json) | ce commit |
| fin de séance | Question du prof : la grid map est-elle faite une fois à la fin ou scan par scan ? Réponse (scan par scan, log-odds : traversée −, impact +, les scans qui se recouvrent renforcent les murs et effacent ce qui a bougé ; même résultat dans n'importe quel ordre) : [docs/SLAM.md](docs/SLAM.md), test `tests/test_gridmap.py`. **Version validée marquée par le tag `jour3`** | ce commit |

### Problèmes rencontrés et solutions (Jour 3)

- **Moteurs qui tournent seuls :** broches mises à 0 en tout premier dans `setup()`, et firmware sans la bibliothèque
  I2C du prof.
- **Rotation inversée :** côtés gauche et droit échangés dans `moteurs.h` (relevé fait de face).
- **Suivi qui « saute » à travers la salle :** chaque tour était cherché seul, et un tour flou pendant un virage
  donnait une pose fausse. Maintenant, la pose prévue par les commandes sert de départ, un tour douteux n'est pas
  tracé, et une recherche globale n'est acceptée que si elle est nette.
- **Suivi en retard, interface figée :** la localisation tournait dans l'interface. Elle tourne maintenant dans un fil
  à part, toujours sur le dernier tour reçu.
- **Suivi en direct qui se fige au bout d'un moment :** le ramasse-miettes de Python libérait une police de
  l'interface depuis le fil de calcul, et Tk (qui n'accepte qu'un seul fil) bloquait ce fil. Trouvé en affichant la
  pile du fil bloqué. Solution : polices partagées, ramasse-miettes lancé seulement par l'interface.
- **« Commande non reçue : timed out » :** le robot n'a qu'un petit serveur web, et les demandes de `/scan` et les
  commandes passent l'une après l'autre. Il suffit d'appuyer à nouveau ; chaque mouvement s'arrête de lui-même après
  sa durée (sécurité).

### Suite (compte rendu à rendre pour le 20/11)

- Compte rendu du Jour 3 et conclusion personnelle.
- Présentation et rapidité de l'application : améliorations possibles, **sans perdre la version validée** :
  le tag `jour3` pointe sur la version qui marche en salle (voir le README, « Versions »).
- Optionnel : mesurer la vitesse réelle (m/s et °/s) pour régler l'odométrie dans `robot.py`.

Plan : [TP/Jour3/README.md](TP/Jour3/README.md).
