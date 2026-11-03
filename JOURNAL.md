# Journal de bord du projet

Traçabilité demandée par l'énoncé : ce qui a été fait, quand, et dans quel commit.
Liste complète des commits : `git log --oneline` ; versions de fin d'étape : `git tag`.

## Jour 1 : lundi 02/11/2026

| Heure | Étape | Commit |
|---|---|---|
| 11:25 | Dépôt créé. Premiers outils : acquisition série, visualiseur multi-scans, grid map, segments, carte vectorielle, 46 tests | `1e839dd` |
| 12:01 → 13:30 | **Initiation** (`lecons/`) : octets et trame LD06, UART et débits, recherche de trames dans le flux, 12 angles, tour complet, repères, procédure en salle, exercice de recalage | `0078880` → `4018fd6` |
| 15:55 | **LidarScope** v1 : visualisation en direct (style Teleplot), captures colorées | `069ba35`, `0f5941a` |
| 16:21 | LidarScope v2 : interface moderne, filtres, format `.lidar.json` auto-décrit | `793c93b` |
| 17:07 | LidarScope v3 : onglet Cartographie selon le cours, export au choix (chaque scan / ensemble), fidélité Teleplot | `108eaf3` |
| après-midi | **Acquisition en salle** : 16 scans réels avec LidarScope (export du 02/11 à 21:02) | données ajoutées le 03/11 |

## Jour 1 (suite) : mardi 03/11/2026

| Heure | Étape | Commit |
|---|---|---|
| 07:07 | Scans réels ajoutés (`data/salle/`), recalage **ICP**, filtre de voisinage, étapes intermédiaires de la carte vectorielle | `afff321` |
| 07:12 | **LidarStudio** : cartographie hors ligne en 6 étapes du cours | `75ad0ec` |
| matin | Compte rendu du Jour 1 avec les figures des scans réels, dossier `TP/`, ce journal, tag `jour1` | voir `git log` |

| après-midi | **Outils plus simples et plus rapides** : déplacer / tourner les scans à la souris (plusieurs à la fois, Ctrl+Z), capture par récolte de N tours avec aperçu, arbre k-d (ICP 80× plus rapide), **recalage global** de toutes les poses (6,4 → 2,2 cm), dépôt organisé en deux parties (outils / réalisation du TP) | voir `git log` |

## Jour 2 : mardi 03/11/2026 (suite)

| Heure | Étape | Commit |
|---|---|---|
| 10:00 → 13:45 | Nouvelle acquisition en salle (`printScanCSV`, avec intensité) et recalage à la main dans les applis : export de 13:45 retenu (écart entre scans 5,2 → 3,0 cm par rapport à l'export de 12:41) | données ajoutées ensuite |
| 14:10 | Nettoyage (doublons `aa/`, sorties régénérables), jeu de données `data/salle_jour2/`, plan du Jour 2 | `f68ace9` |
| après-midi | **Localisation, étape 1 :** grid map, champ de vraisemblance. Gradient seul (précis mais coincé quand on part de loin), puis recherche exhaustive et multi-résolution (robustes), puis la combinaison | voir `git log` |
| après-midi | **Étape 2 :** carte vectorielle (Gauss-Newton, calcul direct). **Étape 3 :** entre scans (ICP point à point, point à droite) | voir `git log` |
| après-midi | **Étape 4 :** banc d'essai commun (6 240 localisations), critère de zone couverte, compte rendu du Jour 2 | `9f24a06` → `beeac38` |
| soir (3) | Localiser refaite selon le TP : clic sur un scan de la liste = localisation globale immédiate, fichier importé, robot en direct ; **robot en Wi-Fi** (scans + pilotage, protocole RobotControl) ; plus de clic piège sur la carte | voir `git log` |
| soir (2) | Retours de test : **localisation globale** sans pose de départ (idée reprise de l'appli ld06-slam d'un camarade), **suivi en temps réel** + démo qui roule dans la carte, **carte de référence validée**, retouches appliquées aux deux cartes, outils dans la barre de la carte | voir `git log` |
| soir | **LidarLab**, l'application unique : récolter, nettoyer (gomme), recaler, cartes (retouches des murs), **localiser** (référence + méthode au choix), exporter. Style clair, Nouveau projet, Ctrl+Z | voir `git log` |

## Jour 3 : à venir
