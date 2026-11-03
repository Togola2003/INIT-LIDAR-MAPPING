# Cartographie & localisation avec un lidar LD06

Projet du module **« Cartographie et localisation appliquée à la robotique mobile »** (EIGSI, prof. J. Quillès) :
exploiter les données d'un lidar LD06 monté sur un robot Mecanum (ESP32) pour cartographier la salle de classe,
puis y localiser le robot.

![Carte de la salle](TP/Jour1/figures/fig8_carte_finale.png)

## Où en est le projet

| Grande ligne de l'énoncé | État | Preuve |
|---|---|---|
| Afficher les données du lidar en nuages de points | ✅ | LidarScope, LidarStudio, [Fig. 1](TP/Jour1/figures/fig1_nuages_poses_manuelles.png) |
| Construire une carte d'occupation (grid map) | ✅ | [Fig. 4-5](TP/Jour1/figures/fig5_gridmap_mise_a_jour.png), `src/lidar/gridmap.py` |
| Nuages de points → segments → carte vectorielle | ✅ | [Fig. 6-7](TP/Jour1/figures/fig7_carte_vectorielle.png), `segments.py`, `vectormap.py` |
| Localisation sur grid map / carte vectorielle / scan de référence | 🟡 | ICP entre scans prêt ([Jour 2](TP/Jour2/README.md)) |
| Cartographier la salle et y localiser le robot | 🟡 | première carte complète (16 scans réels) |
| Utiliser git | ✅ | commits réguliers, [JOURNAL.md](JOURNAL.md), tags par jour |

**Comptes rendus :** [TP/](TP/README.md) (un dossier par jour). **Journal de bord :** [JOURNAL.md](JOURNAL.md).

## Les outils

| Outil | Pour quoi faire | Lancer |
|---|---|---|
| **LidarScope** | relier le robot (port série), voir le scan en direct, capturer des scans colorés avec leur pose, exporter | double-clic `LidarScope.bat` ou `python -m lidar.scope` |
| **LidarStudio** | cartographie pas à pas selon le cours : données → prétraitement → recalage (manuel + ICP) → grid map → carte vectorielle → export | double-clic `LidarStudio.bat` ou `python -m lidar.studio fichier.lidar.json` |
| Modules en ligne de commande | `lidar.acquire`, `lidar.viewer`, `lidar.gridmap`, `lidar.vectormap`, `lidar.simulate` | voir l'en-tête de chaque fichier |
| Initiation | comprendre le LD06 octet par octet : trame, UART, angles, tour complet, repères | [lecons/](lecons/README.md) |

## Installation (Windows)

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e .
pip install pytest
python -m pytest            # 74 tests
```

## Données

| Fichier | Contenu |
|---|---|
| `data/salle/session_salle_2026-11-02.lidar.json` | 16 scans réels de la salle, poses mesurées à la main, lignes série brutes |
| `data/salle/session_salle_recalee.lidar.json` | les mêmes scans, poses corrigées par ICP |
| `data/simu/`, `data/exercice8/` | scans simulés (vérité connue) |

Le format `*.lidar.json` est **auto-décrit** : unités (m, °), sens des angles, repères, formule de la pose et
description de chaque colonne sont dans le fichier. Les mesures brutes ne sont jamais filtrées. Avec le programme
Arduino d'exemple (`printScanTeleplot`), l'intensité n'est pas transmise (`null`). Pour l'avoir :
`ld06.printScanCSV(Serial);`.

## Organisation

```
TP/              comptes rendus par jour, figures, résultats, conclusion (livrable)
JOURNAL.md       traçabilité : quoi, quand, quel commit
src/lidar/       geometry, scan, gridmap, segments, extraction, vectormap, matching (ICP), simulate, ...
src/lidar/scope/ LidarScope (acquisition en direct) + briques communes (session, export, rendu)
src/lidar/studio LidarStudio (cartographie pas à pas)
lecons/          initiation au LD06, une leçon = un script
firmware/        programme ESP32 qui envoie les scans en CSV complet
data/            scans réels et simulés
tests/           tests pytest (dont les cas limites de la découpe de segments)
```

Les supports de cours (`RESSOURCES/`, `Cartographie-*/`, `Localisation-*/`, PDF) restent en local et ne sont pas versionnés.
