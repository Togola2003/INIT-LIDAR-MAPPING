# Cartographie & localisation avec un lidar LD06

Projet du module **« Cartographie et localisation appliquée à la robotique mobile »** (EIGSI, prof. J. Quillès) :
exploiter les données d'un lidar LD06 monté sur un robot Mecanum (ESP32) pour cartographier la salle de classe,
puis y localiser le robot.

![Carte de la salle](TP/Jour1/figures/fig8_carte_finale.png)

## Deux parties, un même but

Le dépôt contient **deux choses différentes** qui servent le même objectif : réussir le TP et le comprendre.

| | 🧰 **Partie A : les outils** | 📝 **Partie B : la réalisation du TP** |
|---|---|---|
| **C'est quoi** | Le logiciel que j'ai construit **avant et pendant** le TP : applications, bibliothèque Python, tests, leçons | **Mon travail de TP** : mes mesures en salle, mes résultats, mes comptes rendus jour par jour |
| **À quoi ça sert** | Se préparer, comprendre le capteur, vérifier les algorithmes sur des données à vérité connue, puis traiter les vraies données | Répondre aux attendus de l'énoncé, avec des chiffres et des figures reproductibles |
| **Où** | `src/lidar/`, `LidarScope.bat`, `LidarStudio.bat`, `tests/`, `lecons/`, `firmware/`, `data/simu/` | `TP/`, `data/salle/`, `JOURNAL.md` |
| **Point d'entrée** | [docs/OUTILS.md](docs/OUTILS.md) | [TP/README.md](TP/README.md) |

**Comment les deux parties s'enchaînent :**
1. **Préparer et comprendre** (partie A) : les leçons (`lecons/`) expliquent le LD06 octet par octet. Les tests
   (`tests/`, 81 tests) et les scans simulés (`data/simu/`) vérifient chaque algorithme sur une salle dont on connaît
   la vérité.
2. **Mesurer en salle** (partie B) : acquisition avec LidarScope, données brutes rangées dans `data/salle/`.
3. **Traiter et analyser** (A sur B) : LidarStudio et les scripts `TP/JourN/generer_figures.py` appliquent les
   outils à mes mesures.
4. **Rendre compte** (partie B) : un compte rendu par jour, où chaque chiffre est régénérable.

## Où en est le TP

| Grande ligne de l'énoncé | État | Preuve |
|---|---|---|
| Afficher les données du lidar en nuages de points | ✅ | LidarScope, LidarStudio, [Fig. 1](TP/Jour1/figures/fig1_nuages_poses_manuelles.png) |
| Construire une carte d'occupation (grid map) | ✅ | [Fig. 4-5](TP/Jour1/figures/fig5_gridmap_mise_a_jour.png), `src/lidar/gridmap.py` |
| Nuages de points → segments → carte vectorielle | ✅ | [Fig. 6-7](TP/Jour1/figures/fig7_carte_vectorielle.png), `segments.py`, `vectormap.py` |
| Localisation sur grid map / carte vectorielle / scan de référence | 🟡 | ICP et recalage global prêts ([Jour 2](TP/Jour2/README.md)) |
| Cartographier la salle et y localiser le robot | 🟡 | carte complète de 16 scans réels, recalée à 2,2 cm près |
| Utiliser git | ✅ | commits réguliers, [JOURNAL.md](JOURNAL.md), tags par jour |

## Démarrer

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e .
pip install pytest
python -m pytest
```

Puis double-clic sur **`LidarScope.bat`** (robot branché) ou **`LidarStudio.bat`** (traitement des scans de la salle).
Le mode d'emploi des deux applications est dans [docs/OUTILS.md](docs/OUTILS.md).

## Organisation

```
── Partie A : outils ──────────────────────────────────────────────────────────
src/lidar/          bibliothèque : geometry, scan, gridmap, segments, extraction, vectormap,
                    matching (ICP + recalage global), simulate, ...
src/lidar/scope/    LidarScope (acquisition en direct) + briques communes (carte, session, export)
src/lidar/studio/   LidarStudio (cartographie pas à pas)
tests/              tests pytest (dont les cas limites de la découpe de segments)
lecons/             initiation au LD06 : une leçon = un script
firmware/           programme ESP32 qui envoie les scans en CSV complet
data/simu/, data/exercice8/   scans simulés (vérité connue)
docs/               mode d'emploi des outils, notes d'algorithmes

── Partie B : réalisation du TP ───────────────────────────────────────────────
TP/                 comptes rendus par jour, figures, résultats, conclusion (livrable)
data/salle/         mes 16 scans réels de la salle (bruts + poses corrigées)
JOURNAL.md          traçabilité : quoi, quand, quel commit
```

Les supports de cours (`RESSOURCES/`, `Cartographie-*/`, `Localisation-*/`, PDF) restent en local et ne sont pas versionnés.
