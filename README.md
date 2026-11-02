# TP LIDAR LD06 : cartographie

Outils Python du module « Cartographie et localisation appliquée à la robotique mobile » :
acquisition de scans LD06, visualisation multi-scans, grid map, extraction de segments et carte vectorielle.

La démarche détaillée (choix, hypothèses, limites) est dans [docs/jour1.md](docs/jour1.md).

## Installation (Windows, PowerShell)

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e .
pip install pytest
```

## Utilisation rapide

| Étape | Commande |
| --- | --- |
| **LidarScope** : direct, captures colorées, import/export, analyse | `python -m lidar.scope` |
| 1. Acquisition (ESP32 flashé avec `firmware/ESP32_LD06_CSV`) | `python -m lidar.acquire --port COM5 --count 3 --pose 0,0,0 --out data/scans/pos1` |
| 2. Visualisation / recalage manuel | `python -m lidar.viewer "data/scans/*.csv"` (touche `s` : sauver les poses) |
| 3. Grid map : création | `python -m lidar.gridmap build "data/scans/*.csv" --res 0.05` |
| 3. Grid map : mise à jour | `python -m lidar.gridmap update out/grid.npz data/scans/autre.csv` |
| 4-5. Carte vectorielle | `python -m lidar.vectormap "data/scans/*.csv" --grid out/grid.npz` |
| Sans robot | `python -m lidar.simulate --out data/simu` |
| Tests | `python -m pytest` |

## Organisation

```
src/lidar/      geometry, scan (formats CSV), acquire, simulate, viewer, gridmap, segments, extraction, vectormap
firmware/       sketch ESP32 qui envoie les scans en CSV
data/scans/     scans réels acquis en salle (versionnés)
data/simu/      scans simulés
docs/           comptes rendus par jour
tests/          tests pytest (dont les cas limites de la découpe de segments)
out/            cartes générées (non versionnées)
```

Les supports de cours (`RESSOURCES/`, `Cartographie-*/`, `Localisation-*/`, PDF) restent en local et ne sont pas versionnés.
