# Plan d'apprentissage : une petite étape à la fois

Chaque leçon = une idée + un script à lancer + une question à se poser.

| # | Leçon | On manipule | Statut |
|---|---|---|---|
| 1 | Bits, octets, une trame LD06 de 47 octets, CRC | `01_trame.py` | ✅ |
| 2 | Le trajet des données : LD06 → fil UART → ESP32 → USB → PC (bauds, débits) | `02_debits.py` | ✅ |
| 3 | Retrouver une trame dans un flux d'octets (chercher `54 2C`, compter 47, vérifier le CRC) | `03_flux.py` | ✅ |
| 4 | Les 12 angles d'une trame (interpolation, sens horaire → trigo) | `01_trame.py` | ⬜ |
| 5 | Assembler un tour complet (détecter le passage 360° → 0°) | scan simulé | ⬜ |
| 6 | Polaire → cartésien, puis pose du robot (rotation + translation) | `lidar/geometry.py` | ⬜ |
| 7 | En salle : flasher l'ESP32, enregistrer ses propres scans | `firmware/`, `lidar/acquire.py` | ⬜ |
| 8 | Afficher plusieurs scans et les recaler à la main | `lidar/viewer.py` | ⬜ |
| 9 | Grid map : grille, cellules libres / occupées | `lidar/gridmap.py` | ⬜ |
| 10 | Segments : extraction, angle, fusion, intersection, découpe | `lidar/segments.py` | ⬜ |
| 11 | Carte vectorielle | `lidar/vectormap.py` | ⬜ |

Lancer une leçon : `.venv/Scripts/python lecons/01_trame.py`
