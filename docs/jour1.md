# TP Jour 1 : extraction, visualisation et cartographie

> **Version initiale (02/11).** Le compte rendu à jour, avec les résultats sur les scans réels, est
> [TP/Jour1/COMPTE_RENDU.md](../TP/Jour1/COMPTE_RENDU.md).

Ce document explique la démarche suivie à chaque étape : choix techniques, hypothèses, difficultés et limites.
Le code est dans `src/lidar/`, les tests dans `tests/`.

Conventions communes (identiques aux outils du module) : distances en mètres, angles en degrés dans les fichiers,
sens trigonométrique (antihoraire), 0° vers +X. Une pose est `(x, y, θ)` du lidar dans le repère de la carte.

---

## 1. Extraction des données lidar

### Principe
Le LD06 envoie en UART (230 400 bauds) des trames de 47 octets : en-tête `0x54 0x2C`, vitesse, angle de début,
12 mesures (distance en mm + intensité), angle de fin, horodatage, CRC8. La bibliothèque `LD06` fournie :

1. vérifie l'en-tête et le CRC ;
2. interpole les 12 angles : `Δθ = ((fin − début + 360) % 360) / 12`, puis `θn = début + (n + 0,5)·Δθ` ;
3. convertit le sens horaire du capteur en sens trigonométrique : `θ_math = 360 − θ_LD06` ;
4. accumule les trames et détecte le passage 360° → 0° pour livrer un scan complet (~450 points).

### Ce que nous avons fait
- `firmware/ESP32_LD06_CSV/ESP32_LD06_CSV.ino` : l'exemple fourni, où `printScanTeleplot` est remplacé par
  `printScanCSV`. Chaque scan part sur le port série (115 200 bauds) au format
  `N,Angle(°),Distance(mm),Intensity`, et une ligne vide sépare deux scans.
- `python -m lidar.acquire --port COM5 --count 3 --pose 0.5,1,90 --out data/scans/pos1` : lit le port série,
  découpe les scans sur les lignes vides, **jette le premier scan** (souvent incomplet à la connexion) et ceux de
  moins de 100 points, puis enregistre chaque scan dans un CSV `angle_deg,distance_m,intensity`, avec la pose mesurée
  en commentaire. C'est le même format que l'export de RobotControl, donc les deux sources sont compatibles.
- `lidar/scan.py` relit nos CSV, ceux de RobotControl, ceux de MapTool (`r_m`) et la sortie brute `printScanCSV` (en mm).

### Hypothèses et limites
- Le robot est **immobile** pendant un scan : un tour dure environ 0,15 s, mais le format ne contient pas d'horodatage par point.
- La pose saisie au moment de l'acquisition est mesurée à la main (mètre ruban). Elle est imprécise et sera corrigée
  dans le visualiseur (étape 2).
- `distance = 0` signifie « pas d'écho ». Ce n'est pas un obstacle à l'origine, donc ces points sont retirés.
- Point à vérifier en salle : l'orientation du montage. Placer un obstacle devant le robot (+X) et vérifier qu'il
  apparaît bien vers +X. Sinon, utiliser `setUpsideDown` dans le firmware.

### À faire en salle (données personnelles)
Faire au moins 3 à 5 scans depuis des positions différentes de la salle, en notant chaque fois la pose (x, y, θ)
par rapport à un coin de référence. Ranger les fichiers dans `data/scans/`.

---

## 2. Visualisation des nuages de points

`python -m lidar.viewer data/scans/*.csv`

- Chaque scan est converti en cartésien (`x = r cos θ`, `y = r sin θ`), puis placé dans le repère carte :
  `P_global = R(θ_r)·P_local + T(x_r, y_r)` (`geometry.py`).
- Une couleur par scan (palette `tab10`). Un rond et un trait de 20 cm montrent la position et l'orientation du robot.
- **Déplacement indépendant de chaque scan** : on choisit le scan dans la liste, puis on règle les curseurs x / y / θ,
  ou on utilise le clavier (flèches ±5 cm, `a`/`e` ±2°, Maj = pas ×5).
- `s` enregistre les poses recalées dans `data/poses.json`. Les outils de cartographie relisent ce fichier, ce qui
  remplace le recalage automatique (scan matching) que nous n'avons pas encore fait.

Choix : matplotlib, parce qu'il est disponible partout, sans compilation, et que ses widgets suffisent. Limite : l'outil
devient lent au-delà d'une cinquantaine de scans.

---

## 3. Grid map 2D

`python -m lidar.gridmap build data/scans/*.csv --res 0.05` crée une carte à partir de scans et de leurs poses.
`python -m lidar.gridmap update out/grid.npz data/scans/nouveau.csv` met à jour une carte existante.

Étapes suivies, d'après le document de cours :

1. **Grille** : la taille est calculée pour englober tous les points et les poses, plus une marge de 30 cm.
   Résolution par défaut : 5 cm (environ 5 fois le bruit du LD06, et des cellules assez fines pour une porte).
   Le tableau est `(ny, nx)`, indexé `[j, i]`, avec `i = ⌊(x − x0)/res⌋` et `j = ⌊(y − y0)/res⌋`.
2. **Propriété des cellules** : probabiliste, en **log-odds** `L = ln(p/(1−p))`. L'état initial est **inconnu**
   (L = 0, p = 0,5). Avec le log-odds, la mise à jour bayésienne se réduit à une addition. L'export en `uint8`
   (0 libre, 127 inconnu, 255 occupé) est disponible via `to_uint8()`.
3. **Prétraitement** : on retire les points sans écho, hors portée [0,02 ; 12] m ou d'intensité trop faible
   (seuil réglable avec `--min-intensity`).
4. **Cellules traversées** : on trace le rayon de la cellule du robot à la cellule d'impact avec **Bresenham**
   (entier, sans trou ni doublon), sans la dernière cellule, puis `L += L_FREE` (−0,4).
5. **Cellule d'impact** : `L += L_OCC` (+0,85).
6. `L` est borné à [−4 ; 4]. Sans cette borne, un mur vu 100 fois ne pourrait plus jamais disparaître.

Détails de mise en œuvre :
- Une cellule traversée par plusieurs rayons **du même scan** n'est décrémentée qu'une fois. Près du robot, des
  dizaines de rayons passent par les mêmes cellules, qui deviendraient « certaines » après un seul scan.
- Une cellule d'impact n'est pas libérée par un autre rayon du même scan qui la frôle. Sans cette règle, les murs
  vus en incidence rasante disparaissaient.
- `update_grid` **renvoie une nouvelle grille** et ne modifie pas la grille d'entrée, ce qui permet de comparer avant/après.

Extension : `decay(grid, facteur)` ramène toutes les cellules vers l'inconnu. Appliquée entre deux scans, elle donne une
durée de vie aux cellules : les obstacles qu'on ne revoit plus s'effacent.

**Limites** : la grille ne s'agrandit pas (avec `update`, les points hors grille sont ignorés), le modèle de capteur
ignore l'incertitude de position, et la qualité dépend entièrement de la justesse des poses.

---

## 4. Segments (`segments.py`, `extraction.py`)

### Nuage de points → segments
1. **Segmentation** : on parcourt les points dans l'ordre angulaire. Si deux points successifs sont séparés de plus de
   `5 cm + 5 % × distance`, on change de cluster. Le seuil est adaptatif parce que l'écart entre deux points voisins
   augmente avec la distance (environ 0,8° × r). On démarre après une rupture pour ne pas couper un objet à cheval sur 0°.
2. **Split** (Douglas-Peucker) : on coupe le cluster au point le plus éloigné de la corde tant que cet écart dépasse 3 cm.
   Une forme en L donne ainsi deux morceaux.
3. **Ajustement** par régression **orthogonale** (ACP / SVD). Contrairement à `y = ax + b`, elle reste valable pour
   les murs verticaux. Les extrémités sont les projections des points extrêmes sur la droite.
4. **Merge** des segments consécutifs proches, puis rejet des segments de moins de 6 points ou de moins de 10 cm.

### Opérations
| Fonction | Définition retenue |
| --- | --- |
| `angle_between` | angle entre droites support, dans [0°, 90°] (un segment n'a pas de sens : 1° et 179° font 2°) |
| `are_close` | angle ≤ 10° **et** extrémités du plus court à ≤ 5 cm de la droite du plus long **et** recouvrement ou écart ≤ 10 cm |
| `merge` | direction = moyenne pondérée par les longueurs (en doublant les angles pour gérer 0°/180°), droite passant par le barycentre pondéré, extrémités = projections extrêmes des 4 points |
| `intersection` | forme paramétrique `P1 + t·v1 = P2 + u·v2` ; il faut t et u dans [0, 1] à EPS près ; parallèles et colinéaires renvoient `None` |
| `cut` | voir ci-dessous |

### Découpe : règles et tolérance
`EPS = 1e-6 m`. Cette valeur est très inférieure au bruit du capteur (~1 cm) et très supérieure aux erreurs d'arrondi.
- Il faut une intersection **sur les deux segments**. Si la droite du coupeur traverse la cible mais que le coupeur
  s'arrête avant, rien ne change.
- Elle doit être **strictement à l'intérieur** de la cible, c'est-à-dire à plus de EPS des deux extrémités. Une
  intersection à une extrémité ne change rien.
- Segments **colinéaires** ou parallèles : pas de découpe.
- On garde le **plus long** côté. Si les longueurs sont **égales à EPS près**, on garde le côté qui contient **p1**.

Chaque cas limite est testé dans `tests/test_segments.py`.

---

## 5. Carte vectorielle

`python -m lidar.vectormap data/scans/*.csv --grid out/grid.npz` (la grid map est affichée en fond pour comparer).

D'après le document « Construction d'une carte vectorielle 2D », pour chaque scan :

1. **Représentation** : liste de *features* = segment + confiance + dernier scan où elle a été vue. Elle est exportée
   au format du module (`x1,y1,x2,y2` par ligne).
2. **Prétraitement et passage en repère carte** : comme pour la grid map.
3. **Détection d'absence** (§5 du cours) : chaque faisceau robot → impact, raccourci de 10 cm pour ignorer le bruit
   autour de l'impact, est un segment de vide. Si au moins 3 faisceaux coupent une feature, elle est découpée avec
   `cut` (on garde le plus long côté) et sa confiance baisse de 1. Cette étape se fait **avant** l'ajout des nouvelles
   observations : sinon, un mur revu en partie (porte ouverte) serait fusionné puis jamais raccourci.
4. **Extraction** des segments du scan (étape 4).
5. **Association** : un segment `are_close` d'une feature existante est fusionné avec elle (`merge`), et la confiance
   augmente. Sinon, il est ajouté avec une confiance de 1.
6. **Nettoyage** : fusion des features devenues redondantes, suppression des features de confiance nulle ou de moins de 10 cm.

Sur la pièce simulée (3 scans), on retrouve les 4 murs et les 3 faces visibles de la caisse. La 4ᵉ face n'est vue
depuis aucune pose, donc elle est absente, ce qui est correct.

**Limites** : la découpe par « plus long côté » est grossière. Une porte ouverte au milieu d'un mur supprime une moitié
entière, alors qu'il faudrait scinder le mur en deux. On pourrait aussi pondérer la fusion par la confiance. Comme pour
la grid map, tout repose sur la qualité des poses : sans scan matching, il faut les recaler à la main.

---

## Tester sans robot

```bash
python -m lidar.simulate --out data/simu        # 3 scans d'une pièce de 4 m × 3 m
python -m lidar.viewer "data/simu/*.csv"
python -m lidar.gridmap build "data/simu/*.csv"
python -m lidar.vectormap "data/simu/*.csv" --grid out/grid.npz
python -m pytest
```
Les scans simulés valident les outils. Ils **ne remplacent pas** les scans réels demandés par le TP.
