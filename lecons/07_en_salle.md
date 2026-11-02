# Leçon 7 : la séance en salle (à cocher)

## A. Préparer l'ordinateur (une seule fois, possible avant la séance)
- [ ] Arduino IDE 2 installé
- [ ] **Fichier > Préférences**, URL des cartes : `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- [ ] **Gestionnaire de cartes** : installer « esp32 by Espressif Systems »
- [ ] **Croquis > Inclure une bibliothèque > Ajouter la bibliothèque .ZIP** : `RESSOURCES/LD06/Librairie/LD06.zip`
- [ ] Noter les versions (cœur ESP32 et bibliothèque) pour le compte rendu

## B. Téléverser le programme
- [ ] Ouvrir `firmware/ESP32_LD06_CSV/ESP32_LD06_CSV.ino`
- [ ] **Outils > Type de carte** : « ESP32 Dev Module »
- [ ] **Outils > Port** : débrancher puis rebrancher l'ESP32 et repérer le port qui apparaît (ex. `COM5`)
- [ ] Téléverser (maintenir **BOOT** si l'envoi ne démarre pas)
- [ ] Ouvrir le **Moniteur série à 115200** : on doit voir défiler des lignes `N,angle,distance,intensité`
- [ ] **Fermer le moniteur série.** Un seul programme à la fois peut utiliser le port : sinon Python affichera « accès refusé ».

## C. Test d'orientation (la question de la leçon 4)
- [ ] Poser un objet à environ 50 cm **devant** le robot, puis lancer :
  `.venv/Scripts/python -m lidar.acquire --port COM5 --count 1 --out data/scans/test_devant`
  et `.venv/Scripts/python lecons/07_verif_scan.py data/scans/test_devant_01.csv`
  → l'obstacle le plus proche doit être vers **0°**
- [ ] Même chose avec l'objet **à gauche** → l'obstacle le plus proche doit être vers **90°**
  (si on lit 270°, la carte est en miroir : `ld06.setUpsideDown(true)` dans le firmware)

## D. Les scans pour la carte
- [ ] Choisir un **coin de référence** de la salle = origine (0, 0). Axe X le long d'un mur, axe Y le long de l'autre.
- [ ] Pour chaque position (3 à 5 positions réparties dans la salle) :
  - poser le robot, mesurer au mètre **x** et **y** du centre du lidar, estimer l'angle **θ** de l'avant du robot par rapport à l'axe X
  - `.venv/Scripts/python -m lidar.acquire --port COM5 --count 3 --pose x,y,θ --out data/scans/posN`
  - vérifier avec `lecons/07_verif_scan.py` (≈ 450 points, distances cohérentes avec le mètre)
- [ ] Faire une **photo / un croquis** de la salle avec les positions : utile pour le compte rendu
- [ ] Ne pas bouger pendant un scan (personne devant le lidar si possible)

## E. Sauvegarder sur GitHub
```bash
git add data/scans
git commit -m "data: scans réels de la salle"
git push
```

## Si ça ne marche pas
| Symptôme | Cause probable |
|---|---|
| Rien dans le moniteur série | mauvais port, mauvaise vitesse (115200), lidar non alimenté (5 V), fil TX pas sur GPIO33 |
| Caractères bizarres | moniteur pas à 115200 |
| Python : « accès refusé » / « PermissionError » | le moniteur série Arduino est encore ouvert |
| Python attend sans rien écrire | le lidar ne tourne pas, ou mauvais port |
| Beaucoup de points « sans écho » | surfaces noires / vitres, ou objets à plus de 12 m |
