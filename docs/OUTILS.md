# Partie A : les outils

Le logiciel construit pour le TP. Il ne contient **aucun résultat de TP** : ceux-ci sont dans [../TP/](../TP/README.md).
Les outils se vérifient sur des données dont on connaît la vérité : les tests (`python -m pytest`) et les scans
simulés (`data/simu/`).

| Outil | Rôle | Lancer |
|---|---|---|
| **LidarLab** (application unique) | tout le TP dans une seule fenêtre : récolter, nettoyer, recaler, cartes (avec retouches), localiser, exporter | `LidarLab.bat` |
| **LidarScope** | relier le robot, voir le scan en direct, récolter des captures propres, les placer | `LidarScope.bat` |
| **LidarStudio** | traiter les captures selon les 6 étapes du cours et produire les cartes | `LidarStudio.bat` |
| Bibliothèque `src/lidar/` | les algorithmes, utilisables sans interface | `python -m lidar.gridmap …` |
| Leçons `lecons/` | comprendre le LD06 octet par octet avant de s'en servir | `python lecons/01_trame.py` |

LidarScope et LidarStudio sont les premières versions (Jour 1). LidarLab les réunit et les remplace pour la suite.

En ligne de commande, depuis le dossier du projet :

```powershell
.venv\Scripts\python.exe -m lidar.app
.venv\Scripts\python.exe -m lidar.scope
.venv\Scripts\python.exe -m lidar.studio "data\salle\session_salle_2026-11-02.lidar.json"
```

---

## LidarLab : l'application unique

**Barre d'outils au-dessus de la carte, toujours visible :**
- **⌫ Gomme :** glisser un rectangle efface les points de tous les scans visibles ;
- **✎ Tracer un mur**, **↔ Prolonger** (attraper une extrémité) et **✕ Effacer un mur.**

Chaque retouche est une opération rejouée sur **les deux cartes** : un mur tracé devient des cases occupées de la grid
map, et un mur effacé des cases libres. Il compte donc aussi pour l'export et la localisation. Échap arrête l'outil.

Une fenêtre, trois zones :
- **à gauche,** les 6 étapes et la liste des scans du projet (clic = sélectionner, Ctrl + clic = plusieurs,
  ◉ = afficher ou masquer) ;
- **au centre,** la carte ;
- **à droite,** les réglages de l'étape, avec en tête « Ce que dit le cours ».

| Étape | Ce qu'on y fait |
|---|---|
| 1 Récolter | câble USB ou démo (Wi-Fi : Jour 3). Clic sur la carte = pose du robot. **C** récolte N tours, puis aperçu : garder (Entrée), refaire ou jeter (Échap). Import de fichiers |
| 2 Nettoyer | filtres du cours (portée, intensité, points isolés). Suppr = supprimer les scans sélectionnés |
| 3 Recaler | glisser = déplacer, clic droit glissé = tourner, flèches / A E, plusieurs scans en bloc. **Recalage complet**, ICP en bloc, cohérence |
| 4 Cartes | grid map et carte vectorielle, **retouches comprises**. **✓ Valider cette carte comme référence** : elle est figée et sert à la localisation |
| 5 Localiser | **Un scan du projet** : sans pose de départ, recherche dans toute la carte (globale), puis affinage ; ou pose approximative (clic). **Le robot en direct** : suivi en temps réel du robot (ou de la démo qui roule dans ta carte). Avancé : référence, méthode, comparaison, contrôle sans le scan |
| 6 Exporter | projet `.lidar.json` (retouches comprises), dossier complet, image PNG, carte vectorielle `.txt` |

- **Toujours disponibles :**
  - **Nouveau projet** vide tout ;
  - **Ouvrir** ajoute des scans ;
  - **Ctrl+Z / Ctrl+Y** annule ou rétablit chaque action (80 niveaux) ;
  - **Ctrl+S** enregistre.
- **Un jeu de données ouvert n'est jamais écrasé :** le premier « Enregistrer » demande toujours où écrire.

**Architecture** (pour faire évoluer l'appli) :
- `app/project.py` : le projet, immuable et testé, sans interface ;
- `app/style.py` : le thème ;
- `app/tools.py` : les outils de la carte ;
- `app/steps/` : une étape par fichier ;
- `app/window.py` : la fenêtre, la sélection, la connexion et les caches.

Changer d'interface, par exemple pour une version web, ne touche ni au projet ni aux algorithmes.

## LidarScope : acquisition

1. **Connexion.** Choisis le port et le débit, puis **Connecter**. Sans robot, le bouton **Démo** simule une salle.
2. **Pose.** Onglet *Direct* : saisis la pose du robot (x, y, θ) mesurée en salle.
3. **Capture par récolte.** Appuie sur **C** :
   - LidarScope **récolte N tours** (10 par défaut), avec une barre de progression. Le robot doit rester immobile.
   - Appuie de nouveau sur C pour arrêter plus tôt. La récolte s'arrête aussi si plus rien n'arrive pendant 5 s.
   - Les tours sont combinés de deux façons au choix. **Médiane** : un point par angle, le bruit est réduit.
     **Tous les points** : le nuage est plus dense.
   - L'**aperçu** apparaît en jaune, sans le direct par-dessus, avec un bilan : tours reçus, points, bruit des
     distances (« bon » sous 2 cm). Ensuite **Garder** (Entrée), **Refaire**, ou **Jeter** (Échap).
   - En pause (Espace), C capture tout de suite les derniers tours reçus.
4. **Placer les captures** : onglet *Captures*, le bouton ✋ s'active tout seul (voir les gestes plus bas).
5. **Exporter** : onglet *Données*, au choix chaque scan et/ou l'ensemble. La session `.lidar.json` est
   auto-décrite, et LidarStudio la relit.

## LidarStudio : cartographie pas à pas

| Étape | Ce qu'on fait |
|---|---|
| 1 Données | choisir les scans utilisés (cases) et la sélection (clic, Ctrl + clic) |
| 2 Prétraitement | portée, intensité, filtre des points isolés (rayon qui grandit avec la distance) |
| 3 Recalage | déplacer et tourner à la souris, **⚡ Recalage complet**, ICP d'un groupe en bloc, mesure de cohérence |
| 4 Grid map | résolution, état initial, modèle, probabilités, construction scan après scan |
| 5 Carte vectorielle | les 4 étapes de l'extraction (clusters, split, ajustement, fusion), puis la carte |
| 6 Résultat | couches, carte de référence MapTool, export, image, projet |

**Recalage, du plus simple au plus fort :**
- **À la main**, à la souris ou au clavier, pour dégrossir un scan très mal placé.
- **🎯 Recaler la sélection en bloc :** les scans sélectionnés gardent leur placement relatif et sont recalés
  ensemble sur les autres. C'est utile pour un groupe de scans bien alignés entre eux mais décalés du reste.
- **🧩 ICP scan par scan :** chaque scan est recalé sur ceux déjà placés. C'est rapide, mais une erreur se propage.
- **🚀 Optimisation globale :** toutes les poses sont corrigées en même temps (ICP point à droite, moindres carrés).
- **⚡ Recalage complet** (recommandé) : l'optimisation globale est lancée depuis deux départs, les poses actuelles
  et les poses après l'ICP scan par scan, et on garde le meilleur. Sur la salle, l'écart médian entre scans passe
  de 6,4 cm à 2,2 cm.

Le scan **principal** (contour épais) sert d'ancre : il ne bouge pas pendant les recalages automatiques.

## Gestes communs aux deux applications (bouton ✋ actif)

| Geste | Effet |
|---|---|
| clic sur un scan | le sélectionner |
| Ctrl + clic | l'ajouter à la sélection, ou l'en retirer |
| glisser un scan | déplacer toute la sélection |
| clic droit glissé (ou Maj + glisser) | tourner la sélection autour de son centre |
| Ctrl + molette | tourner de ±1° (Maj : ±5°) |
| flèches, A / E | déplacer, tourner (pas fin, normal ou gros) |
| Ctrl + Z | annuler (jusqu'à 60 fois) |
| glisser dans le vide, molette | déplacer, zoomer la vue |

Pendant un geste, le dessin est seulement transformé, rien n'est recalculé. Le mouvement reste donc fluide, et les
cartes sont reconstruites au relâchement.

## Bibliothèque `src/lidar/`

| Module | Contenu |
|---|---|
| `geometry.py` | pose, polaire → cartésien, repère lidar → carte, `move_rigid` (rotation d'un groupe autour d'un pivot) |
| `scan.py` | un scan (angles, distances, intensités, pose), lecture et écriture de tous les formats du module |
| `gridmap.py` | grid map en log-odds, tracé des faisceaux, exports MapTool |
| `segments.py`, `extraction.py` | opérations sur segments, et nuage de points → segments |
| `vectormap.py` | carte vectorielle (association, fusion, détection d'absence, nettoyage) |
| `matching.py` | ICP point à point, `refine_global` (multi-scans, point à droite), `align_best`, `consistency` |
| `localization/` | Jour 2 : champ de vraisemblance, recherche exhaustive, multi-résolution, Gauss-Newton (grid map) ; point → segment par Gauss-Newton et calcul direct (carte vectorielle) ; ICP point à point et point à droite (entre scans). `METHODS` les rend interchangeables |
| `scope/collector.py` | récolte de N tours et combinaison (médiane / tous les points), bruit des distances |
| `simulate.py` | salle simulée pour vérifier les algorithmes |

**Performances** (16 scans réels, environ 7 000 points) : ICP de tous les scans 0,17 s, recalage complet 1,3 s,
grid map 0,06 s, carte vectorielle 0,08 s. Les plus proches voisins utilisent un arbre k-d (`scipy`).

Les notes d'algorithmes de la première version (Jour 1) sont dans [jour1.md](jour1.md).
