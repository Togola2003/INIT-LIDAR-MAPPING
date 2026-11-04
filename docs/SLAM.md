# SLAM pas à pas (Jour 3)

**SLAM** = *Simultaneous Localization And Mapping* : le robot construit la carte **et** se repère dedans, en même
temps. Personne ne lui donne de carte au départ.

## La consigne du prof

1. Poser le robot n'importe où (un coin, par exemple) et scanner. Ce 1er scan est la **première référence**.
2. Déplacer **un peu** le robot (à la main ou en Wi-Fi) et rescanner.
3. Le 2e scan a forcément des **points en commun** avec la référence : grâce à eux, le robot calcule où il est.
4. Ses autres points sont la **partie nouvelle** : on l'ajoute. Référence + 2e scan = **nouvelle référence**.
5. Recommencer par **petits déplacements**, jusqu'à avoir toute la salle (le tour, sous les tables, le milieu).

## Dans LidarLab : étape 1 « Récolter »

| Geste | Ce qui se passe |
|---|---|
| **Scanner**, puis **Garder** | 1er scan = référence, origine (0, 0, 0) de la carte |
| Déplacer le robot (pavé ou main) | trait bleu : le robot est suivi en direct sur la référence |
| **Scanner** | le scan est localisé ; aperçu : **orange = commun**, **vert = nouveau** |
| **Garder** | le scan rejoint la référence, avec sa couleur |
| « ↻ Boucle refermée » | le robot revoit un scan ancien : **Optimiser toute la carte** |

Les chiffres à lire dans l'aperçu :
- **Points communs ≥ 35 % :** pose sûre ✓.
- **Points communs < 35 % :** ⚠ rapproche le robot d'une zone déjà scannée et refais le scan.
- **Partie nouvelle :** ce que la carte gagne avec ce scan.

## Comment le robot sait où il est (le calcul)

Code : [`src/lidar/slam.py`](../src/lidar/slam.py).

1. **Pose prévue :**
   - pendant le suivi en direct : la dernière pose suivie ;
   - si le robot a été piloté : la pose déduite des commandes envoyées (`robot.command_motion`) ;
   - sinon : la pose du scan précédent.
2. **Pose grossière :** on cherche (x, y, θ) sur la **grid map** de la référence, à ± 1 m et ± 40° autour de la
   pose prévue (sinon dans toute la carte). La recherche se fait du grossier au fin (cases de 20, 10 puis 5 cm),
   puis un gradient affine le résultat.
3. **Pose fine :** un **ICP** sur les points de tous les scans gardés précise la pose au centimètre, mieux qu'une case
   de 5 cm. Il n'est gardé que s'il améliore l'accord avec la carte.
4. **Commun / nouveau :** un point à moins de 8 cm d'un point de la carte est commun ; les autres sont nouveaux.
5. **Boucle :** si le scan partage au moins 25 % de ses points avec un scan **ancien**, alors que le robot a parcouru
   3 m depuis, la boucle se referme.
   - **Optimiser :** corrige **toutes** les poses ensemble (`matching.refine_global`). Chaque point d'un scan est
     tiré vers le mur du scan voisin, et tout est résolu en une seule fois (Gauss-Newton).
   - **Résultat :** l'erreur accumulée de proche en proche est répartie sur tout le trajet.

**Pourquoi la carte ne rétrécit ni ne grandit :** le lidar mesure en mètres. Poser un scan, c'est le tourner et le
déplacer, jamais l'étirer. Le seul risque est un scan **mal posé**. Les points communs affichés et l'optimisation
finale sont là pour ça.

## Grid map ou carte vectorielle ?

| | Grid map | Carte vectorielle (segments) |
|---|---|---|
| Ajouter un scan | facile : on remplit des cases | difficile : il faut extraire puis fusionner les murs |
| Chaises, pieds de table | gérés | mal gérés (ce ne sont pas des murs) |
| Précision | une case (5 cm), plus l'ICP fin | très précise sur les murs droits |
| Rôle dans LidarLab | **le SLAM se fait dessus** | carte finale (plan, export), localisation précise |

Le cours le dit aussi : grille d'occupation → « SLAM 2D, navigation au sol » ; carte vectorielle → « SLAM basé sur
repères ».

## La grid map : une seule fois à la fin, ou scan par scan ? (question du prof)

**Réponse : scan par scan, comme dans le cours** (« Construction d'une grid map », §5 : chaque nouveau scan est
aligné par scan matching, puis on répète les étapes 2 à 4). Code : `gridmap.update_grid`.

**Ce que fait chaque scan, faisceau par faisceau :**
1. **Cases traversées** par le faisceau (tracé de Bresenham) : la probabilité d'occupation **baisse**
   (log-odds + L_traversée, réglage « p(occupé | traversée) » = 0,40, soit −0,41).
2. **Case d'impact :** la probabilité **monte** (log-odds + L_impact, réglage « p(occupé | impact) » = 0,70,
   soit +0,85).
3. Chaque case garde la **somme** de toutes les observations de tous les scans, bornée à ± 4.

**Ce que ça donne quand les scans se recouvrent :**
- **un mur vu par plusieurs scans** devient de plus en plus sûr (plus noir) ;
- **un objet déplacé** (chaise, personne) : les faisceaux des scans suivants **traversent** sa place, donc il
  s'efface ;
- **une case jamais vue** reste « inconnue » (gris).

**Exemple :** une case touchée par 2 scans et traversée par 3 autres.
- **Réglages par défaut :** 2 × 0,85 − 3 × 0,41 = +0,48, p = 0,62 → **occupée** (le pied de table reste).
- **Traversée réglée à 0,30 (−0,85) :** 1,70 − 2,54 = −0,84, p = 0,30 → **libre** (le pied de table disparaît).

C'est ce que montrent les deux curseurs de l'étape **Cartes** : ils changent le poids d'un impact et d'une
traversée, donc certains murs, tables ou chaises apparaissent ou disparaissent.

**« Une fois à la fin » ou « après chaque scan », c'est la même carte :** une addition ne dépend pas de l'ordre. La
mise à jour est faite après chaque scan, et rejouer tous les scans à la fin donne exactement la même grid map
(vérifié dans `tests/test_gridmap.py`).

**Dans LidarLab :**
- **Pendant le SLAM,** à chaque « Garder », la grid map de la référence est recalculée en rejouant les scans un
  par un (`slam.SlamMap`). Elle sert à localiser le scan suivant.
  - **Pourquoi la recalculer au lieu d'ajouter seulement le dernier scan :**
    - la grille doit **s'agrandir** quand le robot découvre une nouvelle zone ;
    - après **« Optimiser toute la carte »**, les poses changent, et les faisceaux doivent être retracés depuis
      les poses corrigées.
  - Google Cartographer fait pareil : ses sous-cartes sont redessinées après l'optimisation.
- **À l'étape Cartes,** la grid map finale est construite de la même façon, avec les curseurs impact / traversée
  et l'état initial (inconnu, libre, occupé).

## Les grandes familles de SLAM 2D (pour situer notre méthode)

| Méthode | Principe | Notre code |
|---|---|---|
| **Scan-to-map** (Hector SLAM) | chaque scan est aligné sur la grid map construite jusque-là | `place_scan` + `SlamMap` |
| **Recherche corrélative** (Olson 2009, Cartographer) | on teste beaucoup de poses, du grossier au fin, et on garde la meilleure | `localization/grid.py` |
| **ICP** (Besl et McKay 1992) | on apparie chaque point au plus proche, et on corrige la pose jusqu'à convergence | `matching.icp` |
| **Graphe de poses** (Lu et Milios 1997, Karto, Cartographer) | toutes les poses sont corrigées ensemble grâce aux recouvrements | `matching.refine_global` |
| **Filtre particulaire** (GMapping) | beaucoup d'hypothèses de trajet en parallèle | non utilisé : trop lourd, inutile à l'arrêt |

**Notre choix : « stop and go ».**
- **Pourquoi :** le robot n'a pas de codeurs, et un tour pris en roulant est déformé.
- **Ce qu'on fait :** à l'arrêt, on combine 10 tours, ce qui donne un scan net, donc une pose sûre.
- **Le suivi en direct** sert entre deux scans : il donne la pose prévue, et le trajet affiché.

## Résultats en simulation

Salle de 6 × 4 m avec deux obstacles, 18 scans, tour complet avec retour au départ. Tests :
`tests/test_slam.py`, et un essai dans la vraie fenêtre.

| Mesure | Résultat |
|---|---|
| Erreur de pose de chaque scan | ≤ 1 cm, ≤ 0,1° |
| Échelle, après 13 m de trajet | aucune dérive : chaque pose reste à ≤ 1 cm |
| Localisation d'un scan | ≈ 15 ms |
| « Garder » (référence reconstruite) | ≈ 0,35 s |
| Optimisation de 18 scans | ≈ 1,6 s |
| Scan pris ailleurs (pièce ronde) | refusé ⚠ (pas assez de points communs) |

## Conseils en salle

1. Départ dans un **coin** : beaucoup de murs, donc une pose bien définie.
2. Petits pas : **≤ 1 m et ≤ 45°**. Chaque scan doit voir une bonne partie de ce qu'ont vu les autres.
3. Faire le **tour** de la salle, passer **sous les tables**, puis le **milieu**.
4. **Revenir près du départ** à la fin, puis **Optimiser toute la carte**.
5. Ensuite : Nettoyer → Cartes → **Valider** (la carte devient la référence pour l'étape Localiser).

## Références

- S. Kohlbrecher et al., *A Flexible and Scalable SLAM System with Full 3D Motion Estimation* (Hector SLAM),
  2011.
- E. Olson, *Real-Time Correlative Scan Matching*, ICRA 2009.
- W. Hess et al., *Real-Time Loop Closure in 2D LIDAR SLAM* (Google Cartographer), ICRA 2016.
- F. Lu, E. Milios, *Globally Consistent Range Scan Alignment for Environment Mapping*, 1997.
- P. Besl, N. McKay, *A Method for Registration of 3-D Shapes* (ICP), 1992.
- G. Grisetti et al., *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters* (GMapping),
  2007.
