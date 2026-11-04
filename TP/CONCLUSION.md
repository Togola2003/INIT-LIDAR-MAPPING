# Conclusion personnelle sur le module

## Ce que j'ai apprécié

- **Le sujet.** Je voyais souvent passer ce genre de robots et de cartes lidar sur les réseaux sociaux (TikTok,
  LinkedIn, des pages de robotique). Je connaissais, mais je ne comprenais pas comment ça fonctionnait. Ce module
  m'a permis de le comprendre vraiment.
- **Le plaisir de le faire.** Je n'ai pas vu le temps passer. Même en dehors des séances, je pensais tout le temps à
  ce que j'allais faire en classe ensuite.
- **Le traitement des points.** J'ai compris comment on passe des points bruts du lidar à une carte, puis à la
  position du robot. J'ai beaucoup aimé le concept et l'ingéniosité qu'il y a derrière.
- **L'acquisition en direct.** Voir arriver les données du lidar, les décoder et les afficher en temps réel, c'était
  très important pour comprendre, et très passionnant.

## Mes choix

À chaque fois, j'ai commencé par **sortir les points** du lidar : scanner, récupérer les données, les vérifier.
Ensuite, à partir de ces points, j'ai développé une **application en Python** (LidarLab) qui les traite avec les
différentes méthodes vues en cours : grid map, carte vectorielle, scan matching, localisation, puis SLAM.

## Pour chaque outil

| Outil | Ce que j'en retiens |
|---|---|
| Acquisition en direct (LidarScope, puis LidarLab) | voir les trames arriver et devenir un tour de 360° : c'est là que j'ai compris le capteur |
| Grid map | chaque faisceau rend libres les cases traversées et occupée la case d'impact ; les scans s'additionnent |
| Carte vectorielle | des points aux segments, puis aux murs : plus précise sur les murs droits, moins souple que la grille |
| Localisation (Jour 2) | une recherche locale est précise mais aveugle : il faut chercher du grossier au fin |
| SLAM (Jour 3) | petits pas et points communs : la carte de toute la salle, construite scan après scan sur le vrai robot |

## Ce que j'ai moins apprécié

**Le manque de robots.** Il y avait trop peu de matériel pour tout le monde, et on passait beaucoup de temps sans
robot pour tester.

## Ce que j'améliorerais

- **Plus de robots,** pour que chacun puisse tester plus souvent.
- **Plus de temps** pour ceux que ce sujet passionne. Je sais que le temps est limité, mais certaines personnes ont
  vraiment envie d'y passer beaucoup de temps.
- **Aller vers la 3D.** Après avoir découvert la 2D, j'aurais vraiment aimé passer au lidar 3D.

## La suite, de mon côté

Une chose qu'on n'a pas faite, et que je vais développer moi-même : **le guidage**. À partir des cartes générées,
choisir un point sur la carte et laisser le robot calculer son chemin et s'y rendre tout seul.
