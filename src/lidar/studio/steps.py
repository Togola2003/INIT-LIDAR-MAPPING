"""Textes pédagogiques de chaque étape de LidarStudio (résumés des documents du cours)."""

STEPS = [
    ("1", "Données", "Charger les scans",
     "Un scan = des points (r, θ) dans le repère du lidar + la POSE du robot (x, y, θ) au moment de la "
     "mesure. Charge une session LidarScope (.lidar.json) ou des CSV (TP, MapTool, RobotControl). "
     "Coche les scans à utiliser pour la carte."),
    ("2", "Prétraitement", "Filtrer les points aberrants",
     "Cours : « filtrer et prétraiter les données pour supprimer les points aberrants (trop lointains, "
     "intensité trop faible, incohérents) ». Les points écartés sont affichés en rouge. Le filtre de "
     "VOISINAGE retire les points isolés (reflets, passants) et fonctionne même sans l'intensité "
     "(données Teleplot)."),
    ("3", "Recalage", "Corriger les poses",
     "P_global = R(θ)·P_local + T(x, y) : une erreur de quelques degrés sur la pose décale un mur de "
     "plusieurs dizaines de cm. Corrige à la main (flèches) ou automatiquement par ICP : le scan glisse "
     "jusqu'à ce que ses points tombent sur ceux des autres scans."),
    ("4", "Grid map", "Carte d'occupation",
     "① grille (résolution, état initial) ② points -> cellules ③ cellules TRAVERSÉES par le faisceau : "
     "plus libres (Bresenham) ④ cellule d'IMPACT : plus occupée ⑤ mise à jour scan après scan et "
     "nettoyage. Le curseur « pas à pas » montre la carte après 1, 2, … N scans."),
    ("5", "Carte vectorielle", "Segments",
     "① segmentation : un saut de distance entre deux points voisins sépare deux objets ② split : on coupe "
     "au point le plus loin de la corde (Douglas-Peucker) ③ ajustement d'une droite par morceau "
     "(régression orthogonale) ④ fusion des segments proches ⑤ carte : association entre scans, fusion, "
     "détection d'absence."),
    ("6", "Résultat", "Carte finale & export",
     "Superpose grid map, segments, points et poses. Compare avec une carte de référence MapTool (.txt). "
     "Exporte chaque scan et/ou l'ensemble (JSON auto-décrit, CSV, PNG, grid map, carte vectorielle) et "
     "enregistre le projet avec les poses corrigées."),
]

VECTOR_STAGES = ["Clusters", "Split", "Ajustement", "Fusion", "Carte"]
