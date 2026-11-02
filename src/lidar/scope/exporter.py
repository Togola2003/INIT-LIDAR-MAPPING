"""Export « au choix » : pour l'ensemble des captures et/ou pour chaque capture séparément.

Arborescence produite :
    export_AAAAMMJJ_HHMMSS/
        LISEZMOI.md                 rapport : conventions, réglages, résultats, liste des fichiers
        ensemble/                   toutes les captures incluses, fusionnées en une carte
        scan_01_<nom>/  ...         une capture seule (son propre rendu et ses propres cartes)
Dans chaque dossier, selon les cases cochées :
    session.lidar.json          données complètes auto-décrites (+ lignes série brutes + cartes)
    points.csv                  tableau plat, une ligne par point
    carte.png                   rendu de la carte (fond blanc, prêt pour le compte rendu)
    grid_map_maptool.csv        grid map au format MapTool (1 = occupé, 0 = sinon)
    grid_map_ternaire.csv       1 = occupé, 0 = libre, -1 = inconnu
    grid_map_proba.csv          probabilité uint8 (0 libre … 127 inconnu … 255 occupé)
    carte_vectorielle.txt       segments x1,y1,x2,y2 (format MapTool / module)
    scans_tp/*.csv              un fichier par scan au format du TP (lidar.viewer, gridmap...)
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from ..scan import save_scan
from ..segments import Segment
from ..vectormap import Feature, VectorMap, save_vector_map
from .mapping import MapParams, MapResult, build_maps
from .render import render_png
from .session import SOURCE_NOTES, Capture, save_session, save_table

ITEMS = {
    "session": "Session complète auto-décrite (.lidar.json)",
    "table": "Tableau de tous les points (CSV)",
    "png": "Rendu de la carte (PNG)",
    "grid": "Grid map (CSV MapTool + ternaire + probabilités)",
    "vector": "Carte vectorielle (TXT)",
    "tp": "Fichiers scans au format du TP (CSV)",
}
SCOPES = {
    "chaque+ensemble": "Chaque scan séparément + l'ensemble",
    "ensemble": "L'ensemble seulement (une carte fusionnée)",
    "chaque": "Chaque scan séparément",
    "selection": "La capture sélectionnée seulement",
}


@dataclass(frozen=True)
class ExportOptions:
    scope: str = "chaque+ensemble"
    items: frozenset = frozenset(ITEMS)


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_").lower() or "scan"


def _export_target(folder: Path, captures: list[Capture], params: MapParams, items: frozenset,
                   title: str, reference: list[Segment]) -> tuple[list[Path], MapResult]:
    folder.mkdir(parents=True, exist_ok=True)
    result = build_maps([c.scan for c in captures], params)
    files: list[Path] = []
    if "session" in items:
        extra = {"map_params": params.as_dict(), "map_stats": result.stats,
                 "segments_xyxy_m": [list(s.as_tuple()) for s in result.segments]}
        if result.grid is not None:
            g = result.grid
            extra["grid"] = {"resolution_m": g.resolution, "origin_xy_m": list(g.origin),
                             "shape_rows_cols": list(g.shape), "files": "grid_map_*.csv dans le même dossier"}
        files.append(save_session(captures, folder / "session.lidar.json", extra))
    if "table" in items:
        files.append(save_table(captures, folder / "points.csv"))
    if "png" in items:
        files.append(render_png(folder / "carte.png", captures, result, params.grid.model, title, reference))
    if "grid" in items and result.grid is not None:
        from ..gridmap import save_grid_csv
        for enc in ("maptool", "ternaire", "proba"):
            files.append(save_grid_csv(result.grid, folder / f"grid_map_{enc}.csv", enc))
    if "vector" in items and result.segments:
        vmap = VectorMap(tuple(Feature(s) for s in result.segments), len(captures))
        files.append(save_vector_map(vmap, folder / "carte_vectorielle.txt"))
    if "tp" in items:
        files += [save_scan(c.scan, folder / "scans_tp" / f"scan_{c.number:02d}.csv") for c in captures]
    return files, result


def export_bundle(base: str | Path, captures: list[Capture], selected: int | None, options: ExportOptions,
                  params: MapParams, reference: list[Segment] | None = None) -> Path:
    """Crée le dossier d'export et renvoie son chemin."""
    if not captures:
        raise ValueError("aucune capture à exporter")
    root = Path(base) / f"export_{datetime.now():%Y%m%d_%H%M%S}"
    targets: list[tuple[str, list[Capture], str]] = []
    if options.scope in ("chaque", "chaque+ensemble"):
        targets += [(f"scan_{c.number:02d}_{_slug(c.title)}", [c], c.title) for c in captures]
    if options.scope in ("ensemble", "chaque+ensemble"):
        targets.append(("ensemble", captures, f"Carte de l'ensemble ({len(captures)} scans)"))
    if options.scope == "selection":
        sel = [c for c in captures if c.number == selected]
        if not sel:
            raise ValueError("aucune capture sélectionnée")
        targets.append((f"scan_{sel[0].number:02d}_{_slug(sel[0].title)}", sel, sel[0].title))
    report = []
    for name, caps, title in targets:
        files, result = _export_target(root / name, caps, params, options.items, title, reference or [])
        report.append((name, caps, result, files))
    (root / "LISEZMOI.md").write_text(_report(root, report, params), encoding="utf-8")
    return root


def _report(root: Path, report: list, params: MapParams) -> str:
    p = params.as_dict()
    lines = [
        "# Export LidarScope : cartographie lidar LD06",
        "",
        f"Créé le {datetime.now():%d/%m/%Y à %H:%M:%S}.",
        "",
        "## Conventions (valables pour tous les fichiers)",
        "- Distances en **mètres**, angles en **degrés**, sens **trigonométrique** (antihoraire), 0° = avant du robot.",
        "- Repère **lidar** : +X = avant du robot, +Y = sa gauche. Repère **carte** : origine au coin de référence de la salle.",
        "- Pose `(x, y, θ)` : `x_carte = x + cos θ·x_lidar − sin θ·y_lidar`, `y_carte = y + sin θ·x_lidar + cos θ·y_lidar`.",
        "- `distance = 0` = pas d'écho. Intensité vide / `null` = non transmise (format Teleplot).",
        "- `session.lidar.json` contient la description complète des colonnes et les lignes série **brutes**.",
        "",
        "## Réglages de cartographie (étapes du cours)",
        f"- Prétraitement : portée {p['filters']['min_range_m']:.2f} à {p['filters']['max_range_m']:.1f} m, "
        f"intensité ≥ {p['filters']['min_intensity']}",
        f"- Grid map : résolution {p['grid']['resolution_m'] * 100:.0f} cm, état initial « {p['grid']['initial_state']} », "
        f"modèle {p['grid']['model']}, p(occupé | impact) = {p['grid']['p_occ']:.2f}, "
        f"p(occupé | traversée) = {p['grid']['p_free']:.2f}, suppression des cellules isolées : "
        f"{'oui' if p['grid']['remove_isolated'] else 'non'}",
        f"- Carte vectorielle : saut {p['vector']['jump_m'] * 100:.0f} cm, tolérance de découpe "
        f"{p['vector']['split_tol_m'] * 100:.0f} cm, longueur min {p['vector']['min_length_m'] * 100:.0f} cm, "
        f"fusion si angle ≤ {p['vector']['merge_angle_deg']:.0f}°",
        "",
        "## Contenu",
    ]
    for name, caps, result, files in report:
        lines += ["", f"### `{name}/`", ""]
        for c in caps:
            pose = c.scan.pose
            lines.append(f"- **{c.title}** : pose ({pose.x:.2f} m, {pose.y:.2f} m, {pose.theta_deg:.1f}°), "
                         f"{int((c.scan.ranges_m > 0).sum())}/{len(c.scan)} points avec écho, "
                         f"source `{c.source}` : {SOURCE_NOTES.get(c.source, '')}")
        st = result.stats
        if "grid" in st:
            g = st["grid"]
            lines.append(f"- Grid map {g['cells']} cellules : {g['occupied']} occupées, {g['free']} libres, "
                         f"{g['unknown']} inconnues (surface explorée {g['explored_area_m2']} m²)")
        if "vector" in st:
            v = st["vector"]
            lines.append(f"- Carte vectorielle : {v['segments']} segments, longueur totale {v['total_length_m']} m")
        lines.append("- Fichiers : " + ", ".join(f"`{f.relative_to(root / name).as_posix()}`" for f in files))
    return "\n".join(lines) + "\n"
