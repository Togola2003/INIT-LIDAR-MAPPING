"""Génère les figures et les résultats chiffrés du compte rendu du Jour 1.

    .venv/Scripts/python TP/Jour1/generer_figures.py

Entrée  : data/salle/session_salle_2026-11-02.lidar.json (16 scans réels, poses mesurées à la main)
Sorties : TP/Jour1/figures/*.png, TP/Jour1/resultats/* (grid map, carte vectorielle, métriques)
          data/salle/session_salle_recalee.lidar.json (mêmes scans, poses corrigées par ICP)
"""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lidar.extraction import ExtractionParams, extraction_stages  # noqa: E402
from lidar.geometry import Pose, local_to_global  # noqa: E402
from lidar.gridmap import save_grid_csv  # noqa: E402
from lidar.matching import align_sequence, nearest, voxel_downsample  # noqa: E402
from lidar.scope.mapping import GridParams, MapParams, VectorParams, beam_image, build_maps  # noqa: E402
from lidar.scope.processing import Filters  # noqa: E402
from lidar.scope.session import load_session, save_session  # noqa: E402
from lidar.segments import Segment, point_segment_distance  # noqa: E402
from lidar.simulate import ROOM_WALLS, simulate_scan  # noqa: E402
from lidar.vectormap import Feature, VectorMap, save_vector_map  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SESSION = ROOT / "data/salle/session_salle_2026-11-02.lidar.json"
FIG = ROOT / "TP/Jour1/figures"
RES = ROOT / "TP/Jour1/resultats"
FILTERS = Filters(min_range_m=0.15, max_range_m=10.0, min_neighbors=2, neighbor_radius_m=0.10)
PARAMS = MapParams(filters=FILTERS, grid=GridParams(resolution_m=0.05),
                   vector=VectorParams(jump_m=0.08, split_tol_m=0.04, min_length_m=0.30))
ANCHOR = "Scan 16"   # pose (0, 0, 0) : origine choisie pour la salle


def new_ax(ax, title):
    ax.set_title(title, fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, color="#d0d7de", lw=0.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def scatter_caps(ax, caps, filters=FILTERS, size=2):
    for c in caps:
        pts = filters.apply(c.scan).global_points()
        ax.scatter(pts[:, 0], pts[:, 1], s=size, color=c.color, linewidths=0)
        p = c.scan.pose
        ax.plot(p.x, p.y, "^", color=c.color, mec="k", ms=6)


def draw_grid(ax, res, model="probabiliste"):
    ax.imshow(res.grid_image(model), cmap="gray", vmin=0, vmax=1, origin="lower", extent=res.grid.extent(),
              interpolation="nearest")


def draw_segments(ax, segs, color="#cf222e", lw=1.8):
    for s in segs:
        ax.plot([s.x1, s.x2], [s.y1, s.y2], "-", color=color, lw=lw)


def consistency(caps) -> np.ndarray:
    """Pour chaque scan : médiane de la distance de ses points au scan voisin le plus proche (m)."""
    pts = [FILTERS.apply(c.scan).global_points() for c in caps]
    out = []
    for k in range(len(pts)):
        others = voxel_downsample(np.vstack([p for j, p in enumerate(pts) if j != k]), 0.02)
        out.append(float(np.median(nearest(pts[k], others)[1])))
    return np.array(out)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(FIG / name, dpi=130)
    plt.close(fig)
    print("  figure", name)


# ====================================================================== scans réels
def real_data(metrics: dict) -> None:
    caps = load_session(SESSION)
    anchor = [c.title for c in caps].index(ANCHOR)
    print(f"{len(caps)} scans réels, ancre {ANCHOR}")

    # Fig 1 : nuages de points bruts, poses mesurées à la main
    fig, ax = plt.subplots(figsize=(8, 8))
    scatter_caps(ax, caps, Filters(0.01, 12.0))
    new_ax(ax, "Fig. 1 - 16 scans réels, poses mesurées à la main (une couleur par scan)")
    save(fig, "fig1_nuages_poses_manuelles.png")

    # Fig 2 : prétraitement
    fig, ax = plt.subplots(figsize=(8, 8))
    kept = rej_range = rej_nb = 0
    for c in caps:
        raw = c.scan.filtered(0.0001, 1e9)
        out = (raw.ranges_m < FILTERS.min_range_m) | (raw.ranges_m > FILTERS.max_range_m)
        g = raw.global_points()
        ax.scatter(g[out, 0], g[out, 1], s=10, marker="x", color="#cf222e", lw=0.8)
        iso = FILTERS.rejected(c.scan).global_points()
        ax.scatter(iso[:, 0], iso[:, 1], s=10, marker="x", color="#fb8f44", lw=0.8)
        k = FILTERS.apply(c.scan).global_points()
        ax.scatter(k[:, 0], k[:, 1], s=2, color="#24292f", linewidths=0)
        kept, rej_range, rej_nb = kept + len(k), rej_range + int(out.sum()), rej_nb + len(iso)
    new_ax(ax, "Fig. 2 - Prétraitement : gardés (noir), hors portée (rouge), isolés (orange)")
    save(fig, "fig2_pretraitement.png")
    metrics["pretraitement"] = {"points_avec_echo": kept + rej_range + rej_nb, "gardes": kept,
                                "hors_portee": rej_range, "isoles": rej_nb}

    # Recalage ICP
    before = consistency(caps)
    t = time.time()
    out = align_sequence([FILTERS.apply(c.scan).local_points() for c in caps], [c.scan.pose for c in caps], anchor)
    dt = time.time() - t
    aligned = [replace(c, scan=c.scan.with_pose(p)) for c, (p, _) in zip(caps, out)]
    after = consistency(aligned)
    rows = []
    for c, (p, r), b, a in zip(caps, out, before, after):
        q = c.scan.pose
        rows.append({"scan": c.title, "ecart_avant_cm": round(b * 100, 1), "ecart_apres_cm": round(a * 100, 1),
                     "correction": [round(p.x - q.x, 3), round(p.y - q.y, 3), round(p.theta_deg - q.theta_deg, 2)],
                     "icp_rms_cm": None if r is None else round(r.rms_m * 100, 1),
                     "recouvrement": None if r is None else round(r.inlier_ratio, 2)})
    metrics["recalage"] = {"ecart_median_avant_cm": round(float(np.median(before)) * 100, 1),
                           "ecart_median_apres_cm": round(float(np.median(after)) * 100, 1),
                           "temps_s": round(dt, 1), "par_scan": rows}
    save_session(aligned, ROOT / "data/salle/session_salle_recalee.lidar.json",
                 {"recalage": "ICP séquentiel, ancre " + ANCHOR, "filters": FILTERS.as_dict()})

    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    for ax, cs, title in ((axes[0], caps, f"avant (écart médian {np.median(before) * 100:.1f} cm)"),
                          (axes[1], aligned, f"après ICP (écart médian {np.median(after) * 100:.1f} cm)")):
        scatter_caps(ax, cs)
        new_ax(ax, "Fig. 3 - Recalage " + title)
    save(fig, "fig3_recalage_avant_apres.png")

    # Fig 4 : grid map d'UN scan + cellules traversées / d'impact
    one = aligned[anchor]
    res1 = build_maps([one.scan], replace(PARAMS, vector=replace(PARAMS.vector, enabled=False)))
    rgba, ext = beam_image(one.scan, PARAMS)
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    axes[0].imshow(rgba, origin="lower", extent=ext, interpolation="nearest")
    new_ax(axes[0], f"Fig. 4a - {one.title} : cellules traversées (bleu) et d'impact (rouge)")
    draw_grid(axes[1], res1)
    new_ax(axes[1], "Fig. 4b - Grid map obtenue avec ce seul scan (blanc libre, noir occupé, gris inconnu)")
    save(fig, "fig4_gridmap_un_scan.png")

    # Fig 5 : mise à jour progressive de la grid map
    order = sorted(range(len(aligned)), key=lambda k: np.hypot(aligned[k].scan.pose.x, aligned[k].scan.pose.y))
    fig, axes = plt.subplots(1, 4, figsize=(20, 6))
    grid_params = replace(PARAMS, vector=replace(PARAMS.vector, enabled=False))
    for ax, n in zip(axes, (1, 4, 8, len(order))):
        r = build_maps([aligned[k].scan for k in order[:n]], grid_params)
        draw_grid(ax, r)
        new_ax(ax, f"Fig. 5 - après {n} scan(s)")
    save(fig, "fig5_gridmap_mise_a_jour.png")

    # Fig 6 : étapes de l'extraction de segments sur un scan
    s = FILTERS.apply(one.scan).sorted_by_angle()
    st = extraction_stages(s.global_points(), s.ranges_m,
                           ExtractionParams(jump_base_m=0.08, split_tol_m=0.04, min_length_m=0.30))
    pal = plt.get_cmap("tab20")
    fig, axes = plt.subplots(1, 4, figsize=(22, 6))
    for i, c in enumerate(st.clusters):
        axes[0].scatter(c[:, 0], c[:, 1], s=4, color=pal(i % 20))
    for i, c in enumerate(st.pieces):
        axes[1].scatter(c[:, 0], c[:, 1], s=4, color=pal(i % 20))
    for c in st.rejected:
        axes[1].scatter(c[:, 0], c[:, 1], s=6, color="#8c959f", marker="x", lw=0.6)
    for ax in axes[2:]:
        for c in st.pieces:
            ax.scatter(c[:, 0], c[:, 1], s=2, color="#8c959f")
    for i, sg in enumerate(st.fitted):
        axes[2].plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", color=pal(i % 20), lw=2)
    draw_segments(axes[3], st.merged, lw=2.2)
    for ax, title in zip(axes, (f"① segmentation : {len(st.clusters)} clusters",
                                f"② split : {len(st.pieces)} morceaux (+{len(st.rejected)} écartés)",
                                f"③ ajustement : {len(st.fitted)} segments",
                                f"④ fusion : {len(st.merged)} segments")):
        new_ax(ax, "Fig. 6 - " + title)
    save(fig, "fig6_segments_etapes.png")
    metrics["segments_un_scan"] = {"scan": one.title, "clusters": len(st.clusters), "morceaux": len(st.pieces),
                                   "ecartes": len(st.rejected), "ajustes": len(st.fitted), "fusionnes": len(st.merged)}

    # Fig 7 et 8 : cartes finales
    full = build_maps([c.scan for c in aligned], PARAMS)
    fig, ax = plt.subplots(figsize=(8, 9))
    scatter_caps(ax, aligned, size=1)
    draw_segments(ax, full.segments, lw=2)
    new_ax(ax, f"Fig. 7 - Carte vectorielle de la salle : {len(full.segments)} segments")
    save(fig, "fig7_carte_vectorielle.png")
    fig, ax = plt.subplots(figsize=(8, 9))
    draw_grid(ax, full)
    draw_segments(ax, full.segments, lw=1.5)
    for c in aligned:
        ax.plot(c.scan.pose.x, c.scan.pose.y, "^", color=c.color, mec="k", ms=6)
    new_ax(ax, "Fig. 8 - Carte finale : grid map 5 cm + segments + poses")
    save(fig, "fig8_carte_finale.png")
    metrics["carte_finale"] = full.stats

    RES.mkdir(parents=True, exist_ok=True)
    for enc in ("maptool", "ternaire", "proba"):
        save_grid_csv(full.grid, RES / f"grid_map_{enc}.csv", enc)
    save_vector_map(VectorMap(tuple(Feature(s) for s in full.segments), len(aligned)), RES / "carte_vectorielle.txt")


# ====================================================================== validation en simulation
def simulation(metrics: dict) -> None:
    """Vérité terrain connue : on mesure l'erreur de l'ICP et la précision des segments."""
    truths = [Pose(0.8, 0.8, 0), Pose(2.8, 1.2, 90), Pose(1.2, 2.3, -45), Pose(3.2, 2.5, 180)]
    errors = [Pose(), Pose(0.15, -0.10, 6), Pose(-0.20, 0.15, -8), Pose(0.10, 0.20, 5)]
    scans = [simulate_scan(t, noise_m=0.01, seed=k) for k, t in enumerate(truths)]
    starts = [t.moved(e.x, e.y, e.theta_deg) for t, e in zip(truths, errors)]
    out = align_sequence([s.filtered().local_points() for s in scans], starts, anchor=0)
    pose_err = [{"pos_cm": round(float(np.hypot(p.x - t.x, p.y - t.y)) * 100, 2),
                 "angle_deg": round(abs(p.theta_deg - t.theta_deg), 2)} for (p, _), t in zip(out, truths)]
    aligned = [s.with_pose(p) for s, (p, _) in zip(scans, out)]
    res = build_maps(aligned, MapParams(vector=VectorParams(min_length_m=0.2)))
    walls = [Segment(*w) for w in ROOM_WALLS]
    dists = [min(point_segment_distance(p, w) for w in walls) for s in res.segments for p in (s.p1, s.p2)]
    metrics["simulation"] = {"erreur_pose_apres_icp": pose_err,
                             "segments": len(res.segments),
                             "ecart_extremites_vs_vrais_murs_cm": {
                                 "median": round(float(np.median(dists)) * 100, 2),
                                 "max": round(float(np.max(dists)) * 100, 2)}}
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax, poses, title in ((axes[0], starts, "poses perturbées"), (axes[1], [p for p, _ in out], "après ICP")):
        for k, (s, p) in enumerate(zip(scans, poses)):
            g = s.filtered().global_points(p)
            ax.scatter(g[:, 0], g[:, 1], s=2, color=plt.get_cmap("tab10")(k))
        for w in walls:
            ax.plot([w.x1, w.x2], [w.y1, w.y2], "--", color="#8250df", lw=1)
        new_ax(ax, f"Fig. 9 - Simulation (murs vrais en violet) : {title}")
    draw_segments(axes[1], res.segments, lw=2)
    save(fig, "fig9_validation_simulation.png")


def main() -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    RES.mkdir(parents=True, exist_ok=True)
    metrics: dict = {"parametres": PARAMS.as_dict()}
    real_data(metrics)
    simulation(metrics)
    (RES / "metriques.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in metrics.items() if k != "parametres"}, ensure_ascii=False, indent=1)[:3500])


if __name__ == "__main__":
    main()
