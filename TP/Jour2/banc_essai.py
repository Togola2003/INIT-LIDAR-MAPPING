"""Banc d'essai du Jour 2 : compare toutes les méthodes de localisation, sur les mêmes scans et les
mêmes poses initiales, en simulation (vérité exacte) et sur les scans réels du 03/11.

    .venv/Scripts/python TP/Jour2/banc_essai.py

Protocole (identique pour toutes les méthodes) :
- jeu de scans : data/salle_jour2 (13 scans, poses recalées = « vérité » réelle, à quelques cm près) ;
  en simulation, les mêmes 13 poses, scans simulés dans la carte vectorielle de la salle (vérité exacte) ;
- laisser-un-dehors : pour localiser le scan k, les références sont construites SANS lui
  (grid map et carte vectorielle des 12 autres ; scan de référence = l'autre scan le plus proche) ;
- poses initiales : la vraie pose faussée de d mètres dans une direction tirée au hasard et de ± a degrés,
  pour 4 niveaux (d, a) = (0,10 m, 5°) (0,25 m, 10°) (0,50 m, 20°) (1,00 m, 30°), 6 tirages par niveau,
  graine fixe : toutes les méthodes reçoivent exactement les mêmes poses initiales ;
- réussite : erreur de position < 5 cm ET erreur d'angle < 2° (≈ 1 case de grid map, bruit du LD06 1-2 cm) ;
- zone couverte (condition de l'énoncé) : à sa vraie pose, au moins 35 % des points du scan sont à moins de
  10 cm d'un mur de la carte des 12 autres. Les résultats sont donnés pour les scans couverts ET pour tous.
Sorties : TP/Jour2/resultats/ (essais.csv, synthese.json, tableau.md) et TP/Jour2/figures/.
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from lidar.geometry import Pose  # noqa: E402
from lidar.localization import METHODS, References, localize, pose_error  # noqa: E402
from lidar.matching import consistency  # noqa: E402
from lidar.scope.processing import Filters  # noqa: E402
from lidar.scope.session import load_session  # noqa: E402
from lidar.simulate import simulate_scan  # noqa: E402
from lidar.vectormap import build_vector_map  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SESSION = ROOT / "data/salle_jour2/session_salle_2026-11-03.lidar.json"
OUT = ROOT / "TP/Jour2/resultats"
FIG = ROOT / "TP/Jour2/figures"
FILTERS = Filters(min_range_m=0.15, max_range_m=12.0, min_neighbors=2, neighbor_radius_m=0.10)
LEVELS = [(0.10, 5.0), (0.25, 10.0), (0.50, 20.0), (1.00, 30.0)]
DRAWS = 6
OK_POS_M, OK_ANG_DEG = 0.05, 2.0
SIM_NOISE_M = 0.01
COVER_DIST_M, COVER_MIN = 0.10, 0.35


def initial_poses(truth: Pose, seed: int) -> list[tuple[int, Pose]]:
    rng = np.random.default_rng(seed)
    out = []
    for lvl, (d, a) in enumerate(LEVELS):
        for _ in range(DRAWS):
            phi = rng.uniform(0, 2 * np.pi)
            out.append((lvl, truth.moved(d * np.cos(phi), d * np.sin(phi), a * rng.choice([-1.0, 1.0]))))
    return out


def load_real():
    caps = load_session(SESSION)
    scans = [FILTERS.apply(c.scan) for c in caps]
    return [c.label for c in caps], scans


def simulated(scans, walls):
    return [simulate_scan(s.pose, walls=walls, noise_m=SIM_NOISE_M, seed=k).filtered(0.15, 12.0)
            for k, s in enumerate(scans)]


def run_dataset(name: str, scans, rows: list) -> None:
    poses = [s.pose for s in scans]
    for k, scan in enumerate(scans):
        others = [s for j, s in enumerate(scans) if j != k]
        near = min((j for j in range(len(scans)) if j != k),
                   key=lambda j: np.hypot(poses[j].x - poses[k].x, poses[j].y - poses[k].y))
        refs = References.build(others, reference_points=scans[near].global_points())
        pts = scan.local_points()
        cover = float(np.mean(refs.field.residual(scan.global_points())[0] < COVER_DIST_M))
        for lvl, init in initial_poses(scan.pose, seed=1000 * k + 7):
            for method in METHODS:
                res = localize(method, pts, init, refs)
                pos, ang = pose_error(res.pose, scan.pose)
                rows.append({"jeu": name, "scan": k + 1, "couverture": round(cover, 2), "niveau": lvl, "d0_m": LEVELS[lvl][0], "a0_deg": LEVELS[lvl][1],
                             "methode": method, "erreur_m": round(pos, 4), "erreur_deg": round(ang, 3),
                             "temps_ms": round(res.time_s * 1000, 2), "score": round(res.score, 3),
                             "reussite": int(pos < OK_POS_M and ang < OK_ANG_DEG)})
        print(f"  {name} scan {k + 1:2d}/{len(scans)} : couverture {cover:.0%}, scan de référence voisin {near + 1}")


def summarize(rows: list) -> dict:
    out: dict = {}
    groups = {"simulation": lambda r: r["jeu"] == "simulation",
              "réel, zone couverte": lambda r: r["jeu"] == "réel" and r["couverture"] >= COVER_MIN,
              "réel, tous les scans": lambda r: r["jeu"] == "réel"}
    for jeu, keep in groups.items():
        out[jeu] = {}
        for m in METHODS:
            sel = [r for r in rows if keep(r) and r["methode"] == m]
            ok = [r for r in sel if r["reussite"]]
            out[jeu][m] = {
                "reussite_par_niveau": [round(np.mean([r["reussite"] for r in sel if r["niveau"] == lv]), 3)
                                        for lv in range(len(LEVELS))],
                "reussite_totale": round(float(np.mean([r["reussite"] for r in sel])), 3),
                "erreur_mediane_cm_si_reussi": round(float(np.median([r["erreur_m"] for r in ok])) * 100, 2) if ok else None,
                "erreur_mediane_deg_si_reussi": round(float(np.median([r["erreur_deg"] for r in ok])), 3) if ok else None,
                "temps_median_ms": round(float(np.median([r["temps_ms"] for r in sel])), 1),
                "temps_max_ms": round(float(np.max([r["temps_ms"] for r in sel])), 1),
            }
    return out


def table_md(summary: dict, uncertainty: dict) -> str:
    head = ("| Méthode | " + " | ".join(f"{d:.2f} m / {a:.0f}°" for d, a in LEVELS) +
            " | Total | Erreur (réussis) | Temps médian |\n|---|" + "---|" * (len(LEVELS) + 3) + "\n")
    out = []
    for jeu, title in (("simulation", "Simulation (vérité exacte, 13 scans)"),
                       ("réel, zone couverte", "Scans réels, dans la zone couverte par la carte (critère de l'énoncé)"),
                       ("réel, tous les scans", "Scans réels, tous (y compris hors zone couverte)")):
        lines = [f"### {title}\n", "Taux de réussite selon l'erreur de la pose initiale (réussite : < 5 cm et < 2°)\n\n", head]
        for m, s in summary[jeu].items():
            err = (f"{s['erreur_mediane_cm_si_reussi']:.1f} cm / {s['erreur_mediane_deg_si_reussi']:.2f}°"
                   if s["erreur_mediane_cm_si_reussi"] is not None else "—")
            lines.append(f"| {m} | " + " | ".join(f"{v:.0%}" for v in s["reussite_par_niveau"]) +
                         f" | **{s['reussite_totale']:.0%}** | {err} | {s['temps_median_ms']:.0f} ms |\n")
        out.append("".join(lines))
    out.append(f"\nIncertitude des références réelles (écart médian de chaque scan aux 12 autres) : "
               f"médiane {uncertainty['mediane_cm']} cm, de {uncertainty['min_cm']} à {uncertainty['max_cm']} cm.\n")
    return "\n".join(out)


COLORS = {"grid": "#1f6feb", "vecteur": "#d1242f", "scans": "#1a7f37"}
STYLES = {"exhaustive": ":", "multi-résolution": "--", "gradient": "-.", "Gauss-Newton": "-.", "calcul direct": ":",
          "ICP point à point": ":", "ICP point à droite": "-."}


def _style(method: str):
    kind, name = method.split(" · ")
    ls = "-" if "+" in name else STYLES.get(name, "-")
    return COLORS[kind], ls


def figures(summary: dict, rows: list, real_scans, real_names) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    x = [f"{d:.2f} m\n{a:.0f}°" for d, a in LEVELS]
    fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharey=True)
    for ax, jeu in zip(axes, ("simulation", "réel, zone couverte")):
        for m, s in summary[jeu].items():
            c, ls = _style(m)
            ax.plot(x, np.array(s["reussite_par_niveau"]) * 100, ls, color=c, marker="o", lw=2, label=m)
        ax.set_title(f"Fig. 1{'a' if jeu == 'simulation' else 'b'} - Taux de réussite ({jeu})")
        ax.set_xlabel("erreur de la pose initiale (position / angle)")
        ax.grid(True, color="#d0d7de")
        ax.set_ylim(-3, 103)
    axes[0].set_ylabel("réussite (%) : erreur < 5 cm et < 2°")
    axes[1].legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(FIG / "fig1_reussite_selon_initialisation.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5.5))
    names = list(METHODS)
    t = [summary["réel, zone couverte"][m]["temps_median_ms"] for m in names]
    ax.barh(names, t, color=[_style(m)[0] for m in names])
    ax.set_xscale("log")
    ax.invert_yaxis()
    for k, v in enumerate(t):
        ax.text(v * 1.1, k, f"{v:.0f} ms", va="center", fontsize=9)
    ax.set_title("Fig. 2 - Temps de calcul médian par localisation (scans réels, Python, un cœur)")
    ax.set_xlabel("temps (ms, échelle log)")
    ax.grid(True, axis="x", color="#d0d7de")
    fig.tight_layout()
    fig.savefig(FIG / "fig2_temps_de_calcul.png", dpi=130)
    plt.close(fig)

    _example_figure(real_scans, real_names)


def _example_figure(scans, names) -> None:
    """Un scan réel, une mauvaise pose initiale (1 m, 30°), et ce que trouvent trois méthodes sur la grid map."""
    k = 5
    others = [s for j, s in enumerate(scans) if j != k]
    refs = References.build(others)
    truth = scans[k].pose
    init = truth.moved(0.8, -0.6, 30)
    pts = scans[k].local_points()
    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    f = refs.field
    ext = (f.origin[0], f.origin[0] + f.dist.shape[1] * f.resolution, f.origin[1], f.origin[1] + f.dist.shape[0] * f.resolution)
    for ax, method in zip(axes, ("grid · gradient", "grid · multi-résolution", "grid · multi-résolution + gradient")):
        res = localize(method, pts, init, refs)
        ax.imshow(f.likelihood, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1.4)
        g0 = scans[k].global_points(init)
        g1 = scans[k].global_points(res.pose)
        ax.scatter(g0[:, 0], g0[:, 1], s=3, color="#fb8f44", label="pose initiale (1 m, 30° d'erreur)")
        ax.scatter(g1[:, 0], g1[:, 1], s=3, color="#1a7f37", label="pose trouvée")
        ax.plot(truth.x, truth.y, "k+", ms=14, mew=2, label="vraie pose")
        pos, ang = pose_error(res.pose, truth)
        ax.set_title(f"Fig. 3 - {method}\nerreur {pos * 100:.1f} cm, {ang:.2f}° en {res.time_s * 1000:.0f} ms", fontsize=11)
        ax.set_aspect("equal")
        ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(FIG / "fig3_exemple_localisation.png", dpi=110)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    names, real = load_real()
    walls = [s.as_tuple() for s in build_vector_map(real).segments]
    gaps = consistency([s.local_points() for s in real], [s.pose for s in real])
    uncertainty = {"par_scan_cm": [round(g * 100, 1) for g in gaps], "mediane_cm": round(float(np.median(gaps)) * 100, 1),
                   "min_cm": round(float(np.min(gaps)) * 100, 1), "max_cm": round(float(np.max(gaps)) * 100, 1)}
    rows: list = []
    t0 = time.time()
    print("Simulation (même salle, mêmes poses, scans simulés à vérité exacte)")
    run_dataset("simulation", simulated(real, walls), rows)
    print("Scans réels")
    run_dataset("réel", real, rows)
    print(f"{len(rows)} localisations en {time.time() - t0:.0f} s")
    with open(OUT / "essais.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    summary = summarize(rows)
    meta = {"niveaux_initialisation": LEVELS, "tirages_par_niveau": DRAWS, "seuil_reussite": [OK_POS_M, OK_ANG_DEG],
            "bruit_simulation_m": SIM_NOISE_M, "filtres": FILTERS.as_dict(), "scans": names,
            "incertitude_references_reelles": uncertainty}
    (OUT / "synthese.json").write_text(json.dumps({"protocole": meta, "resultats": summary}, ensure_ascii=False, indent=1),
                                      encoding="utf-8")
    (OUT / "tableau.md").write_text(table_md(summary, uncertainty), encoding="utf-8")
    figures(summary, rows, real, names)
    print((OUT / "tableau.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
