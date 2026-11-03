"""Carte vectorielle 2D : liste de segments (features) construite scan après scan.

Pour chaque scan (cf. docs/jour1.md, §5) :
1. extraction des segments du scan dans le repère carte (extraction.scan_to_segments) ;
2. association : un nouveau segment « proche » (segments.are_close) d'une feature existante
   est fusionné avec elle (segments.merge) et sa confiance augmente ; sinon il est ajouté ;
3. détection d'absence : chaque faisceau robot -> impact (raccourci d'une marge) qui
   COUPE une feature prouve qu'il y a du vide à cet endroit. Si assez de faisceaux traversent
   une feature, on la découpe (segments.cut, on garde le plus long côté) et sa confiance baisse ;
4. nettoyage : fusion des features redondantes, suppression des trop courtes / confiance nulle.

    python -m lidar.vectormap data/simu/*.csv --out out/vector_map.txt
    python -m lidar.vectormap data/simu/*.csv --grid out/grid.npz   # superposé à la grid map
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from .extraction import ExtractionParams, scan_to_segments
from .scan import Scan
from .segments import EPS, Segment, are_close, cut, merge


@dataclass(frozen=True)
class Feature:
    segment: Segment
    confidence: int = 1
    last_seen: int = 0


@dataclass(frozen=True)
class VectorMapParams:
    extraction: ExtractionParams = ExtractionParams()
    assoc_angle_deg: float = 10.0
    assoc_offset_m: float = 0.08
    assoc_gap_m: float = 0.15
    beam_margin_m: float = 0.10      # on ignore les 10 derniers cm du faisceau (bruit autour de l'impact)
    min_crossings: int = 3           # faisceaux traversants nécessaires pour déclarer une absence
    min_length_m: float = 0.10
    max_confidence: int = 20


@dataclass(frozen=True)
class VectorMap:
    features: tuple[Feature, ...] = ()
    n_scans: int = 0

    @property
    def segments(self) -> list[Segment]:
        return [f.segment for f in self.features]


def _associate(features: list[Feature], seg: Segment, now: int, p: VectorMapParams) -> list[Feature]:
    for k, f in enumerate(features):
        if are_close(f.segment, seg, p.assoc_angle_deg, p.assoc_offset_m, p.assoc_gap_m):
            merged = Feature(merge(f.segment, seg), min(f.confidence + 1, p.max_confidence), now)
            return features[:k] + [merged] + features[k + 1:]
    return features + [Feature(seg, 1, now)]


def _beams(scan: Scan, margin: float) -> list[Segment]:
    s = scan.filtered()
    keep = s.ranges_m > margin
    shortened = replace(s, angles_deg=s.angles_deg[keep], ranges_m=s.ranges_m[keep] - margin,
                        intensities=s.intensities[keep])
    o = np.array([scan.pose.x, scan.pose.y])
    return [Segment.from_points(o, q) for q in shortened.global_points()]


def _crossing_mask(seg: Segment, starts: np.ndarray, ends: np.ndarray) -> np.ndarray:
    """Même test que segments.intersection, mais pour TOUS les faisceaux d'un coup (numpy)."""
    v1 = seg.vector
    v2 = ends - starts
    den = v1[0] * v2[:, 1] - v1[1] * v2[:, 0]
    w = starts - seg.p1
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[:, 0] * v2[:, 1] - w[:, 1] * v2[:, 0]) / den
        u = (w[:, 0] * v1[1] - w[:, 1] * v1[0]) / den
    lengths = np.hypot(v2[:, 0], v2[:, 1])
    parallel = np.abs(den) <= EPS * np.maximum(seg.length * lengths, EPS)
    tol1 = EPS / max(seg.length, EPS)
    tol2 = EPS / np.maximum(lengths, EPS)
    return ~parallel & (t >= -tol1) & (t <= 1 + tol1) & (u >= -tol2) & (u <= 1 + tol2)


def _apply_absences(features: list[Feature], beams: list[Segment], p: VectorMapParams) -> list[Feature]:
    out = []
    if not beams:
        return list(features)
    starts = np.array([b.p1 for b in beams])
    ends = np.array([b.p2 for b in beams])
    for f in features:
        crossing = [beams[k] for k in np.flatnonzero(_crossing_mask(f.segment, starts, ends))]
        if len(crossing) < p.min_crossings:
            out.append(f)
            continue
        # découpe par le faisceau médian (le plus représentatif de la zone vide)
        new_seg = cut(f.segment, crossing[len(crossing) // 2])
        out.append(Feature(new_seg, f.confidence - 1, f.last_seen))
    return out


def _cleanup(features: list[Feature], p: VectorMapParams) -> list[Feature]:
    # fusion des features devenues redondantes
    merged: list[Feature] = []
    for f in sorted(features, key=lambda f: -f.segment.length):
        for k, g in enumerate(merged):
            if are_close(g.segment, f.segment, p.assoc_angle_deg, p.assoc_offset_m, p.assoc_gap_m):
                merged[k] = Feature(merge(g.segment, f.segment), max(g.confidence, f.confidence),
                                    max(g.last_seen, f.last_seen))
                break
        else:
            merged.append(f)
    return [f for f in merged if f.confidence > 0 and f.segment.length >= p.min_length_m]


def update_vector_map(vmap: VectorMap, scan: Scan, params: VectorMapParams = VectorMapParams()) -> VectorMap:
    """Retourne une NOUVELLE carte intégrant le scan (la carte d'entrée n'est pas modifiée)."""
    now = vmap.n_scans + 1
    # absences d'abord : on confronte la carte EXISTANTE aux faisceaux du nouveau scan,
    # puis on y ajoute ce que le scan voit (sinon un mur ré-observé en partie ne serait jamais raccourci)
    features = _apply_absences(list(vmap.features), _beams(scan, params.beam_margin_m), params)
    for seg in scan_to_segments(scan, params.extraction):
        features = _associate(features, seg, now, params)
    return VectorMap(tuple(_cleanup(features, params)), now)


def build_vector_map(scans: list[Scan], params: VectorMapParams = VectorMapParams()) -> VectorMap:
    vmap = VectorMap()
    for s in scans:
        vmap = update_vector_map(vmap, s, params)
    return vmap


def save_vector_map(vmap: VectorMap, path: str | Path) -> Path:
    """Format du module : une ligne x1,y1,x2,y2 par segment, commentaires '#'."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"# Carte vectorielle (segments) - {vmap.n_scans} scan(s), {len(vmap.features)} segment(s)",
             "# x1,y1,x2,y2 (mètres)"]
    lines += [",".join(f"{v:.4f}" for v in f.segment.as_tuple()) for f in vmap.features]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def load_vector_map(path: str | Path) -> VectorMap:
    feats = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            feats.append(Feature(Segment(*(float(v) for v in line.split(",")[:4]))))
    return VectorMap(tuple(feats))


def plot_vector_map(vmap: VectorMap, ax=None, scans: list[Scan] | None = None):
    import matplotlib.pyplot as plt

    ax = ax or plt.gca()
    for s in scans or []:
        pts = s.filtered().global_points()
        ax.scatter(pts[:, 0], pts[:, 1], s=1, color="0.6")
        ax.plot(s.pose.x, s.pose.y, "r^", markersize=8)
    cmap = plt.get_cmap("viridis")
    for f in vmap.features:
        sg = f.segment
        ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", lw=2.5, color=cmap(min(f.confidence / 5, 1.0)))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    return ax


def main() -> None:
    import matplotlib.pyplot as plt

    from .gridmap import _expand, load_grid, plot_grid
    from .poses import load_scans_with_poses

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scans", nargs="+")
    ap.add_argument("--poses", default="data/poses.json")
    ap.add_argument("--out", default="out/vector_map.txt")
    ap.add_argument("--grid", help="grid map .npz à afficher en fond")
    args = ap.parse_args()

    scans = load_scans_with_poses(_expand(args.scans), args.poses)
    vmap = build_vector_map(scans)
    out = save_vector_map(vmap, args.out)
    ax = plt.gca()
    if args.grid:
        plot_grid(load_grid(args.grid), ax)
    plot_vector_map(vmap, ax, scans)
    ax.set_title(f"{len(vmap.features)} segments à partir de {len(scans)} scan(s)")
    png = out.with_suffix(".png")
    plt.savefig(png, dpi=150)
    print(f"carte : {out}  image : {png}")
    plt.show()


if __name__ == "__main__":
    main()
