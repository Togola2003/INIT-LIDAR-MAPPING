"""SLAM pas à pas (« stop and go ») : localiser chaque nouveau scan sur la carte déjà construite, puis l'ajouter.

C'est la consigne du Jour 3 (Simultaneous Localization And Mapping) :
  1. 1er scan, n'importe où : il devient la PREMIÈRE RÉFÉRENCE (origine 0, 0, 0 de la carte) ;
  2. on déplace un peu le robot (à la main ou en Wi-Fi), on refait un scan ;
  3. LOCALISATION : ce scan a forcément des points en commun avec la référence. On cherche la pose
     (x, y, θ) qui pose ces points communs sur la carte :
       a. grossier : recherche sur la grid map (multi-résolution + gradient, `localization.tracking.place_scan`,
          même famille que Hector SLAM et Google Cartographer), autour de la pose prévue, sinon partout ;
       b. fin : ICP sur les points de tous les scans gardés (précision de l'ordre du cm, plus fine qu'une case) ;
  4. CARTOGRAPHIE : les points du scan qui ne tombent pas sur la carte sont la PARTIE NOUVELLE. Le scan est
     ajouté : référence + nouveau scan = nouvelle référence ;
  5. on recommence par petits pas jusqu'à avoir toute la salle ;
  6. FERMETURE DE BOUCLE : quand un scan retrouve un scan ancien (on revient près du départ), l'erreur
     accumulée de proche en proche se voit. L'optimisation globale (`matching.align_best`) recale alors
     toutes les poses ensemble.

L'échelle ne peut pas dériver : le lidar mesure en mètres, poser un scan ne fait que le tourner et le déplacer.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .geometry import Pose, local_to_global
from .localization.grid import LikelihoodField, build_pyramid
from .localization.tracking import place_scan
from .matching import agreement, icp, voxel_downsample
from .refmap import ReferenceMap, build_reference
from .scan import Scan

COMMON_M = 0.08          # un point à moins de 8 cm de la carte est « commun » ; au-delà, il est « nouveau »
GOOD_OVERLAP = 0.35      # au moins 35 % de points communs : placement sûr
LOOP_SHARE = 0.25        # un scan ancien qui partage ≥ 25 % des points : la boucle se referme
ICP_MAX_JUMP_M, ICP_MAX_JUMP_DEG = 0.30, 10.0   # l'ICP fin ne doit que corriger un peu le placement grossier
MIN_POINTS = 20


@dataclass(frozen=True)
class SlamStep:
    """Résultat de la localisation d'un nouveau scan sur la carte en construction."""
    pose: Pose
    overlap: float                  # part des points du scan déjà sur la carte (commun)
    common: np.ndarray              # masque des points communs (les autres sont la partie nouvelle)
    how: str
    ambiguous: bool
    shared: tuple[tuple[int, float], ...]   # (indice d'un scan gardé, part des points partagés avec lui)
    time_s: float

    @property
    def ok(self) -> bool:
        return self.overlap >= GOOD_OVERLAP and not self.ambiguous

    @property
    def new_share(self) -> float:
        return 1.0 - self.overlap

    def loops(self, n_scans: int, recent: int = 2) -> list[int]:
        """Scans ANCIENS (pas les `recent` derniers) retrouvés par ce scan : signe d'une boucle refermée."""
        return [k for k, share in self.shared if share >= LOOP_SHARE and k < n_scans - recent]


class SlamMap:
    """La référence courante : tous les scans gardés (filtrés, avec leur pose), en grid map + nuage de points."""

    def __init__(self, scans: list[Scan], labels: list[str] | None = None, edits=()) -> None:
        if not scans:
            raise ValueError("la carte SLAM a besoin d'au moins un scan")
        self.scans = list(scans)
        self.ref: ReferenceMap = build_reference(self.scans, labels or [f"scan {k + 1}" for k in range(len(scans))],
                                                 edits)
        self.field = LikelihoodField.from_grid(self.ref.grid)
        self.pyramid = build_pyramid(self.field)
        clouds = [voxel_downsample(s.global_points(), 0.03) for s in self.scans]
        self._trees = [cKDTree(c) for c in clouds]          # un arbre par scan : « avec qui ce scan est-il commun ? »
        self.cloud = np.vstack(clouds)
        self._tree = cKDTree(self.cloud)

    @property
    def grid(self):
        return self.ref.grid

    @property
    def segments(self):
        return self.ref.segments

    def __len__(self) -> int:
        return len(self.scans)

    def locate(self, points_local: np.ndarray, guess: Pose | None) -> SlamStep:
        """Pose d'un nouveau scan (pris à l'arrêt) sur la carte : grossier sur la grid map, puis ICP fin."""
        t0 = time.perf_counter()
        if len(points_local) < MIN_POINTS:
            raise ValueError("scan trop pauvre (moins de 20 points) : vérifie le lidar ou les filtres")
        coarse = place_scan(points_local, guess, self.ref.grid, self.ref.segments, self.field, self.pyramid)
        pose, how = coarse.pose, coarse.how
        fine = self._refine(points_local, coarse.pose)
        if fine is not None:
            pose, how = fine, how + " + ICP fin"
        g = local_to_global(points_local, pose)
        common = np.isfinite(self._tree.query(g, distance_upper_bound=COMMON_M)[0])
        shares = [float(np.isfinite(t.query(g, distance_upper_bound=COMMON_M)[0]).mean()) for t in self._trees]
        shared = tuple((k, v) for k, v in enumerate(shares) if v > 0)
        return SlamStep(pose, float(common.mean()), common, how, coarse.ambiguous, shared,
                        time.perf_counter() - t0)

    def _refine(self, points_local: np.ndarray, start: Pose) -> Pose | None:
        """ICP sur le nuage de la carte ; gardé seulement s'il reste proche et s'accorde mieux avec la carte."""
        pts = voxel_downsample(points_local, 0.02)
        res = icp(pts, start, self.cloud, max_dist_schedule=(0.25, 0.12, 0.06), iterations_per_stage=12)
        if not res.converged:
            return None
        jump = np.hypot(res.pose.x - start.x, res.pose.y - start.y)
        turn = abs((res.pose.theta_deg - start.theta_deg + 180) % 360 - 180)
        if jump > ICP_MAX_JUMP_M or turn > ICP_MAX_JUMP_DEG:
            return None
        before = agreement(pts, start, self.cloud, 0.1)
        after = agreement(pts, res.pose, self.cloud, 0.1)
        return res.pose if after >= before else None
