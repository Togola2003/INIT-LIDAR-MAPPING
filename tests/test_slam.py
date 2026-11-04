"""SLAM pas à pas : un tour complet de la salle simulée, chaque scan posé sur les précédents."""

import math

import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.localization import pose_error
from lidar.matching import align_best
from lidar.simulate import simulate_scan
from lidar.slam import SlamMap

# salle de 6 × 4 m avec deux obstacles (les mêmes que tests/test_localization.py)
WALLS = [(0, 0, 6, 0), (6, 0, 6, 4), (6, 4, 0, 4), (0, 4, 0, 0), (1, 1, 2.2, 1), (2.2, 1, 2.2, 1.6),
         (4.2, 2.6, 5, 2.6), (5, 2.6, 5, 3.3)]
# tour de la salle par petits pas (≈ 0,7 m, 10 à 30°), départ dans un coin, retour près du départ
TOUR = [(0.6, 0.5, 0), (1.3, 0.5, 15), (2.6, 0.6, 30), (3.3, 0.7, 40), (4.0, 0.8, 60), (4.7, 1.0, 80),
        (5.3, 1.6, 100), (5.4, 2.2, 120), (4.8, 2.2, 150), (4.0, 2.4, 170), (3.3, 3.0, 190), (2.6, 3.3, 200),
        (1.9, 3.3, 220), (1.2, 3.2, 240), (0.6, 2.6, 260), (0.5, 1.9, 280), (0.5, 1.2, 300), (0.7, 0.6, 330)]


def relative(t0: Pose, t: Pose) -> Pose:
    """Pose t dans le repère du 1er scan (= repère de la carte SLAM)."""
    c, s = math.cos(t0.theta), math.sin(t0.theta)
    dx, dy = t.x - t0.x, t.y - t0.y
    return Pose(c * dx + s * dy, -s * dx + c * dy, t.theta_deg - t0.theta_deg)


def _scan(truth: Pose, seed: int):
    return simulate_scan(truth, walls=WALLS, noise_m=0.01, seed=seed).filtered()


def _run_tour():
    truths = [Pose(*p) for p in TOUR]
    kept = [_scan(truths[0], 0).with_pose(Pose())]            # 1er scan = première référence (origine)
    steps = []
    for k, truth in enumerate(truths[1:], start=1):
        slam = SlamMap(kept)
        scan = _scan(truth, k)
        step = slam.locate(scan.local_points(), kept[-1].pose)   # déplacé « à la main » : départ = pose d'avant
        steps.append(step)
        kept.append(scan.with_pose(step.pose))                  # référence + nouveau scan = nouvelle référence
    return truths, kept, steps


@pytest.fixture(scope="module")
def tour():
    return _run_tour()


def test_each_scan_is_placed_on_the_previous_ones(tour):
    truths, kept, steps = tour
    for k, (scan, step) in enumerate(zip(kept[1:], steps), start=1):
        e, a = pose_error(scan.pose, relative(truths[0], truths[k]))
        assert e < 0.05 and a < 2.0, f"scan {k + 1} : {e * 100:.1f} cm, {a:.1f}°"
        assert step.ok, f"scan {k + 1} : recouvrement {step.overlap:.0%}"


def test_scale_does_not_drift(tour):
    """Distance entre le 1er et le dernier scan : la même dans la carte et dans la salle (pas de zoom)."""
    truths, kept, _ = tour
    real = math.hypot(truths[-1].x - truths[0].x, truths[-1].y - truths[0].y)
    mapped = math.hypot(kept[-1].pose.x, kept[-1].pose.y)
    assert abs(mapped - real) < 0.05


def test_new_part_and_loop_closure(tour):
    truths, kept, steps = tour
    assert steps[1].new_share > 0.05                             # le 3e scan découvre une partie nouvelle
    assert all(s.new_share < 0.65 for s in steps)                # petits pas : jamais trop de nouveau d'un coup
    last = steps[-1]
    assert 0 in last.loops(len(kept) - 1)                        # le dernier scan retrouve le 1er : boucle


def test_global_optimization_keeps_the_map(tour):
    truths, kept, _ = tour
    res, _ = align_best([s.local_points() for s in kept], [s.pose for s in kept], 0)
    errs = [pose_error(p, relative(truths[0], t))[0] for p, t in zip(res.poses, truths)]
    assert max(errs) < 0.05
    assert res.median_after <= res.median_before + 0.005


def test_scan_from_elsewhere_is_flagged():
    """Un scan pris dans un tout autre endroit (ici une pièce ronde) n'a rien en commun avec la carte :
    il ne doit pas être déclaré sûr."""
    slam = SlamMap([_scan(Pose(0.6, 0.5, 0), 0).with_pose(Pose())])
    circle = [(1.3 * math.cos(math.radians(a)), 1.3 * math.sin(math.radians(a)),
               1.3 * math.cos(math.radians(a + 10)), 1.3 * math.sin(math.radians(a + 10))) for a in range(0, 360, 10)]
    far = simulate_scan(Pose(0, 0, 0), walls=circle, noise_m=0.01).filtered()
    assert not slam.locate(far.local_points(), Pose()).ok


def test_poor_scan_raises():
    slam = SlamMap([_scan(Pose(1, 2, 0), 0).with_pose(Pose())])
    with pytest.raises(ValueError):
        slam.locate(np.zeros((5, 2)), Pose())
