import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.localization import METHODS, References, localize, pose_error
from lidar.localization.grid import LikelihoodField
from lidar.localization.vector import SegmentMap
from lidar.segments import Segment
from lidar.simulate import simulate_scan

MAP_POSES = [Pose(0.8, 0.8, 0), Pose(2.8, 1.2, 90), Pose(1.2, 2.3, -45), Pose(3.2, 2.5, 180)]
TRUTH = Pose(1.6, 1.5, 20)


@pytest.fixture(scope="module")
def setup():
    maps = [simulate_scan(p, noise_m=0.005, seed=k).filtered() for k, p in enumerate(MAP_POSES)]
    test = simulate_scan(TRUTH, noise_m=0.01, seed=9).filtered()
    refs = References.build(maps, reference_points=maps[2].global_points())
    return refs, test.local_points()


@pytest.mark.parametrize("method", list(METHODS))
def test_every_method_recovers_a_close_initial_pose(setup, method):
    refs, pts = setup
    res = localize(method, pts, TRUTH.moved(0.15, -0.10, 6), refs)
    pos, ang = pose_error(res.pose, TRUTH)
    assert pos < 0.05 and ang < 2.0
    assert 0.0 <= res.score <= 1.0 and res.time_s > 0


@pytest.mark.parametrize("method", [m for m in METHODS if "multi-résolution" in m])
def test_multiresolution_survives_a_bad_initial_pose(setup, method):
    refs, pts = setup
    res = localize(method, pts, TRUTH.moved(0.8, -0.6, 30), refs)
    pos, ang = pose_error(res.pose, TRUTH)
    assert pos < 0.05 and ang < 2.0


def test_likelihood_field_distances_and_pyramid():
    occ = np.zeros((20, 20), dtype=bool)
    occ[10, :] = True                                   # un mur horizontal en y = 0,5 m (cases de 5 cm)
    f = LikelihoodField.from_mask(occ, 0.05, (0.0, 0.0))
    assert f.dist[10, 3] == 0 and f.dist[14, 3] == pytest.approx(0.20)
    c = f.coarser(2)
    assert c.resolution == pytest.approx(0.10) and c.occupied.shape == (10, 10) and c.occupied[5].all()
    on_wall = np.array([[0.30, 0.525], [0.6, 0.525]])
    assert f.score(on_wall) > 0.9 and f.score(on_wall + [0, 0.4]) < 0.01


def test_segment_map_distance_and_feet():
    segmap = SegmentMap.from_segments([Segment(0, 0, 2, 0), Segment(2, 0, 2, 1)])
    d, feet = segmap.nearest(np.array([[1.0, 0.3], [3.0, 0.5], [-1.0, 0.0]]))
    assert d == pytest.approx([0.3, 1.0, 1.0])
    assert feet[1] == pytest.approx([2.0, 0.5]) and feet[2] == pytest.approx([0.0, 0.0])


def test_pose_error_wraps_angles():
    assert pose_error(Pose(0, 0, 359), Pose(0, 0, 1)) == (0.0, pytest.approx(2.0))


def test_missing_reference_is_reported():
    with pytest.raises(ValueError):
        localize("scans · ICP point à point", np.zeros((20, 2)), Pose(), References())


def test_global_localization_without_initial_pose():
    from lidar.localization.globalloc import global_localize
    from lidar.refmap import build_reference
    walls = [(0, 0, 6, 0), (6, 0, 6, 4), (6, 4, 0, 4), (0, 4, 0, 0), (1, 1, 2.2, 1), (2.2, 1, 2.2, 1.6),
             (4.2, 2.6, 5, 2.6), (5, 2.6, 5, 3.3)]
    poses = [Pose(1, 3, 0), Pose(3, 2, 45), Pose(5, 1, 100), Pose(4, 3.4, -90)]
    maps = [simulate_scan(p, walls=walls, noise_m=0.005, seed=k).filtered() for k, p in enumerate(poses)]
    ref = build_reference(maps, [str(k) for k in range(len(maps))])
    truth = Pose(2.6, 2.7, 133)
    scan = simulate_scan(truth, walls=walls, noise_m=0.01, seed=11).filtered()
    res = global_localize(scan.local_points(), ref.grid, ref.segments)
    pos, ang = pose_error(res.pose, truth)
    assert pos < 0.05 and ang < 2.0
    assert res.inliers > 0.6 and not res.ambiguous and res.positions > 50
