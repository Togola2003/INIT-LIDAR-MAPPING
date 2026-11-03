import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.matching import align_sequence, icp, rigid_transform, voxel_downsample
from lidar.scope.processing import Filters, isolated_mask
from lidar.simulate import simulate_scan


def test_rigid_transform_recovers_known_motion():
    rng = np.random.default_rng(0)
    a = rng.uniform(-2, 2, (50, 2))
    th = np.radians(12)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    b = a @ R.T + np.array([0.3, -0.5])
    theta, t = rigid_transform(a, b)
    assert np.degrees(theta) == pytest.approx(12, abs=1e-9)
    assert t == pytest.approx([0.3, -0.5], abs=1e-9)


@pytest.mark.parametrize("err", [(0.15, -0.10, 6.0), (-0.25, 0.20, -8.0)])
def test_icp_corrects_wrong_pose(err):
    truth = Pose(2.8, 1.2, 90)
    ref = simulate_scan(Pose(0.8, 0.8, 0), noise_m=0.005, seed=1).filtered().global_points()
    scan = simulate_scan(truth, noise_m=0.005, seed=2).filtered()
    start = truth.moved(*err)
    res = icp(scan.local_points(), start, ref)
    assert res.converged
    assert np.hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.03
    assert abs(res.pose.theta_deg - truth.theta_deg) < 1.0
    assert res.rms_m < 0.03


def test_align_sequence_keeps_anchor_and_fixes_others():
    truths = [Pose(0.8, 0.8, 0), Pose(2.8, 1.2, 90), Pose(1.2, 2.3, -45)]
    starts = [truths[0], truths[1].moved(0.1, -0.1, 5), truths[2].moved(-0.15, 0.1, -6)]
    scans = [simulate_scan(t, noise_m=0.005, seed=k).filtered().local_points() for k, t in enumerate(truths)]
    out = align_sequence(scans, starts, anchor=0)
    assert out[0][0] == truths[0] and out[0][1] is None
    for (pose, _), truth in zip(out[1:], truths[1:]):
        assert np.hypot(pose.x - truth.x, pose.y - truth.y) < 0.05
        assert abs(pose.theta_deg - truth.theta_deg) < 1.5


def test_voxel_downsample_reduces_and_keeps_shape():
    pts = np.column_stack((np.linspace(0, 1, 1000), np.zeros(1000)))
    out = voxel_downsample(pts, 0.05)
    assert 15 <= len(out) <= 25
    assert out[:, 0].min() < 0.05 and out[:, 0].max() > 0.95


def test_isolated_points_are_flagged():
    wall = np.column_stack((np.linspace(0, 2, 60), np.zeros(60)))
    lonely = np.array([[1.0, 1.5], [3.0, 3.0]])
    mask = isolated_mask(np.vstack([wall, lonely]), radius=0.15, min_neighbors=2)
    assert not mask[:60].any() and mask[60:].all()


def test_neighbor_filter_in_filters():
    scan = simulate_scan(Pose(1, 1, 0), noise_m=0.0)
    f = Filters(min_neighbors=2)
    assert len(f.apply(scan)) + len(f.rejected(scan)) == len(scan.filtered(0.05, 12.0))


def test_move_rigid_rotates_group_around_pivot():
    from lidar.geometry import local_to_global, move_rigid
    a, b = Pose(1, 0, 0), Pose(3, 0, 90)
    pivot = (2.0, 0.0)
    a2, b2 = move_rigid(a, dtheta_deg=90, pivot=pivot), move_rigid(b, dtheta_deg=90, pivot=pivot)
    assert (a2.x, a2.y, a2.theta_deg) == pytest.approx((2, -1, 90))
    assert (b2.x, b2.y, b2.theta_deg) == pytest.approx((2, 1, 180))
    # un point vu par les deux scans reste au même endroit relatif : le groupe est rigide
    pa = local_to_global(np.array([[1.0, 0.0]]), a)[0]          # (2, 0) vu depuis a
    pa2 = local_to_global(np.array([[1.0, 0.0]]), a2)[0]
    assert pa2 == pytest.approx([2, 0])                           # le pivot ne bouge pas
    assert move_rigid(a, 0.5, -0.2) == Pose(1.5, -0.2, 0)


def test_refine_global_fixes_all_poses_together():
    from lidar.matching import refine_global
    truths = [Pose(0.8, 0.8, 0), Pose(2.8, 1.2, 90), Pose(1.2, 2.3, -45), Pose(3.2, 2.5, 180)]
    starts = [truths[0], truths[1].moved(0.15, -0.1, 6), truths[2].moved(-0.2, 0.1, -7),
              truths[3].moved(0.1, 0.15, 4)]
    scans = [simulate_scan(t, noise_m=0.005, seed=k).filtered().local_points() for k, t in enumerate(truths)]
    res = refine_global(scans, starts, anchor=0)
    assert res.poses[0] == truths[0]
    for pose, truth in zip(res.poses, truths):
        assert np.hypot(pose.x - truth.x, pose.y - truth.y) < 0.01
        assert abs(pose.theta_deg - truth.theta_deg) < 0.3
    assert res.median_after < res.median_before / 3
