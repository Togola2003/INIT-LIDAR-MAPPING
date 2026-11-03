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
