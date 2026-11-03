import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.scope.collector import Collection, finish, merge_all, range_spread
from lidar.scope.sources import Frame
from lidar.simulate import simulate_scan


def _frames(n, noise=0.01):
    return [Frame(simulate_scan(Pose(), noise_m=noise, seed=k), "printScanCSV", "") for k in range(n)]


def test_collection_counts_until_target():
    col = Collection(3, Pose(1, 2, 90))
    for f in _frames(3):
        assert not col.done
        col = col.add(f)
    assert col.done and col.count == 3 and col.progress == 1.0


def test_finish_uses_click_pose_and_reports_quality():
    col = Collection(5, Pose(1, 2, 90))
    for f in _frames(5):
        col = col.add(f)
    scan, info = finish(col, "Médiane")
    assert scan.pose == Pose(1, 2, 90)
    assert info["tours"] == 5 and info["points"] > 100
    assert 0.002 < info["bruit_m"] < 0.03


def test_merge_all_is_denser_than_median():
    frames = _frames(4)
    merged = merge_all([f.scan for f in frames])
    assert len(merged) > 3 * int((frames[0].scan.ranges_m > 0).sum())
    assert np.all(np.diff(merged.angles_deg) >= 0)


def test_spread_needs_two_tours_and_grows_with_noise():
    assert range_spread([_frames(1)[0].scan]) is None
    quiet = range_spread([f.scan for f in _frames(5, 0.002)])
    noisy = range_spread([f.scan for f in _frames(5, 0.03)])
    assert quiet < noisy


def test_finish_without_frames_fails():
    with pytest.raises(ValueError):
        finish(Collection(3, Pose()))
