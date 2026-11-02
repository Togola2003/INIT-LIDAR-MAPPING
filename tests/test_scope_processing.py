import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.scope.processing import Filters, average_scans, scan_stats
from lidar.simulate import simulate_scan


def test_average_reduces_noise():
    truth = simulate_scan(Pose(1, 1, 0), noise_m=0.0)
    noisy = [simulate_scan(Pose(1, 1, 0), noise_m=0.03, seed=k) for k in range(7)]
    err_one = np.abs(noisy[-1].ranges_m - truth.ranges_m).mean()
    err_avg = np.abs(average_scans(noisy).ranges_m - truth.ranges_m).mean()
    assert err_avg < err_one * 0.7


def test_average_ignores_no_echo():
    a = simulate_scan(Pose(1, 1, 0), noise_m=0.0)
    holes = a.ranges_m.copy()
    holes[::2] = 0.0
    from dataclasses import replace
    avg = average_scans([replace(a, ranges_m=holes), a])
    assert np.all(avg.ranges_m > 0)


def test_stats_directions_on_known_room():
    st = scan_stats(simulate_scan(Pose(0.8, 0.8, 0), noise_m=0.0), Filters())
    assert st["directions"]["Devant"] == pytest.approx(3.2, abs=0.02)
    assert st["directions"]["Gauche"] == pytest.approx(2.2, abs=0.02)
    assert st["no_echo_pct"] == 0


def test_filters_limit_range():
    st = scan_stats(simulate_scan(Pose(0.8, 0.8, 0)), Filters(max_range_m=1.0))
    assert st["max_range"] <= 1.0 and st["kept"] < st["points"]
