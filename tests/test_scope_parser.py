import numpy as np
import pytest

from lidar.scope.sources import FORMAT_CSV, FORMAT_TELEPLOT, MIN_POINTS, LineParser


def _csv_lines(n: int) -> list[str]:
    return [f"{k},{k * 0.8:.2f},{1000 + k},200" for k in range(n)]


def test_csv_scan_emitted_on_blank_line():
    p = LineParser()
    events = [ev for line in ["N,Angle(°),Distance(mm),Intensity"] + _csv_lines(150) + [""] for ev in p.feed(line)]
    assert len(events) == 1 and events[0].kind == "scan"
    frame = events[0].payload
    assert frame.fmt == FORMAT_CSV and len(frame.raw) == 150      # lignes brutes conservées
    assert len(frame.scan) == 150
    assert frame.scan.ranges_m[0] == pytest.approx(1.0)
    assert frame.scan.intensities[0] == 200


def test_truncated_csv_scan_is_dropped():
    p = LineParser()
    events = [ev for line in _csv_lines(MIN_POINTS - 1) + [""] for ev in p.feed(line)]
    assert events == []


def test_teleplot_xy_corrects_library_mirror():
    # la lib écrit y = -d·sin(angle) : un point « maths » à 90° (gauche), 1 m, arrive en x=0, y=-1000
    pairs = ";".join(["0:-1000"] * MIN_POINTS)
    line = f">lidar:{pairs};|xy"
    (ev,) = LineParser().feed(line)
    frame = ev.payload
    assert frame.fmt == FORMAT_TELEPLOT and frame.raw == (line,)
    assert frame.scan.angles_deg[0] == pytest.approx(90.0)
    assert frame.scan.ranges_m[0] == pytest.approx(1.0)
    assert np.isnan(frame.scan.intensities).all()           # intensité inconnue, pas inventée


def test_teleplot_no_echo_points_get_interpolated_angle():
    # 120 points de 10° à 129° (sens maths), dont un sans écho (0:0) au milieu
    pts = []
    for k in range(120):
        a = np.radians(10 + k)
        pts.append("0:0" if k == 50 else f"{1000 * np.cos(a):.1f}:{-1000 * np.sin(a):.1f}")
    (ev,) = LineParser().feed(">lidar:" + ";".join(pts) + "|xy")
    s = ev.payload.scan
    assert s.ranges_m[50] == 0
    assert s.angles_deg[50] == pytest.approx(60.0, abs=0.01)
    assert len(s.filtered()) == 119


def test_teleplot_scalar_is_telemetry():
    (ev,) = LineParser().feed(">speed:2152.5")
    assert ev.kind == "telemetry" and ev.payload == ("speed", 2152.5)


def test_garbage_is_ignored():
    p = LineParser()
    assert p.feed("blabla") == [] and p.feed(">lidar:abc|xy") == []
    assert p.feed("") == []
