import pytest

from lidar.scope.sources import MIN_POINTS, LineParser


def _csv_lines(n: int) -> list[str]:
    return [f"{k},{k * 0.8:.2f},{1000 + k},200" for k in range(n)]


def test_csv_scan_emitted_on_blank_line():
    p = LineParser()
    events = [ev for line in ["N,Angle(°),Distance(mm),Intensity"] + _csv_lines(150) + [""] for ev in p.feed(line)]
    assert len(events) == 1 and events[0].kind == "scan"
    scan = events[0].payload
    assert len(scan) == 150
    assert scan.ranges_m[0] == pytest.approx(1.0)


def test_truncated_csv_scan_is_dropped():
    p = LineParser()
    events = [ev for line in _csv_lines(MIN_POINTS - 1) + [""] for ev in p.feed(line)]
    assert events == []


def test_teleplot_xy_corrects_library_mirror():
    # la lib écrit y = -d·sin(angle) : un point « maths » à 90° (gauche), 1 m, arrive en x=0, y=-1000
    pairs = ";".join(["0:-1000"] * MIN_POINTS)
    (ev,) = LineParser().feed(f">lidar:{pairs};|xy")
    assert ev.kind == "scan"
    assert ev.payload.angles_deg[0] == pytest.approx(90.0)
    assert ev.payload.ranges_m[0] == pytest.approx(1.0)


def test_teleplot_scalar_is_telemetry():
    (ev,) = LineParser().feed(">speed:2152.5")
    assert ev.kind == "telemetry" and ev.payload == ("speed", 2152.5)


def test_garbage_is_ignored():
    p = LineParser()
    assert p.feed("blabla") == [] and p.feed(">lidar:abc|xy") == []
    assert p.feed("") == []
