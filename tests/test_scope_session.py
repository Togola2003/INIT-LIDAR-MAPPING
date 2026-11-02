import json

import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.scope.session import (
    FORMAT_NAME, Capture, load_session, read_any, save_session, save_table,
)
from lidar.simulate import simulate_scan


@pytest.fixture
def captures():
    return [
        Capture(1, simulate_scan(Pose(0.8, 0.8, 0), seed=1), "#2ecc71", "salle A"),
        Capture(2, simulate_scan(Pose(2.8, 1.2, 90), seed=2), "#e67e22"),
    ]


def test_json_roundtrip_is_faithful(tmp_path, captures):
    path = save_session(captures, tmp_path / "s.lidar.json")
    back = load_session(path)
    assert [c.number for c in back] == [1, 2]
    assert back[0].label == "salle A" and back[1].color == "#e67e22"
    for a, b in zip(captures, back):
        assert b.scan.pose == a.scan.pose
        assert b.scan.angles_deg == pytest.approx(a.scan.angles_deg, abs=1e-3)
        assert b.scan.ranges_m == pytest.approx(a.scan.ranges_m, abs=1e-4)
        assert np.array_equal(b.scan.intensities, a.scan.intensities)


def test_json_is_self_describing(tmp_path, captures):
    data = json.loads(save_session(captures, tmp_path / "s.json").read_text(encoding="utf-8"))
    assert data["format"] == FORMAT_NAME
    d = data["description"]
    assert {"units", "angle_convention", "frames", "pose", "no_echo", "columns"} <= d.keys()
    cap = data["captures"][0]
    assert set(cap["columns"]) <= d["columns"].keys()          # chaque colonne est documentée
    assert len(cap["points"][0]) == len(cap["columns"])


def test_derived_map_coordinates_are_consistent(tmp_path, captures):
    data = json.loads(save_session(captures, tmp_path / "s.json").read_text(encoding="utf-8"))
    cap = data["captures"][1]   # pose (2.8, 1.2, 90°)
    i = cap["columns"].index
    row = next(r for r in cap["points"] if r[i("distance_m")] > 0)
    assert row[i("x_map_m")] == pytest.approx(2.8 - row[i("y_lidar_m")], abs=1e-3)
    assert row[i("y_map_m")] == pytest.approx(1.2 + row[i("x_lidar_m")], abs=1e-3)


def test_table_roundtrip(tmp_path, captures):
    back = read_any(save_table(captures, tmp_path / "t.csv"))
    assert len(back) == 2 and back[1][1].pose == Pose(2.8, 1.2, 90)
    assert len(back[0][1]) == len(captures[0].scan)


def test_wrong_json_rejected(tmp_path):
    p = tmp_path / "x.json"
    p.write_text('{"format": "autre"}', encoding="utf-8")
    with pytest.raises(ValueError):
        load_session(p)
