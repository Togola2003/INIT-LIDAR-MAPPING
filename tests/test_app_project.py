import numpy as np
import pytest

from lidar.app import project as P
from lidar.geometry import Pose
from lidar.segments import Segment
from lidar.simulate import simulate_scan


def _project(n=2):
    proj, nums = P.add_scans(P.Project(), [(f"s{k}", simulate_scan(Pose(1 + k, 1, 0), seed=k)) for k in range(n)])
    return proj, nums


def test_add_scans_numbers_and_colors():
    proj, nums = _project(3)
    assert nums == [1, 2, 3]
    assert len({c.color for c in proj.captures}) == 3
    proj2, more = P.add_scans(proj, [("x", simulate_scan(Pose()))])
    assert more == [4] and len(proj.captures) == 3          # le projet d'origine n'est pas modifié


def test_group_move_rotates_around_pivot():
    proj, _ = _project(2)
    pivot = P.pivot_of(proj, {1, 2})
    assert pivot == pytest.approx((1.5, 1.0))
    moved = P.move(proj, {1, 2}, 0, 0, 180, pivot)
    assert moved.find(1).scan.pose.x == pytest.approx(2.0)
    assert moved.find(2).scan.pose.theta_deg == pytest.approx(180)


def test_erase_points_in_box_keeps_angles():
    proj, _ = _project(1)
    before = proj.find(1).scan
    erased_proj, n = P.erase_points(proj, {1}, (3.0, -1.0, 6.0, 5.0))      # le mur x = 4 de la pièce simulée
    after = erased_proj.find(1).scan
    assert n > 10 and len(after) == len(before)
    assert ((after.ranges_m == 0).sum() - (before.ranges_m == 0).sum()) == n


def test_store_undo_redo_and_reset():
    store = P.Store()
    seen = []
    store.subscribe(lambda p: seen.append(len(p.captures)))
    proj, _ = _project(2)
    store.apply(proj)
    store.apply(P.remove(store.project, {1}))
    assert len(store.project.captures) == 1 and store.dirty
    assert store.undo() and len(store.project.captures) == 2
    assert store.redo() and len(store.project.captures) == 1
    store.reset()
    assert store.project.is_empty and not store.past and not store.dirty
    assert seen == [2, 1, 2, 1, 0]


def test_manual_segments_and_save_roundtrip(tmp_path):
    proj, _ = _project(2)
    proj = P.set_manual_segments(proj, [Segment(0, 0, 1, 0)])
    saved = P.save(proj, str(tmp_path / "essai.lidar.json"))
    assert saved.name == "essai" and saved.path
    reopened, nums, errors = P.open_files(P.Project(), [saved.path])
    assert nums == [1, 2] and not errors
    assert np.allclose(reopened.find(2).scan.ranges_m, proj.find(2).scan.ranges_m, atol=1e-4)   # format : 0,1 mm
    assert reopened.manual_segments == (Segment(0, 0, 1, 0),)


def test_open_files_reports_errors(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("{}", encoding="utf-8")
    _, nums, errors = P.open_files(P.Project(), [str(bad)])
    assert nums == [] and len(errors) == 1
