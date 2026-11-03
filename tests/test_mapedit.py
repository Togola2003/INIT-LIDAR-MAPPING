import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.gridmap import create_grid
from lidar.mapedit import ADD, DELETE, WallEdit, apply_to_grid, apply_to_segments, lies_on
from lidar.refmap import build_reference, from_dict, to_dict
from lidar.segments import Segment
from lidar.simulate import simulate_scan


def test_add_then_delete_segments():
    base = [Segment(0, 0, 4, 0), Segment(4, 0, 4, 3)]
    edits = [WallEdit(ADD, Segment(1, 1, 2, 1)), WallEdit(DELETE, Segment(4, -0.05, 4, 3.05))]
    out = apply_to_segments(base, edits)
    assert Segment(0, 0, 4, 0) in out and Segment(1, 1, 2, 1) in out and len(out) == 2
    assert apply_to_segments(out, [WallEdit(DELETE, Segment(1, 1, 2, 1))]) == [Segment(0, 0, 4, 0)]


def test_lies_on_needs_both_ends_close():
    assert lies_on(Segment(0, 0.05, 1, 0.05), Segment(-1, 0, 2, 0))
    assert not lies_on(Segment(0, 0, 1, 0.5), Segment(-1, 0, 2, 0))


def test_edits_change_the_grid_too():
    g = create_grid(2.0, 2.0, 0.05)
    added = apply_to_grid(g, [WallEdit(ADD, Segment(0.2, 1.0, 1.8, 1.0))])
    assert added.occupied_mask()[20, 4:36].all()
    removed = apply_to_grid(added, [WallEdit(DELETE, Segment(0.2, 1.0, 1.8, 1.0))])
    assert not removed.occupied_mask().any() and removed.free_mask()[20, 4:36].all()
    assert apply_to_grid(g, []) is g


def test_edit_serialization():
    e = WallEdit(DELETE, Segment(0, 1, 2, 3))
    assert WallEdit.from_dict(e.as_dict()) == e
    with pytest.raises(ValueError):
        WallEdit.from_dict({**e.as_dict(), "operation": "?"})


def test_reference_map_includes_edits_and_roundtrips():
    scans = [simulate_scan(p, seed=k).filtered() for k, p in enumerate([Pose(1, 1, 0), Pose(3, 2, 90)])]
    wall = Segment(1.5, 0.5, 1.5, 1.5)
    ref = build_reference(scans, ["s1", "s2"], [WallEdit(ADD, wall)])
    assert wall in ref.segments and ref.edits == 1
    i, j = ref.grid.world_to_cell(1.5, 1.0)
    assert ref.grid.occupied_mask()[j, i]
    back = from_dict(to_dict(ref))
    assert back.grid.origin == pytest.approx(ref.grid.origin)
    assert (back.grid.occupied_mask() == ref.grid.occupied_mask()).all()
    assert back.scans[1][1] == Pose(3, 2, 90) and "2 scans" in back.describe()
    with pytest.raises(ValueError):
        build_reference([], [])
