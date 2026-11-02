import numpy as np
import pytest

from lidar.segments import (
    EPS, Segment, angle_between, are_close, cut, intersection, merge, point_segment_distance,
)

H = Segment(0, 0, 4, 0)  # segment horizontal de 4 m


# ------------------------------------------------------------------ angle
@pytest.mark.parametrize(
    "s2, expected",
    [
        (Segment(0, 0, 0, 1), 90.0),
        (Segment(0, 1, 2, 1), 0.0),
        (Segment(2, 0, 0, 0), 0.0),          # même droite, sens inverse
        (Segment(0, 0, 1, 1), 45.0),
        (Segment(0, 0, -1, 1), 45.0),        # 135° -> 45° entre droites
        (Segment(0, 0, 1, np.tan(np.radians(179))), 1.0),
    ],
)
def test_angle_between(s2, expected):
    assert angle_between(H, s2) == pytest.approx(expected, abs=1e-9)


# ------------------------------------------------------------------ proximité
def test_close_when_aligned_and_overlapping():
    assert are_close(H, Segment(3, 0.01, 6, 0.02))


def test_close_when_small_gap():
    assert are_close(H, Segment(4.05, 0, 6, 0), max_gap_m=0.1)


def test_not_close_when_large_gap():
    assert not are_close(H, Segment(4.5, 0, 6, 0), max_gap_m=0.1)


def test_not_close_when_parallel_but_offset():
    assert not are_close(H, Segment(0, 0.5, 4, 0.5))


def test_not_close_when_angle_too_large():
    assert not are_close(H, Segment(1, 0, 3, 1), max_angle_deg=10)


# ------------------------------------------------------------------ fusion
def test_merge_extends_to_extreme_points():
    m = merge(H, Segment(3, 0, 6, 0))
    xs = sorted([m.x1, m.x2])
    assert xs == pytest.approx([0, 6])
    assert [m.y1, m.y2] == pytest.approx([0, 0])


def test_merge_handles_opposite_directions():
    m = merge(Segment(0, 0, 2, 0), Segment(4, 0, 2, 0))
    assert m.length == pytest.approx(4)
    assert angle_between(m, H) == pytest.approx(0, abs=1e-9)


def test_merge_direction_weighted_by_length():
    m = merge(Segment(0, 0, 10, 0), Segment(0, 0, 1, 0.1))
    assert angle_between(m, H) < 1.0


# ------------------------------------------------------------------ intersection
def test_intersection_cross():
    p = intersection(H, Segment(1, -1, 1, 1))
    assert p == pytest.approx([1, 0])


def test_intersection_at_endpoint_is_found():
    assert intersection(H, Segment(4, -1, 4, 1)) == pytest.approx([4, 0])


def test_intersection_none_when_lines_cross_outside():
    assert intersection(H, Segment(5, -1, 5, 1)) is None


def test_intersection_none_when_parallel():
    assert intersection(H, Segment(0, 1, 4, 1)) is None


def test_intersection_none_when_collinear_overlap():
    assert intersection(H, Segment(1, 0, 3, 0)) is None


def test_intersection_within_tolerance():
    # le coupeur s'arrête à EPS/2 du segment : considéré comme touchant
    assert intersection(H, Segment(2, -1, 2, -EPS / 2)) is not None
    assert intersection(H, Segment(2, -1, 2, -10 * EPS)) is None


# ------------------------------------------------------------------ découpe
def test_cut_keeps_longest_side():
    out = cut(H, Segment(1, -1, 1, 1))     # morceaux de 1 m et 3 m
    assert out.as_tuple() == pytest.approx((1, 0, 4, 0))


def test_cut_keeps_first_side_when_it_is_longest():
    out = cut(H, Segment(3, -1, 3, 1))
    assert out.as_tuple() == pytest.approx((0, 0, 3, 0))


def test_cut_equal_lengths_keeps_side_with_first_point():
    out = cut(H, Segment(2, -1, 2, 1))
    assert out.as_tuple() == pytest.approx((0, 0, 2, 0))


def test_cut_nearly_equal_lengths_within_tolerance_keeps_first_point():
    out = cut(H, Segment(2 + EPS / 4, -1, 2 + EPS / 4, 1))
    assert out.p1 == pytest.approx([0, 0])


def test_cut_at_endpoint_changes_nothing():
    assert cut(H, Segment(0, -1, 0, 1)) == H
    assert cut(H, Segment(4, -1, 4, 1)) == H
    assert cut(H, Segment(EPS / 2, -1, EPS / 2, 1)) == H   # à EPS près de l'extrémité


def test_cut_requires_intersection_on_cutter_too():
    # la droite du coupeur traverse H mais le coupeur s'arrête avant
    assert cut(H, Segment(2, 1, 2, 0.5)) == H


def test_cut_cutter_touching_with_its_endpoint_inside_target_cuts():
    # intersection sur les deux segments (extrémité du coupeur) et strictement dans target
    out = cut(H, Segment(1, 1, 1, 0))
    assert out.as_tuple() == pytest.approx((1, 0, 4, 0))


def test_cut_collinear_does_nothing():
    assert cut(H, Segment(1, 0, 3, 0)) == H
    assert cut(H, Segment(-1, 0, 5, 0)) == H


def test_cut_parallel_does_nothing():
    assert cut(H, Segment(0, 1, 4, 1)) == H


def test_cut_does_not_mutate_inputs():
    target, cutter = Segment(0, 0, 4, 0), Segment(1, -1, 1, 1)
    cut(target, cutter)
    assert target == Segment(0, 0, 4, 0) and cutter == Segment(1, -1, 1, 1)


def test_point_segment_distance():
    assert point_segment_distance(np.array([2, 1]), H) == pytest.approx(1)
    assert point_segment_distance(np.array([5, 0]), H) == pytest.approx(1)
