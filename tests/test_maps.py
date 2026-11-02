import numpy as np
import pytest

from lidar.acquire import read_scans
from lidar.extraction import points_to_segments, scan_to_segments
from lidar.geometry import Pose, local_to_global, polar_to_cartesian
from lidar.gridmap import bresenham, create_grid, grid_from_scan, update_grid
from lidar.scan import Scan, load_scan, save_scan
from lidar.segments import angle_between, Segment
from lidar.simulate import simulate_scan
from lidar.vectormap import build_vector_map


# ------------------------------------------------------------------ géométrie / IO
def test_polar_to_cartesian_counter_clockwise():
    pts = polar_to_cartesian([0, 90], [1, 2])
    assert pts == pytest.approx(np.array([[1, 0], [0, 2]]), abs=1e-12)


def test_local_to_global_rotation_then_translation():
    out = local_to_global(np.array([[1.0, 0.0]]), Pose(2, 3, 90))
    assert out[0] == pytest.approx([2, 4])


def test_save_and_load_roundtrip(tmp_path):
    scan = simulate_scan(Pose(1, 1, 30))
    loaded = load_scan(save_scan(scan, tmp_path / "s.csv"))
    assert loaded.pose == Pose(1, 1, 30)
    assert loaded.ranges_m == pytest.approx(scan.ranges_m, abs=1e-4)


def test_load_ld06_mm_format(tmp_path):
    f = tmp_path / "ld.csv"
    f.write_text("N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)\n0,10.5,1500,200,0,0\n1,11.0,0,0,0,0\n",
                 encoding="utf-8")
    s = load_scan(f)
    assert s.ranges_m == pytest.approx([1.5, 0.0])
    assert len(s.filtered()) == 1  # distance 0 = pas d'impact


def test_read_scans_splits_on_blank_line():
    lines = [f"{k},{k * 0.8},1000,200\n" for k in range(150)] + ["\n"] + ["0,1,1000,200\n", "\n"]
    scans = list(read_scans(lines, count=5, pose=Pose()))
    assert len(scans) == 1 and len(scans[0]) == 150   # le 2e scan (1 point) est rejeté


# ------------------------------------------------------------------ grid map
def test_bresenham_endpoints_and_continuity():
    cells = bresenham(0, 0, 5, 2)
    assert cells[0] == (0, 0) and cells[-1] == (5, 2)
    steps = np.abs(np.diff(np.array(cells), axis=0))
    assert steps.max() <= 1


def test_create_grid_dimensions():
    g = create_grid(2.0, 3.0, 0.1)
    assert g.shape == (30, 20)
    assert np.all(g.probability() == 0.5)


def test_update_marks_free_ray_and_occupied_impact():
    g = create_grid(3, 1, 0.1, origin=(0, 0))
    scan = Scan(np.array([0.0]), np.array([2.05]), np.array([200.0]), Pose(0.05, 0.55, 0))
    g2 = update_grid(g, scan)
    p = g2.probability()
    assert p[5, 20] > 0.5            # cellule d'impact (x=2.05 -> i=20, y=0.55 -> j=5)
    assert np.all(p[5, 1:20] < 0.5)  # cellules traversées
    assert p[5, 25] == 0.5           # au-delà : inconnu
    assert np.all(g.logodds == 0)    # grille d'origine non modifiée


def test_repeated_updates_increase_confidence():
    scan = simulate_scan(Pose(1, 1, 0))
    g1 = grid_from_scan(scan)
    g2 = update_grid(g1, scan)
    assert g2.probability().max() > g1.probability().max()


def test_simulated_room_walls_are_occupied():
    g = grid_from_scan(simulate_scan(Pose(1, 1, 0), noise_m=0.0))
    i, j = g.world_to_cell(np.array([4.0 - 0.01]), np.array([1.5]))  # mur x = 4
    assert g.occupied_mask()[j[0], i[0]]
    i, j = g.world_to_cell(np.array([2.0]), np.array([1.0]))         # milieu de la pièce
    assert g.free_mask()[j[0], i[0]]


# ------------------------------------------------------------------ segments / carte vectorielle
def test_points_on_a_line_give_one_segment():
    pts = np.column_stack((np.linspace(0, 2, 50), np.full(50, 1.0)))
    segs = points_to_segments(pts)
    assert len(segs) == 1 and segs[0].length == pytest.approx(2, abs=0.01)


def test_l_shape_gives_two_perpendicular_segments():
    a = np.column_stack((np.linspace(0, 2, 80), np.zeros(80)))       # pas de 2,5 cm < seuil de saut
    b = np.column_stack((np.full(80, 2.0), np.linspace(0.025, 2, 80)))
    segs = points_to_segments(np.vstack((a, b)))
    assert len(segs) == 2
    assert angle_between(*segs) == pytest.approx(90, abs=2)


def test_room_extraction_finds_walls():
    segs = scan_to_segments(simulate_scan(Pose(1.0, 1.0, 0), noise_m=0.005))
    long_segs = [s for s in segs if s.length > 1.0]
    assert len(long_segs) >= 4  # 4 murs (certains peuvent être coupés par l'ombre de la caisse)


def test_vector_map_merges_scans_of_the_same_wall():
    scans = [simulate_scan(Pose(*p), noise_m=0.005, seed=k)
             for k, p in enumerate([(1, 1, 0), (3, 1, 90), (1, 2.5, 180)])]
    vmap = build_vector_map(scans)
    bottom = [f for f in vmap.features if f.segment.length > 3.0
              and abs(f.segment.y1) < 0.05 and abs(f.segment.y2) < 0.05]
    assert len(bottom) == 1 and bottom[0].confidence >= 2


def test_vector_map_removes_wall_that_disappeared():
    # le scan 1 voit un mur en y = 2 ; puis le mur disparaît (porte ouverte) et le robot voit à travers
    wall = [(0.0, 2.0, 4.0, 2.0), (0.0, 4.0, 4.0, 4.0)]
    s1 = simulate_scan(Pose(2, 1, 0), walls=wall, noise_m=0.0)
    s2 = simulate_scan(Pose(2, 1, 0), walls=[(0.0, 4.0, 4.0, 4.0), (1.5, 2.0, 2.5, 2.0)], noise_m=0.0)
    vmap = build_vector_map([s1])
    assert any(abs(f.segment.y1 - 2) < 0.05 and f.segment.length > 3 for f in vmap.features)
    from lidar.vectormap import update_vector_map
    vmap2 = update_vector_map(vmap, s2)
    at_y2 = [f.segment for f in vmap2.features if abs(f.segment.y1 - 2) < 0.05]
    assert all(s.length < 3.5 for s in at_y2)  # le mur traversé a été raccourci
    assert isinstance(at_y2[0], Segment)
