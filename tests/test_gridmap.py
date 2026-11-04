"""Grid map probabiliste : comment plusieurs scans se combinent (question du prof, Jour 3).

Chaque scan met à jour les cellules qu'il observe : traversées -> L += L_FREE (< 0), impact -> L += L_OCC (> 0).
"""

import numpy as np

from lidar.geometry import Pose
from lidar.gridmap import create_grid, update_grid
from lidar.simulate import simulate_scan

ROOM = [(0, 0, 4, 0), (4, 0, 4, 3), (4, 3, 0, 3), (0, 3, 0, 0)]
BOX = [(2.0, 1.2, 2.4, 1.2), (2.4, 1.2, 2.4, 1.6), (2.4, 1.6, 2.0, 1.6), (2.0, 1.6, 2.0, 1.2)]
POSES = [Pose(1.0, 0.8, 0), Pose(3.2, 0.8, 90), Pose(3.0, 2.3, 180), Pose(1.0, 2.2, -90)]


def _grid():
    return create_grid(4.6, 3.6, 0.05, origin=(-0.3, -0.3))


def _p(grid, x, y):
    """Probabilité d'occupation la plus forte autour de (x, y) (± 1 case : un mur peut tomber sur une frontière)."""
    i, j = grid.world_to_cell(x, y)
    return float(grid.probability()[j - 1:j + 2, i - 1:i + 2].max())


def test_a_wall_seen_by_more_scans_is_more_certain():
    one = update_grid(_grid(), simulate_scan(POSES[0], walls=ROOM, noise_m=0.0))
    many = _grid()
    for p in POSES:
        many = update_grid(many, simulate_scan(p, walls=ROOM, noise_m=0.0))
    wall = (2.0, 0.0)                                        # le mur du bas, vu par les 4 scans
    assert 0.5 < _p(one, *wall) < _p(many, *wall)


def test_scan_by_scan_or_all_at_the_end_gives_the_same_map():
    """Mise à jour après chaque scan, ou tous les scans à la fin : même carte (le log-odds s'additionne)."""
    scans = [simulate_scan(p, walls=ROOM + BOX, noise_m=0.01, seed=k) for k, p in enumerate(POSES)]
    step_by_step = _grid()
    for s in scans:                                           # comme pendant le SLAM : un scan après l'autre
        step_by_step = update_grid(step_by_step, s)
    other_order = _grid()
    for s in reversed(scans):
        other_order = update_grid(other_order, s)
    assert np.allclose(step_by_step.logodds, other_order.logodds)


def test_an_object_removed_later_fades_away():
    """Le 1er scan voit une boîte ; elle est enlevée ; les faisceaux suivants la TRAVERSENT : elle s'efface."""
    grid = update_grid(_grid(), simulate_scan(POSES[0], walls=ROOM + BOX, noise_m=0.0))
    box = (2.0, 1.4)                                          # face de la boîte vue par le 1er scan
    assert _p(grid, *box) > 0.5
    for k in range(3):                                        # 3 scans de plus, boîte enlevée
        grid = update_grid(grid, simulate_scan(Pose(0.8 + 0.1 * k, 1.4, 0), walls=ROOM, noise_m=0.0))
    assert _p(grid, *box) < 0.5
