import json

import numpy as np
import pytest

from lidar.geometry import Pose
from lidar.gridmap import create_grid, remove_isolated, save_grid_csv, update_grid
from lidar.scan import Scan
from lidar.scope.exporter import ExportOptions, export_bundle
from lidar.scope.mapping import GridParams, MapParams, build_maps
from lidar.scope.session import Capture, load_session, read_any
from lidar.scope.sources import LineParser
from lidar.simulate import simulate_scan


@pytest.fixture
def captures():
    return [Capture(k + 1, simulate_scan(Pose(*p), seed=k), c, source="printScanCSV")
            for k, (p, c) in enumerate([((0.8, 0.8, 0), "#2ecc71"), ((2.8, 1.2, 90), "#e67e22")])]


def test_export_each_and_all(tmp_path, captures):
    root = export_bundle(tmp_path, captures, None, ExportOptions(), MapParams())
    dirs = sorted(p.name for p in root.iterdir() if p.is_dir())
    assert dirs == ["ensemble", "scan_01_scan_1", "scan_02_scan_2"]
    ens = root / "ensemble"
    for name in ("session.lidar.json", "points.csv", "carte.png", "grid_map_maptool.csv",
                 "grid_map_ternaire.csv", "grid_map_proba.csv", "carte_vectorielle.txt"):
        assert (ens / name).exists(), name
    assert len(list((ens / "scans_tp").glob("*.csv"))) == 2
    report = (root / "LISEZMOI.md").read_text(encoding="utf-8")
    assert "Conventions" in report and "Grid map" in report
    # la session de l'ensemble se relit et contient les résultats de cartographie
    data = json.loads((ens / "session.lidar.json").read_text(encoding="utf-8"))
    assert data["processing"]["map_stats"]["vector"]["segments"] >= 4
    assert len(load_session(ens / "session.lidar.json")) == 2


def test_export_selection_only(tmp_path, captures):
    root = export_bundle(tmp_path, captures, 2, ExportOptions(scope="selection", items=frozenset({"session"})),
                         MapParams())
    assert [p.name for p in root.iterdir() if p.is_dir()] == ["scan_02_scan_2"]


def test_maptool_grid_csv_layout(tmp_path):
    g = create_grid(0.3, 0.2, 0.1)
    g = update_grid(g, Scan(np.array([0.0]), np.array([0.2]), np.array([200.0]), Pose(0.05, 0.15, 0)))
    lines = save_grid_csv(g, tmp_path / "g.csv", "maptool").read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("# Grid map 3x2")
    assert lines[1] == "0,0,1"      # 1re ligne = HAUT de la carte (y = 0,1..0,2) : impact en x = 0,25
    assert lines[2] == "0,0,0"
    tern = save_grid_csv(g, tmp_path / "t.csv", "ternaire").read_text(encoding="utf-8").splitlines()
    assert tern[1] == "0,0,1" and tern[2] == "-1,-1,-1"


def test_remove_isolated_keeps_walls():
    scan = simulate_scan(Pose(1, 1, 0), noise_m=0.0)
    res = build_maps([scan], MapParams(grid=GridParams(remove_isolated=False)))
    i, j = res.grid.world_to_cell(2.0, 1.5)                    # milieu de la pièce, loin des murs
    noisy = res.grid.logodds.copy()
    noisy[j, i] = 3.0                                          # une cellule occupée isolée dans le vide
    g = remove_isolated(type(res.grid)(noisy, res.grid.resolution, res.grid.origin))
    assert g.logodds[j, i] == 0.0
    assert g.occupied_mask().sum() >= res.grid.occupied_mask().sum() * 0.95


def test_initial_state_and_binary_model():
    scan = simulate_scan(Pose(1, 1, 0))
    res = build_maps([scan], MapParams(grid=GridParams(initial_state="occupé", model="binaire")))
    img = res.grid_image("binaire")
    assert set(np.unique(img)) <= {0.0, 0.5, 1.0}
    assert res.stats["grid"]["unknown"] == 0      # pessimiste : rien d'inconnu, le non-vu reste « occupé »


def test_teleplot_capture_roundtrip_keeps_unknown_intensity_and_raw(tmp_path):
    pts = ";".join(f"{1000 * np.cos(np.radians(a)):.1f}:{-1000 * np.sin(np.radians(a)):.1f}" for a in range(0, 360, 2))
    line = f">lidar:{pts}|xy"
    (ev,) = LineParser().feed(line)
    cap = Capture(1, ev.payload.scan, "#2ecc71", source=ev.payload.fmt, raw_tours=(ev.payload.raw,))
    root = export_bundle(tmp_path, [cap], None, ExportOptions(scope="ensemble"), MapParams())
    data = json.loads((root / "ensemble" / "session.lidar.json").read_text(encoding="utf-8"))
    c = data["captures"][0]
    assert c["source"]["firmware_output"] == "printScanTeleplot"
    assert c["source"]["raw_tours"] == [[line]]
    assert all(r[c["columns"].index("intensity")] is None for r in c["points"])
    (_, back), = read_any(root / "ensemble" / "points.csv")
    assert np.isnan(back.intensities).all() and len(back) == 180
