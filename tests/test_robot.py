import queue
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import pytest

from lidar.robot import RobotClient, WifiSource, decode_scan


def _frame(points, pose=(1500, -250, 9000)):
    buf = struct.pack("<hhHH", *pose, len(points))
    for a, d, i in points:
        buf += struct.pack("<HHB", a, d, i)
    return buf


def test_decode_scan_units_and_pose():
    pose, scan = decode_scan(_frame([(0, 1000, 200), (9000, 2500, 50), (18050, 0, 0)]))
    assert (pose.x, pose.y, pose.theta_deg) == (1.5, -0.25, 90.0)
    assert np.allclose(scan.angles_deg, [0, 90, 180.5]) and np.allclose(scan.ranges_m, [1.0, 2.5, 0.0])
    assert np.allclose(scan.local_points()[1], [0, 2.5], atol=1e-9)


@pytest.mark.parametrize("buf", [b"", b"\x00" * 7, _frame([(0, 1, 1)])[:-1], struct.pack("<hhHH", 0, 0, 0, 0)])
def test_decode_scan_rejects_bad_frames(buf):
    with pytest.raises(ValueError):
        decode_scan(buf)


class _FakeRobot(BaseHTTPRequestHandler):
    calls: list = []

    def do_GET(self):
        _FakeRobot.calls.append(self.path)
        body = _frame([(k * 100, 1000 + k, 100) for k in range(360)]) if self.path == "/scan" else b"OK"
        self.send_response(200)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


def test_wifi_source_and_commands_against_a_fake_robot():
    server = HTTPServer(("127.0.0.1", 0), _FakeRobot)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host = f"127.0.0.1:{server.server_address[1]}"
    q = queue.Queue()
    src = WifiSource(q, host)
    src.start()
    client = RobotClient(host)
    assert client.move("forward", 5000)
    with pytest.raises(ValueError):
        client.move("jump")
    client.stop()
    time.sleep(0.6)
    src.stop()
    src.join(2)
    server.shutdown()
    scans = [e.payload for e in list(q.queue) if e.kind == "scan"]
    assert len(scans) == 1 and len(scans[0].scan) == 360        # même tour renvoyé : publié une seule fois
    assert "/forward?time=2000" in _FakeRobot.calls and "/stop" in _FakeRobot.calls


def test_speed_command_against_a_fake_robot():
    server = HTTPServer(("127.0.0.1", 0), _FakeRobot)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    client = RobotClient(f"127.0.0.1:{server.server_address[1]}")
    client.set_speed(999)
    time.sleep(0.3)
    server.shutdown()
    assert "/speed?v=255" in _FakeRobot.calls


def test_command_motion_and_odometry():
    from lidar.robot import Odometry, command_motion
    dx, dy, dth = command_motion("forward", 1000, 150)
    assert dx == pytest.approx(0.30) and dy == 0 and dth == 0
    assert command_motion("rotateleft", 500, 150)[2] == pytest.approx(45.0)
    assert command_motion("right", 1000, 150)[1] < 0
    assert command_motion("forward", 9999, 150)[0] == pytest.approx(0.60)     # 2 s maximum
    odo = Odometry()
    odo.add("forward", 1000, 100)
    odo.add("rotateright", 1000, 100)
    assert odo.take() == pytest.approx((0.2, 0.0, -60.0))
    assert odo.take() == (0.0, 0.0, 0.0)
