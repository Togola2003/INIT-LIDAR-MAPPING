"""Robot en Wi-Fi : lire ses scans et le piloter, avec le firmware RobotControl du module.

Le robot (ESP32) crée son propre réseau Wi-Fi « RobotESP32 » (mot de passe 12345678) et répond en HTTP
sur http://192.168.4.1 :
- GET /scan : le dernier tour du lidar, en binaire little-endian :
    octets 0-5  pose mémorisée (x mm int16, y mm int16, θ centièmes de degré uint16)
    octets 6-7  nombre de points N (uint16)
    puis N × 5 octets : angle (centièmes de degré, uint16), distance (mm, uint16), intensité (uint8)
  Les angles sont déjà dans le sens trigonométrique (comme printScanCSV).
- GET /forward, /backward, /left, /right, /rotateleft, /rotateright ?time=ms : un mouvement de time ms
  (2 s au plus ; le firmware divise le temps des rotations par deux) ;
- GET /stop : arrêt ; GET /pose?x=mm&y=mm&z=cdeg : mémoriser une pose ; GET /ping.
Le robot n'a pas de codeurs : il ne connaît pas sa position. C'est la localisation qui la trouve.
"""

from __future__ import annotations

import queue
import struct
import threading
import time
import urllib.error
import urllib.request

import numpy as np

from .geometry import Pose
from .scan import Scan
from .scope.sources import Event, Frame

DEFAULT_HOST = "192.168.4.1"
FORMAT_WIFI = "wifi (RobotControl)"
MOVES = ("forward", "backward", "left", "right", "rotateleft", "rotateright")
MAX_MOVE_MS = 2000
POLL_S = 0.2          # 5 scans/s au plus, comme RobotControl


def decode_scan(buf: bytes) -> tuple[Pose, Scan]:
    """Décode la réponse binaire de /scan. Lève ValueError si elle est tronquée ou incohérente."""
    if len(buf) < 8:
        raise ValueError("en-tête de scan incomplet")
    x, y, z, n = struct.unpack_from("<hhHH", buf, 0)
    if n == 0 or len(buf) != 8 + 5 * n:
        raise ValueError(f"taille de scan invalide ({len(buf)} octets pour {n} points)")
    rows = np.frombuffer(buf, dtype=np.dtype([("a", "<u2"), ("d", "<u2"), ("i", "u1")]), count=n, offset=8)
    scan = Scan(rows["a"] / 100.0, rows["d"] / 1000.0, rows["i"].astype(float), Pose(), name="wifi")
    return Pose(x / 1000.0, y / 1000.0, z / 100.0), scan


class RobotClient:
    """Commandes de déplacement, envoyées en arrière-plan : l'interface ne gèle jamais, même sans réseau.
    Une seule commande ordinaire à la fois ; /stop passe toujours."""

    def __init__(self, host: str = DEFAULT_HOST, timeout_s: float = 1.0) -> None:
        self.base = f"http://{host}"
        self.timeout = timeout_s
        self.busy = False
        self.last_error: str | None = None

    def _get(self, path: str) -> bytes:
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
            return r.read()

    def _send(self, path: str, ordinary: bool) -> None:
        try:
            self._get(path)
            self.last_error = None
        except (urllib.error.URLError, OSError) as exc:
            self.last_error = str(exc)
        finally:
            if ordinary:
                self.busy = False

    def move(self, action: str, time_ms: int = 300) -> bool:
        if action not in MOVES:
            raise ValueError(f"mouvement inconnu : {action}")
        if self.busy:
            return False
        self.busy = True
        t = int(max(1, min(MAX_MOVE_MS, time_ms)))
        threading.Thread(target=self._send, args=(f"/{action}?time={t}", True), daemon=True).start()
        return True

    def stop(self) -> None:
        threading.Thread(target=self._send, args=("/stop", False), daemon=True).start()

    def set_pose(self, pose: Pose) -> None:
        z = int(round(pose.theta_deg * 100)) % 36000
        path = f"/pose?x={int(round(pose.x * 1000))}&y={int(round(pose.y * 1000))}&z={z}"
        threading.Thread(target=self._send, args=(path, False), daemon=True).start()


class WifiSource(threading.Thread):
    """Lit /scan en boucle et publie chaque nouveau tour comme les autres sources (Event « scan »)."""

    def __init__(self, out: queue.Queue, host: str = DEFAULT_HOST) -> None:
        super().__init__(daemon=True)
        self.out, self.host = out, host
        self.client = RobotClient(host)
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        self.out.put(Event("status", f"Wi-Fi : connexion à {self.host}…"))
        failures, last = 0, None
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                buf = self.client._get("/scan")
                if buf != last:                       # le robot renvoie le même tour tant qu'il n'en a pas de nouveau
                    _, scan = decode_scan(buf)
                    self.out.put(Event("scan", Frame(scan, FORMAT_WIFI, (f"{len(buf)} octets binaires /scan",))))
                    last = buf
                if failures:
                    self.out.put(Event("status", f"Wi-Fi : robot {self.host} joignable"))
                failures = 0
            except ValueError:
                pass                                  # réponse 503 « pas encore de scan » ou trame tronquée : on réessaie
            except (urllib.error.URLError, OSError) as exc:
                failures += 1
                if failures == 1:
                    self.out.put(Event("status", f"Wi-Fi : {self.host} injoignable ({exc}). Es-tu connecté au "
                                                 "réseau RobotESP32 ?"))
                if failures >= 15:
                    self.out.put(Event("error", f"Robot {self.host} injoignable depuis 3 s. Connecte le PC au Wi-Fi "
                                                "« RobotESP32 » (mot de passe 12345678)."))
                    break
            self._stop.wait(max(0.0, POLL_S - (time.monotonic() - t0)))
        self.out.put(Event("status", "Wi-Fi déconnecté"))
