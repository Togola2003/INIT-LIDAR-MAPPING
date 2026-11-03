"""Sources de données pour LidarScope : port série réel ou démo simulée.

Formats reconnus ligne par ligne (les deux sorties possibles de la bibliothèque LD06) :
- printScanCSV    : ``N,angle_deg,distance_mm,intensité[,x,y]`` ; une ligne vide termine le scan ;
- printScanTeleplot : ``>lidar:x:y;x:y;...|xy`` (x, y en mm) ; une ligne = un scan.
  ⚠ la bibliothèque calcule y = -d·sin(angle) (repère « miroir ») : on le corrige ici.
Les autres lignes Teleplot ``>nom:valeur`` sont gardées comme télémétrie (ex. vitesse).
"""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass

import numpy as np

from ..geometry import Pose
from ..scan import Scan

MIN_POINTS = 100   # un tour LD06 ≈ 450 points : en dessous, le scan est considéré tronqué


FORMAT_CSV = "printScanCSV"
FORMAT_TELEPLOT = "printScanTeleplot"


@dataclass(frozen=True)
class Event:
    kind: str           # "scan", "telemetry", "status", "error"
    payload: object


@dataclass(frozen=True)
class Frame:
    """Un tour complet reçu, avec ce qu'il faut pour le relire de bout en bout."""
    scan: Scan
    fmt: str                  # FORMAT_CSV ou FORMAT_TELEPLOT
    raw: tuple[str, ...]      # lignes exactement telles que reçues du port série


def _fill_angles(angles: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Teleplot ne donne pas d'angle aux points sans écho (x = y = 0) : on l'interpole
    entre les voisins (les points arrivent dans l'ordre de rotation)."""
    if valid.sum() < 2:
        return angles
    idx = np.arange(len(angles))
    unwrapped = np.degrees(np.unwrap(np.radians(angles[valid])))
    return np.interp(idx, idx[valid], unwrapped) % 360.0


class LineParser:
    """Transforme des lignes de texte en tours complets. Garde l'état entre deux lignes."""

    def __init__(self) -> None:
        self._points: list[tuple[float, float, float]] = []
        self._raw: list[str] = []

    def feed(self, line: str) -> list[Event]:
        line = line.strip()
        if line.startswith(">"):
            return self._teleplot(line)
        if not line:
            return self._end_of_csv_scan()
        parts = line.split(",")
        if len(parts) >= 4:
            try:
                _, angle, dist_mm, inten = (float(p) for p in parts[:4])
            except ValueError:
                return []   # en-tête « N,Angle(°),... » ou ligne abîmée
            self._points.append((angle, dist_mm / 1000.0, inten))
            self._raw.append(line)
        return []

    def _end_of_csv_scan(self) -> list[Event]:
        pts, raw = self._points, self._raw
        self._points, self._raw = [], []
        if len(pts) < MIN_POINTS:
            return []
        arr = np.array(pts)
        scan = Scan(arr[:, 0], arr[:, 1], arr[:, 2], name="live")
        return [Event("scan", Frame(scan, FORMAT_CSV, tuple(raw)))]

    def _teleplot(self, line: str) -> list[Event]:
        name, _, rest = line[1:].partition(":")
        if rest.endswith("|xy"):
            return self._teleplot_xy(rest[:-3], line)
        try:
            return [Event("telemetry", (name, float(rest.split("|")[0])))]
        except ValueError:
            return []

    @staticmethod
    def _teleplot_xy(data: str, line: str) -> list[Event]:
        xs, ys = [], []
        for pair in data.split(";"):
            try:
                x, y = (float(v) for v in pair.split(":"))
            except ValueError:
                continue
            xs.append(x)
            ys.append(y)
        if len(xs) < MIN_POINTS:
            return []
        x, y = np.array(xs), np.array(ys)
        ranges = np.hypot(x, y) / 1000.0
        angles = np.degrees(np.arctan2(-y, x)) % 360.0     # -y : correction du miroir de la bibliothèque
        angles = _fill_angles(angles, ranges > 0)
        scan = Scan(angles, ranges, np.full(len(x), np.nan), name="live")   # intensité : non transmise
        return [Event("scan", Frame(scan, FORMAT_TELEPLOT, (line,)))]


class SerialSource(threading.Thread):
    """Lit le port série dans un fil d'exécution séparé (l'interface ne se fige jamais)."""

    def __init__(self, port: str, baud: int, out: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.port, self.baud, self.out = port, baud, out
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        import serial

        parser = LineParser()
        try:
            with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                ser.reset_input_buffer()
                self.out.put(Event("status", f"Connecté à {self.port} ({self.baud} bauds)"))
                while not self._stop.is_set():
                    raw = ser.readline()
                    if raw:
                        for ev in parser.feed(raw.decode("utf-8", errors="replace")):
                            self.out.put(ev)
        except Exception as exc:   # port absent, déjà ouvert, câble débranché...
            self.out.put(Event("error", f"{self.port} : {exc}"))
        self.out.put(Event("status", "Déconnecté"))


class DemoSource(threading.Thread):
    """Démo sans robot : la pièce simulée, envoyée au format printScanCSV à ~5 scans/s."""

    def __init__(self, out: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.out = out
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        from ..simulate import simulate_scan

        parser = LineParser()
        self.out.put(Event("status", "Démo : pièce simulée 4 m × 3 m"))
        k = 0
        while not self._stop.is_set():
            # le robot « tourne » lentement sur place pour que l'image bouge
            scan = simulate_scan(Pose(1.6, 1.2, 15 * math.sin(k / 10)), seed=k)
            for n, (a, r, i) in enumerate(zip(scan.angles_deg, scan.ranges_m, scan.intensities)):
                for ev in parser.feed(f"{n},{a:.2f},{r * 1000:.0f},{i:.0f}"):
                    self.out.put(ev)
            for ev in parser.feed(""):
                self.out.put(ev)
            self.out.put(Event("telemetry", ("vitesse_demo", 3600.0)))
            k += 1
            time.sleep(0.2)
        self.out.put(Event("status", "Démo arrêtée"))


class MapDemoSource(threading.Thread):
    """Démo de la localisation en temps réel : un robot simulé roule dans UNE CARTE DONNÉE (les murs de
    la carte de référence) en passant par des points de passage, et envoie ses scans comme le vrai
    robot (printScanCSV, ~4 scans/s). Sa vraie pose est lisible dans `truth` pour mesurer l'erreur."""

    def __init__(self, out: queue.Queue, walls, waypoints, speed_m_s: float = 0.25, rate_hz: float = 4.0) -> None:
        super().__init__(daemon=True)
        self.out, self.walls, self.waypoints = out, list(walls), list(waypoints)
        self.speed, self.period = speed_m_s, 1.0 / rate_hz
        self.truth: Pose | None = None
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def _route(self):
        """Poses successives le long des points de passage (aller-retour), orientées dans le sens de la marche."""
        pts = self.waypoints + self.waypoints[-2:0:-1]
        step = self.speed * self.period
        while True:
            for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
                d = math.hypot(x1 - x0, y1 - y0)
                th = math.degrees(math.atan2(y1 - y0, x1 - x0))
                for k in range(max(1, int(d / step))):
                    t = k * step / max(d, 1e-9)
                    yield Pose(x0 + t * (x1 - x0), y0 + t * (y1 - y0), th)

    def run(self) -> None:
        from ..simulate import simulate_scan

        parser = LineParser()
        self.out.put(Event("status", "Démo sur la carte : robot simulé dans la salle validée"))
        for k, pose in enumerate(self._route()):
            if self._stop.is_set():
                break
            scan = simulate_scan(pose, walls=self.walls, noise_m=0.01, seed=k)
            self.truth = pose
            for n, (a, r, i) in enumerate(zip(scan.angles_deg, scan.ranges_m, scan.intensities)):
                for ev in parser.feed(f"{n},{a:.2f},{r * 1000:.0f},{i:.0f}"):
                    self.out.put(ev)
            for ev in parser.feed(""):
                self.out.put(ev)
            time.sleep(self.period)
        self.out.put(Event("status", "Démo arrêtée"))


def list_ports() -> list[str]:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    return [p.device for p in lp.comports()]
