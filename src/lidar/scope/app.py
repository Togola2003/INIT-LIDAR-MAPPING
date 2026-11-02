"""LidarScope : fenêtre de visualisation en direct des scans du LD06.

    .venv/Scripts/python -m lidar.scope

Choisir le port série, Connecter (ou « Démo » sans robot), regarder le scan en direct au centre.
Quand l'image plaît : renseigner la pose du robot puis « Capturer » (touche C). Chaque capture
reçoit sa couleur. Exports : image PNG, tableau CSV de tous les points, fichiers au format du TP.
"""

from __future__ import annotations

import collections
import csv
import queue
import time
import tkinter as tk
from dataclasses import dataclass, replace
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from ..geometry import Pose
from ..scan import Scan, load_scan, save_scan
from .sources import DemoSource, SerialSource, list_ports

CAPTURE_COLORS = ["#2ecc71", "#e67e22", "#3498db", "#e84393", "#f1c40f", "#9b59b6", "#1abc9c", "#e74c3c"]
LIVE_COLOR = "#dfe6e9"
BG, PANEL, INK, MUTED, ACCENT = "#14171c", "#1d2128", "#e8eaed", "#8b93a1", "#2ecc71"
REFRESH_MS = 100
BAUDS = ["115200", "230400", "460800", "921600"]


@dataclass(frozen=True)
class Capture:
    number: int
    scan: Scan
    color: str
    visible: bool = True


class LidarScope:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.events: queue.Queue = queue.Queue()
        self.source = None
        self.live: Scan | None = None
        self.paused = False
        self.captures: list[Capture] = []
        self.artists: dict[int, object] = {}
        self.scan_times: collections.deque = collections.deque(maxlen=20)
        self.scan_count = 0
        self.telemetry: dict[str, float] = {}
        self._needs_recenter = True
        self.analysis_artists: list = []

        root.title("LidarScope · LD06")
        root.geometry("1400x860")
        root.configure(bg=BG)
        self._style()
        self._build_topbar()
        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True)
        self._build_sidebar(body)
        self._build_map(body)
        self._build_statusbar()
        root.bind("<space>", lambda e: self.toggle_pause())
        root.bind("<KeyPress-c>", lambda e: self.capture())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.refresh_ports()
        root.after(REFRESH_MS, self._tick)

    # ------------------------------------------------------------------ construction
    def _style(self) -> None:
        s = ttk.Style()
        s.theme_use("clam")
        s.configure(".", background=PANEL, foreground=INK, fieldbackground=BG, bordercolor="#2b313b")
        s.configure("TButton", padding=(10, 5), background="#2b313b", foreground=INK)
        s.map("TButton", background=[("active", "#3a414d")])
        s.configure("Accent.TButton", background=ACCENT, foreground="#0b0d10", font=("Segoe UI", 11, "bold"))
        s.map("Accent.TButton", background=[("active", "#58d68d")])
        s.configure("TLabelframe", background=PANEL)
        s.configure("TLabelframe.Label", background=PANEL, foreground=MUTED, font=("Segoe UI", 9, "bold"))
        s.configure("TLabel", background=PANEL)
        s.configure("Muted.TLabel", foreground=MUTED)
        s.configure("Big.TLabel", font=("Consolas", 11))
        s.configure("Treeview", background=BG, fieldbackground=BG, foreground=INK, rowheight=24)
        s.configure("TCheckbutton", background=PANEL)

    def _build_topbar(self) -> None:
        bar = tk.Frame(self.root, bg=PANEL, padx=10, pady=8)
        bar.pack(fill="x")
        tk.Label(bar, text="LidarScope", bg=PANEL, fg=ACCENT, font=("Segoe UI", 14, "bold")).pack(side="left")
        tk.Label(bar, text="  Port", bg=PANEL, fg=MUTED).pack(side="left", padx=(20, 4))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=12, state="readonly")
        self.port_box.pack(side="left")
        ttk.Button(bar, text="⟳", width=3, command=self.refresh_ports).pack(side="left", padx=4)
        tk.Label(bar, text="Débit", bg=PANEL, fg=MUTED).pack(side="left", padx=(12, 4))
        self.baud_var = tk.StringVar(value=BAUDS[0])
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUDS, width=8, state="readonly").pack(side="left")
        self.connect_btn = ttk.Button(bar, text="Connecter", style="Accent.TButton", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=10)
        ttk.Button(bar, text="Démo (sans robot)", command=self.start_demo).pack(side="left")
        self.led = tk.Label(bar, text="●", bg=PANEL, fg="#636e72", font=("Segoe UI", 14))
        self.led.pack(side="right")
        self.conn_label = tk.Label(bar, text="Déconnecté", bg=PANEL, fg=MUTED)
        self.conn_label.pack(side="right", padx=6)

    def _build_sidebar(self, parent: tk.Frame) -> None:
        side = ttk.Frame(parent, padding=10, width=330)
        side.pack(side="right", fill="y")
        side.pack_propagate(False)

        live = ttk.LabelFrame(side, text="SCAN EN DIRECT", padding=8)
        live.pack(fill="x")
        self.stats_var = tk.StringVar(value="En attente de données…")
        ttk.Label(live, textvariable=self.stats_var, style="Big.TLabel", justify="left").pack(anchor="w")
        row = ttk.Frame(live)
        row.pack(fill="x", pady=(8, 0))
        self.pause_btn = ttk.Button(row, text="⏸ Pause (Espace)", command=self.toggle_pause)
        self.pause_btn.pack(side="left")
        ttk.Button(row, text="Recentrer", command=self.recenter).pack(side="left", padx=6)
        row2 = ttk.Frame(live)
        row2.pack(fill="x", pady=(6, 0))
        ttk.Label(row2, text="Portée affichée (m)", style="Muted.TLabel").pack(side="left")
        self.range_var = tk.DoubleVar(value=5.0)
        ttk.Spinbox(row2, from_=1, to=12, increment=0.5, textvariable=self.range_var, width=5,
                    command=self.recenter).pack(side="left", padx=6)

        cap = ttk.LabelFrame(side, text="POSE DU ROBOT  (mètres, degrés)", padding=8)
        cap.pack(fill="x", pady=10)
        self.pose_vars = [tk.StringVar(value="0") for _ in range(3)]
        grid = ttk.Frame(cap)
        grid.pack(fill="x")
        for k, name in enumerate(("x", "y", "θ")):
            ttk.Label(grid, text=name, style="Muted.TLabel").grid(row=0, column=2 * k, padx=(0, 3))
            ttk.Entry(grid, textvariable=self.pose_vars[k], width=7).grid(row=0, column=2 * k + 1, padx=(0, 8))
        ttk.Label(cap, text="Mesurée au mètre depuis le coin de référence", style="Muted.TLabel").pack(anchor="w", pady=(4, 6))
        ttk.Button(cap, text="📸  Capturer ce scan  (C)", style="Accent.TButton", command=self.capture).pack(fill="x")

        lst = ttk.LabelFrame(side, text="CAPTURES", padding=8)
        lst.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(lst, columns=("pts", "pose"), show="tree headings", height=6, selectmode="browse")
        self.tree.heading("#0", text="Scan")
        self.tree.heading("pts", text="Points")
        self.tree.heading("pose", text="Pose (x, y, θ)")
        self.tree.column("#0", width=80)
        self.tree.column("pts", width=55, anchor="e")
        self.tree.column("pose", width=130)
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._on_select_capture)
        btns = ttk.Frame(lst)
        btns.pack(fill="x", pady=(6, 0))
        ttk.Button(btns, text="Afficher/masquer", command=self.toggle_visible).pack(side="left")
        ttk.Button(btns, text="Appliquer pose", command=self.apply_pose).pack(side="left", padx=4)
        ttk.Button(btns, text="Suppr.", command=self.delete_capture).pack(side="left")

        ana = ttk.LabelFrame(side, text="ANALYSE DES CAPTURES", padding=8)
        ana.pack(fill="x", pady=(10, 0))
        self.grid_on, self.seg_on = tk.BooleanVar(), tk.BooleanVar()
        ttk.Checkbutton(ana, text="Grid map (5 cm)", variable=self.grid_on, command=self.update_analysis).pack(anchor="w")
        ttk.Checkbutton(ana, text="Segments (carte vectorielle)", variable=self.seg_on,
                        command=self.update_analysis).pack(anchor="w")

        exp = ttk.LabelFrame(side, text="IMPORTER / EXPORTER", padding=8)
        exp.pack(fill="x", pady=(10, 0))
        ttk.Button(exp, text="Importer des fichiers CSV…", command=self.import_files).pack(fill="x", pady=(0, 4))
        ttk.Button(exp, text="Image de la carte (PNG)", command=self.export_png).pack(fill="x")
        ttk.Button(exp, text="Tableau complet des points (CSV)", command=self.export_table).pack(fill="x", pady=4)
        ttk.Button(exp, text="Fichiers pour le TP  →  data/scans", command=self.export_tp).pack(fill="x")

        tel = ttk.LabelFrame(side, text="TÉLÉMÉTRIE", padding=8)
        tel.pack(fill="x", pady=(10, 0))
        self.tel_var = tk.StringVar(value="—")
        ttk.Label(tel, textvariable=self.tel_var, style="Big.TLabel", justify="left").pack(anchor="w")

    def _build_map(self, parent: tk.Frame) -> None:
        frame = tk.Frame(parent, bg=BG)
        frame.pack(side="left", fill="both", expand=True)
        self.fig = Figure(facecolor=BG)
        self.ax = self.fig.add_subplot(111, facecolor="#0f1216")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, color="#2b313b", lw=0.6)
        self.ax.tick_params(colors=MUTED)
        for spine in self.ax.spines.values():
            spine.set_color("#2b313b")
        self.ax.set_xlabel("x (m)", color=MUTED)
        self.ax.set_ylabel("y (m)", color=MUTED)
        self.fig.subplots_adjust(left=0.06, right=0.98, top=0.97, bottom=0.07)
        self.live_art = self.ax.scatter([], [], s=5, c=LIVE_COLOR, label="direct", zorder=3)
        (self.robot_art,) = self.ax.plot([], [], "-", color="#ff4757", lw=2.5, zorder=4)
        (self.robot_dot,) = self.ax.plot([], [], "o", color="#ff4757", ms=8, zorder=4)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        toolbar = NavigationToolbar2Tk(self.canvas, frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side="bottom", fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_statusbar(self) -> None:
        self.status_var = tk.StringVar(value="Choisis un port et clique sur Connecter, ou essaie la Démo.")
        tk.Label(self.root, textvariable=self.status_var, bg=PANEL, fg=MUTED, anchor="w", padx=10).pack(fill="x", side="bottom")

    # ------------------------------------------------------------------ connexion
    def refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.status(f"{len(ports)} port(s) série trouvé(s)" if ports else "Aucun port série : branche l'ESP32")

    def toggle_connect(self) -> None:
        if self.source:
            self.disconnect()
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Port", "Choisis d'abord un port série (bouton ⟳ pour actualiser).")
            return
        self._start(SerialSource(port, int(self.baud_var.get()), self.events))

    def start_demo(self) -> None:
        self.disconnect()
        self._start(DemoSource(self.events))

    def _start(self, source) -> None:
        self.source = source
        self.scan_count = 0
        self._needs_recenter = True
        source.start()
        self.connect_btn.configure(text="Déconnecter")
        self.led.configure(fg="#f1c40f")

    def disconnect(self) -> None:
        if self.source:
            self.source.stop()
            self.source = None
        self.connect_btn.configure(text="Connecter")
        self.led.configure(fg="#636e72")

    def quit(self) -> None:
        self.disconnect()
        self.root.destroy()

    # ------------------------------------------------------------------ boucle d'affichage
    def _tick(self) -> None:
        latest = None
        while True:
            try:
                ev = self.events.get_nowait()
            except queue.Empty:
                break
            if ev.kind == "scan":
                latest = ev.payload
                self.scan_count += 1
                self.scan_times.append(time.monotonic())
            elif ev.kind == "telemetry":
                name, value = ev.payload
                self.telemetry[name] = value
            elif ev.kind == "status":
                self.conn_label.configure(text=ev.payload)
                self.status(ev.payload)
            elif ev.kind == "error":
                self.status(f"Erreur : {ev.payload}")
                self.led.configure(fg="#e74c3c")
                messagebox.showerror("Port série", f"{ev.payload}\n\nLe moniteur série Arduino est-il encore ouvert ?")
                self.disconnect()
        if latest is not None:
            self.led.configure(fg=ACCENT)
            if not self.paused:
                self.live = latest
                self._draw_live()
        self._update_texts()
        self.root.after(REFRESH_MS, self._tick)

    def _draw_live(self) -> None:
        if self.live is None:
            return
        pose = self.current_pose(quiet=True) or Pose()
        pts = self.live.filtered().global_points(pose)
        self.live_art.set_offsets(pts if len(pts) else np.empty((0, 2)))
        self.robot_dot.set_data([pose.x], [pose.y])
        self.robot_art.set_data([pose.x, pose.x + 0.3 * np.cos(pose.theta)],
                                [pose.y, pose.y + 0.3 * np.sin(pose.theta)])
        if self._needs_recenter:
            self.recenter()
            self._needs_recenter = False
        self.canvas.draw_idle()

    def _update_texts(self) -> None:
        if self.live is None:
            return
        s = self.live.filtered()
        freq = 0.0
        if len(self.scan_times) >= 2:
            freq = (len(self.scan_times) - 1) / max(self.scan_times[-1] - self.scan_times[0], 1e-6)
        lines = [f"Scans reçus  {self.scan_count:>8d}",
                 f"Fréquence    {freq:>6.1f} /s",
                 f"Points       {len(self.live):>8d}",
                 f"Sans écho    {100 * (1 - len(s) / max(len(self.live), 1)):>6.0f} %"]
        if len(s):
            k = int(np.argmin(s.ranges_m))
            lines.append(f"Plus proche  {s.ranges_m[k]:>5.2f} m à {s.angles_deg[k]:.0f}°")
            for name, ang in (("Devant", 0), ("Gauche", 90), ("Derrière", 180), ("Droite", 270)):
                sel = np.abs((s.angles_deg - ang + 180) % 360 - 180) <= 10
                val = f"{np.median(s.ranges_m[sel]):5.2f} m" if sel.any() else "   —"
                lines.append(f"{name:<12s} {val}")
        if self.paused:
            lines.append("⏸ EN PAUSE")
        self.stats_var.set("\n".join(lines))
        self.tel_var.set("\n".join(f"{k:<14s} {v:>10.1f}" for k, v in self.telemetry.items()) or "—")

    # ------------------------------------------------------------------ actions
    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_btn.configure(text="▶ Reprendre (Espace)" if self.paused else "⏸ Pause (Espace)")
        self.status("Pause : l'image est figée, tu peux la capturer." if self.paused else "Direct repris.")

    def recenter(self) -> None:
        pose = self.current_pose(quiet=True) or Pose()
        r = float(self.range_var.get())
        self.ax.set_xlim(pose.x - r, pose.x + r)
        self.ax.set_ylim(pose.y - r, pose.y + r)
        self.canvas.draw_idle()

    def current_pose(self, quiet: bool = False) -> Pose | None:
        try:
            return Pose(*(float(v.get().replace(",", ".")) for v in self.pose_vars))
        except ValueError:
            if not quiet:
                messagebox.showwarning("Pose", "x, y et θ doivent être des nombres (ex. 1.25).")
            return None

    def capture(self) -> None:
        if self.live is None:
            self.status("Rien à capturer : pas encore de scan reçu.")
            return
        pose = self.current_pose()
        if pose is None:
            return
        cap = self._add_capture(self.live.with_pose(pose))
        self.status(f"Scan {cap.number} capturé ({len(cap.scan)} points, pose {pose.x:.2f}, {pose.y:.2f}, {pose.theta_deg:.0f}°).")

    def _add_capture(self, scan: Scan, label: str = "") -> Capture:
        n = max((c.number for c in self.captures), default=0) + 1
        color = CAPTURE_COLORS[(n - 1) % len(CAPTURE_COLORS)]
        cap = Capture(n, scan, color)
        self.captures = self.captures + [cap]
        self.tree.insert("", "end", iid=str(n), text=f"● {label or f'Scan {n}'}", values=self._row(cap), tags=(color,))
        self.tree.tag_configure(color, foreground=color)
        self._draw_capture(cap)
        return cap

    @staticmethod
    def _row(cap: Capture) -> tuple:
        p = cap.scan.pose
        return (len(cap.scan.filtered()), f"{p.x:.2f}, {p.y:.2f}, {p.theta_deg:.0f}°")

    def _draw_capture(self, cap: Capture) -> None:
        old = self.artists.pop(cap.number, None)
        if old is not None:
            old.remove()
        if cap.visible:
            pts = cap.scan.filtered().global_points()
            self.artists[cap.number] = self.ax.scatter(pts[:, 0], pts[:, 1], s=6, c=cap.color, zorder=2)
        self.update_analysis()

    def _selected(self) -> Capture | None:
        sel = self.tree.selection()
        if not sel:
            self.status("Sélectionne d'abord une capture dans la liste.")
            return None
        return next(c for c in self.captures if c.number == int(sel[0]))

    def _replace(self, new: Capture) -> None:
        self.captures = [new if c.number == new.number else c for c in self.captures]
        self.tree.item(str(new.number), values=self._row(new))
        self._draw_capture(new)

    def _on_select_capture(self, _event) -> None:
        sel = self.tree.selection()
        if sel:
            p = next(c for c in self.captures if c.number == int(sel[0])).scan.pose
            for var, v in zip(self.pose_vars, (p.x, p.y, p.theta_deg)):
                var.set(f"{v:g}")

    def toggle_visible(self) -> None:
        cap = self._selected()
        if cap:
            self._replace(replace(cap, visible=not cap.visible))

    def apply_pose(self) -> None:
        cap, pose = self._selected(), self.current_pose()
        if cap and pose:
            self._replace(replace(cap, scan=cap.scan.with_pose(pose)))
            self.status(f"Pose du scan {cap.number} mise à jour.")

    def delete_capture(self) -> None:
        cap = self._selected()
        if cap and messagebox.askyesno("Supprimer", f"Supprimer le scan {cap.number} ?"):
            self.captures = [c for c in self.captures if c.number != cap.number]
            self.tree.delete(str(cap.number))
            self._draw_capture(replace(cap, visible=False))

    # ------------------------------------------------------------------ analyse
    def update_analysis(self) -> None:
        """Recalcule la grid map / les segments à partir des captures visibles."""
        from ..gridmap import grid_for_scans, update_grid
        from ..vectormap import build_vector_map

        for art in self.analysis_artists:
            art.remove()
        self.analysis_artists = []
        scans = [c.scan for c in self.captures if c.visible]
        if scans and self.grid_on.get():
            grid = grid_for_scans(scans, 0.05)
            for s in scans:
                grid = update_grid(grid, s)
            self.analysis_artists.append(self.ax.imshow(
                1 - grid.probability(), cmap="gray", vmin=0, vmax=1, origin="lower",
                extent=grid.extent(), alpha=0.55, zorder=0))
        if scans and self.seg_on.get():
            features = build_vector_map(scans).features
            for f in features:
                sg = f.segment
                (line,) = self.ax.plot([sg.x1, sg.x2], [sg.y1, sg.y2], "-", color="#ffffff", lw=2.2, zorder=5)
                self.analysis_artists.append(line)
            self.status(f"{len(features)} segment(s) extraits des captures visibles.")
        self.canvas.draw_idle()

    # ------------------------------------------------------------------ import
    def import_files(self) -> None:
        paths = filedialog.askopenfilenames(title="Importer des scans", filetypes=[("Scans CSV", "*.csv *.txt")])
        added = 0
        for path in paths:
            try:
                for label, scan in read_any_csv(Path(path)):
                    self._add_capture(scan, label)
                    added += 1
            except (ValueError, OSError, KeyError) as exc:
                messagebox.showerror("Import", f"{Path(path).name} : {exc}")
        if added:
            self.status(f"{added} scan(s) importé(s). Sélectionne-en un pour corriger sa pose.")

    # ------------------------------------------------------------------ exports
    def export_png(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".png", initialfile="carte_salle.png",
                                            filetypes=[("Image PNG", "*.png")])
        if path:
            self.fig.savefig(path, dpi=200, facecolor=BG)
            self.status(f"Image enregistrée : {path}")

    def export_table(self) -> None:
        if not self.captures:
            messagebox.showinfo("Export", "Aucune capture : capture au moins un scan.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="points_salle.csv",
                                            filetypes=[("Tableau CSV", "*.csv")])
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["scan", "x_robot_m", "y_robot_m", "theta_robot_deg", "angle_deg", "distance_m",
                        "intensite", "x_lidar_m", "y_lidar_m", "x_carte_m", "y_carte_m"])
            for cap in self.captures:
                s, p = cap.scan, cap.scan.pose
                loc, glob = s.local_points(), s.global_points()
                for k in range(len(s)):
                    w.writerow([cap.number, p.x, p.y, p.theta_deg, f"{s.angles_deg[k]:.3f}",
                                f"{s.ranges_m[k]:.4f}", int(s.intensities[k]),
                                f"{loc[k, 0]:.4f}", f"{loc[k, 1]:.4f}", f"{glob[k, 0]:.4f}", f"{glob[k, 1]:.4f}"])
        self.status(f"Tableau enregistré : {path} (tous les points, y compris sans écho : distance 0)")

    def export_tp(self) -> None:
        if not self.captures:
            messagebox.showinfo("Export", "Aucune capture : capture au moins un scan.")
            return
        prefix = simpledialog.askstring("Fichiers TP", "Nom des fichiers (ex. salle) :", initialvalue="salle")
        if not prefix:
            return
        folder = Path("data/scans")
        paths = [save_scan(c.scan, folder / f"{prefix}_{c.number:02d}.csv") for c in self.captures]
        self.status(f"{len(paths)} fichier(s) écrits dans {folder.resolve()} (utilisables par viewer, gridmap, vectormap)")

    def status(self, text: str) -> None:
        self.status_var.set(text)


def read_any_csv(path: Path) -> list[tuple[str, Scan]]:
    """Lit un fichier de scan : format TP / RobotControl / MapTool / LD06 (via load_scan),
    ou le « tableau complet » exporté par LidarScope (plusieurs scans, colonne 'scan')."""
    with open(path, encoding="utf-8", errors="replace") as f:
        first = next((line for line in f if line.strip() and not line.startswith("#")), "")
    if not first.startswith("scan,"):
        return [(path.stem, load_scan(path))]
    groups: dict[str, list[dict]] = collections.defaultdict(list)
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            groups[row["scan"]].append(row)
    out = []
    for num, rows in groups.items():
        r0 = rows[0]
        pose = Pose(float(r0["x_robot_m"]), float(r0["y_robot_m"]), float(r0["theta_robot_deg"]))
        col = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
        out.append((f"{path.stem} #{num}", Scan(col("angle_deg"), col("distance_m"), col("intensite"), pose)))
    return out


def main() -> None:
    root = tk.Tk()
    LidarScope(root)
    root.mainloop()


if __name__ == "__main__":
    main()
