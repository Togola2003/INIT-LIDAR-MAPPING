"""LidarScope : visualisation en direct, captures, traitement et échange de données du lidar LD06.

    .venv/Scripts/python -m lidar.scope        (ou double-clic sur LidarScope.bat)

Raccourcis : Espace = pause · C = capturer · M = règle de mesure · R = recentrer
"""

from __future__ import annotations

import collections
import queue
import time
from dataclasses import replace
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk

from ..extraction import scan_to_segments
from ..geometry import Pose
from ..gridmap import grid_for_scans, update_grid
from ..scan import Scan, save_scan
from ..vectormap import VectorMap, build_vector_map, save_vector_map
from . import theme as T
from .mapview import MapView
from .processing import Filters, average_scans, scan_stats
from .session import Capture, now_utc, read_any, save_session, save_table
from .sources import DemoSource, SerialSource, list_ports
from .widgets import LabeledSlider, PoseEntry, StatCard, button, font, section

REFRESH_MS = 100
BAUDS = ["115200", "230400", "460800", "921600"]
NUDGES = {"Fin": (0.01, 0.5), "Normal": (0.05, 2.0), "Gros": (0.20, 5.0)}
GRID_RES = {"2 cm": 0.02, "5 cm": 0.05, "10 cm": 0.10}


class LidarScope:
    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.events: queue.Queue = queue.Queue()
        self.source = None
        self.paused = False
        self.recent: collections.deque[Scan] = collections.deque(maxlen=10)   # derniers tours bruts
        self.captures: list[Capture] = []
        self.selected: int | None = None
        self.filters = Filters()
        self.scan_times: collections.deque = collections.deque(maxlen=20)
        self.scan_count = 0
        self.telemetry: dict[str, float] = {}
        self._framed = False
        self._analysis_job = None
        self.last_segments: list = []

        root.title("LidarScope · LD06")
        root.geometry("1500x900")
        root.minsize(1150, 720)
        root.configure(fg_color=T.BG)
        self._build_topbar()
        body = ctk.CTkFrame(root, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        body.grid_columnconfigure(0, weight=1)
        body.grid_rowconfigure(0, weight=1)
        self._build_map(body)
        self._build_sidebar(body)
        for key, fn in (("<space>", self.toggle_pause), ("c", self.capture), ("m", self.toggle_ruler),
                        ("r", self.recenter)):
            root.bind(key, lambda e, f=fn: None if self._typing(e) else f())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.refresh_ports()
        self._tick_job = root.after(REFRESH_MS, self._tick)
        root.after(400, self.recenter)   # cadre la vue une fois la fenêtre dimensionnée

    # ================================================================== construction
    def _build_topbar(self) -> None:
        bar = ctk.CTkFrame(self.root, fg_color=T.SURFACE, corner_radius=0, height=60)
        bar.pack(fill="x", pady=(0, 12))
        brand = ctk.CTkFrame(bar, fg_color="transparent")
        brand.pack(side="left", padx=16, pady=8)
        ctk.CTkLabel(brand, text="◉ LidarScope", font=font(18, "bold"), text_color=T.ACCENT).pack(anchor="w")
        ctk.CTkLabel(brand, text="LD06 · temps réel", font=font(10), text_color=T.MUTED).pack(anchor="w")

        ctk.CTkLabel(bar, text="Port", font=font(12), text_color=T.MUTED).pack(side="left", padx=(24, 6))
        self.port_menu = ctk.CTkOptionMenu(bar, values=["—"], width=110, fg_color=T.SURFACE_2,
                                           button_color=T.BORDER, button_hover_color=T.MUTED)
        self.port_menu.pack(side="left")
        button(bar, "↻", self.refresh_ports, width=34).pack(side="left", padx=6)
        ctk.CTkLabel(bar, text="Débit", font=font(12), text_color=T.MUTED).pack(side="left", padx=(12, 6))
        self.baud_menu = ctk.CTkOptionMenu(bar, values=BAUDS, width=100, fg_color=T.SURFACE_2,
                                           button_color=T.BORDER, button_hover_color=T.MUTED)
        self.baud_menu.pack(side="left")
        self.connect_btn = button(bar, "Connecter", self.toggle_connect, primary=True, width=120)
        self.connect_btn.pack(side="left", padx=(14, 6))
        button(bar, "Démo sans robot", self.start_demo).pack(side="left")

        pill = ctk.CTkFrame(bar, fg_color=T.SURFACE_2, corner_radius=14)
        pill.pack(side="right", padx=16)
        self.led = ctk.CTkLabel(pill, text="●", font=font(14), text_color=T.MUTED)
        self.led.pack(side="left", padx=(12, 4), pady=4)
        self.conn_label = ctk.CTkLabel(pill, text="Déconnecté", font=font(12), text_color=T.INK)
        self.conn_label.pack(side="left", padx=(0, 12))

    def _build_map(self, parent) -> None:
        card = ctk.CTkFrame(parent, fg_color=T.SURFACE, corner_radius=12)
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        tools = ctk.CTkFrame(card, fg_color="transparent")
        tools.pack(fill="x", padx=10, pady=(10, 4))
        self.pause_btn = button(tools, "⏸  Pause", self.toggle_pause, width=100)
        self.pause_btn.pack(side="left")
        button(tools, "⌖  Recentrer", self.recenter, width=110).pack(side="left", padx=6)
        self.ruler_btn = button(tools, "📏  Règle", self.toggle_ruler, width=100)
        self.ruler_btn.pack(side="left")
        ctk.CTkLabel(tools, text="Couleur du direct", font=font(11), text_color=T.MUTED).pack(side="left", padx=(18, 6))
        self.color_mode = ctk.CTkSegmentedButton(tools, values=["Uni", "Intensité"], command=lambda _: self._redraw_live(),
                                                 selected_color=T.ACCENT, selected_hover_color=T.ACCENT_HOVER,
                                                 unselected_color=T.SURFACE_2, text_color=T.INK)
        self.color_mode.set("Uni")
        self.color_mode.pack(side="left")
        self.live_seg_switch = ctk.CTkSwitch(tools, text="Segments en direct", font=font(11),
                                             progress_color=T.ACCENT, command=self._redraw_live)
        self.live_seg_switch.pack(side="left", padx=14)
        ctk.CTkLabel(tools, text="molette : zoom · glisser : déplacer", font=font(10),
                     text_color=T.MUTED).pack(side="right")

        holder = ctk.CTkFrame(card, fg_color=T.BG, corner_radius=8)
        holder.pack(fill="both", expand=True, padx=10)
        self.cursor_var = ctk.StringVar(value="Connecte le robot ou lance la démo.")
        self.map = MapView(holder, on_cursor=self.cursor_var.set)
        self.map.widget.pack(fill="both", expand=True, padx=2, pady=2)
        ctk.CTkLabel(card, textvariable=self.cursor_var, font=font(11, mono=True), text_color=T.MUTED,
                     anchor="w").pack(fill="x", padx=14, pady=(4, 8))

    def _build_sidebar(self, parent) -> None:
        tabs = ctk.CTkTabview(parent, width=390, fg_color=T.SURFACE, corner_radius=12,
                              segmented_button_selected_color=T.ACCENT,
                              segmented_button_selected_hover_color=T.ACCENT_HOVER,
                              segmented_button_unselected_color=T.SURFACE_2, text_color=T.INK)
        tabs.grid(row=0, column=1, sticky="ns")
        for name in ("Direct", "Captures", "Traitement", "Données"):
            tabs.add(name)
        self._tab_direct(ctk.CTkScrollableFrame(tabs.tab("Direct"), fg_color="transparent"))
        self._tab_captures(ctk.CTkScrollableFrame(tabs.tab("Captures"), fg_color="transparent"))
        self._tab_processing(ctk.CTkScrollableFrame(tabs.tab("Traitement"), fg_color="transparent"))
        self._tab_data(ctk.CTkScrollableFrame(tabs.tab("Données"), fg_color="transparent"))
        self.status_var = ctk.StringVar(value="")
        ctk.CTkLabel(parent, textvariable=self.status_var, font=font(11), text_color=T.MUTED, anchor="w",
                     wraplength=1300).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _tab_direct(self, tab) -> None:
        tab.pack(fill="both", expand=True)
        grid = ctk.CTkFrame(tab, fg_color="transparent")
        grid.pack(fill="x")
        self.cards = {}
        for k, name in enumerate(("Points", "Tours / s", "Sans écho", "Plus proche")):
            c = StatCard(grid, name)
            c.grid(row=k // 2, column=k % 2, sticky="ew", padx=3, pady=3)
            grid.grid_columnconfigure(k % 2, weight=1)
            self.cards[name] = c
        box = section(tab, "Distances autour du robot")
        self.dir_labels = {}
        for name in ("Devant", "Gauche", "Derrière", "Droite"):
            row = ctk.CTkFrame(box, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=2)
            ctk.CTkLabel(row, text=name, font=font(12), text_color=T.MUTED).pack(side="left")
            self.dir_labels[name] = ctk.CTkLabel(row, text="—", font=font(13, "bold", mono=True), text_color=T.INK)
            self.dir_labels[name].pack(side="right")
        box = section(tab, "Pose du robot (coin de référence)")
        self.pose_entry = PoseEntry(box)
        self.pose_entry.pack(padx=10, pady=(10, 4))
        ctk.CTkLabel(box, text="Le direct s'affiche à cette pose sur la carte.", font=font(10),
                     text_color=T.MUTED).pack(pady=(0, 8))
        box = section(tab, "Qualité de la capture")
        self.avg_slider = LabeledSlider(box, "Moyenne sur N tours", 1, 10, 3, "{:.0f}", lambda v: None, steps=9)
        self.avg_slider.pack(fill="x", padx=12, pady=10)
        button(tab, "📸   Capturer ce scan      C", self.capture, primary=True, height=46).pack(fill="x", pady=(16, 4))
        ctk.CTkLabel(tab, text="Espace : figer l'image avant de capturer", font=font(10),
                     text_color=T.MUTED).pack()
        box = section(tab, "Télémétrie")
        self.tel_label = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.tel_label.pack(anchor="w", padx=12, pady=8)

    def _tab_captures(self, tab) -> None:
        tab.pack(fill="both", expand=True)
        self.capture_list = ctk.CTkFrame(tab, fg_color="transparent")
        self.capture_list.pack(fill="x")
        self.empty_label = ctk.CTkLabel(self.capture_list, text="Aucune capture.\nOnglet Direct → Capturer, ou Données → Importer.",
                                        font=font(12), text_color=T.MUTED)
        self.empty_label.pack(pady=20)
        box = section(tab, "Recaler la capture sélectionnée")
        self.sel_label = ctk.CTkLabel(box, text="Clique sur une capture", font=font(12, "bold"), text_color=T.MUTED)
        self.sel_label.pack(pady=(8, 4))
        self.nudge_mode = ctk.CTkSegmentedButton(box, values=list(NUDGES), selected_color=T.ACCENT,
                                                 selected_hover_color=T.ACCENT_HOVER, unselected_color=T.SURFACE)
        self.nudge_mode.set("Normal")
        self.nudge_mode.pack(pady=4)
        pad = ctk.CTkFrame(box, fg_color="transparent")
        pad.pack(pady=6)
        for (r, c, txt, d) in ((0, 1, "↑", (0, 1, 0)), (1, 0, "←", (-1, 0, 0)), (1, 2, "→", (1, 0, 0)),
                               (2, 1, "↓", (0, -1, 0)), (0, 0, "⟲", (0, 0, 1)), (0, 2, "⟳", (0, 0, -1))):
            button(pad, txt, lambda d=d: self.nudge(*d), width=44, height=36, font=font(16)).grid(row=r, column=c, padx=3, pady=3)
        self.cap_pose = PoseEntry(box)
        self.cap_pose.pack(padx=10, pady=6)
        button(box, "Appliquer cette pose", self.apply_pose).pack(fill="x", padx=12, pady=(0, 10))
        box = section(tab, "Statistiques de la capture")
        self.cap_stats = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.cap_stats.pack(anchor="w", padx=12, pady=8)

    def _tab_processing(self, tab) -> None:
        tab.pack(fill="both", expand=True)
        box = section(tab, "Filtres (affichage et analyse)")
        self.f_min = LabeledSlider(box, "Portée minimale", 0, 1, 0.05, "{:.2f} m", self._filters_changed)
        self.f_max = LabeledSlider(box, "Portée maximale", 0.5, 12, 12, "{:.1f} m", self._filters_changed)
        self.f_int = LabeledSlider(box, "Intensité minimale", 0, 255, 0, "{:.0f}", self._filters_changed, steps=255)
        for s in (self.f_min, self.f_max, self.f_int):
            s.pack(fill="x", padx=12, pady=6)
        ctk.CTkLabel(box, text="Les exports gardent toujours les mesures brutes.", font=font(10),
                     text_color=T.MUTED).pack(pady=(0, 8))
        box = section(tab, "Grid map (carte d'occupation)")
        self.grid_switch = ctk.CTkSwitch(box, text="Afficher la grid map", progress_color=T.ACCENT,
                                         command=self.schedule_analysis)
        self.grid_switch.pack(anchor="w", padx=12, pady=(10, 4))
        self.grid_res = ctk.CTkSegmentedButton(box, values=list(GRID_RES), command=lambda _: self.schedule_analysis(),
                                               selected_color=T.ACCENT, selected_hover_color=T.ACCENT_HOVER,
                                               unselected_color=T.SURFACE)
        self.grid_res.set("5 cm")
        self.grid_res.pack(padx=12, pady=(0, 10), anchor="w")
        box = section(tab, "Carte vectorielle (segments)")
        self.seg_switch = ctk.CTkSwitch(box, text="Extraire les murs des captures", progress_color=T.ACCENT,
                                        command=self.schedule_analysis)
        self.seg_switch.pack(anchor="w", padx=12, pady=10)
        box = section(tab, "Vue")
        self.view_range = LabeledSlider(box, "Taille de la vue (recentrer)", 1, 12, 5, "± {:.1f} m", lambda v: None)
        self.view_range.pack(fill="x", padx=12, pady=10)
        box = section(tab, "Résultat de l'analyse")
        self.analysis_label = ctk.CTkLabel(box, text="Active la grid map ou les segments.", font=font(11, mono=True),
                                           text_color=T.INK, justify="left")
        self.analysis_label.pack(anchor="w", padx=12, pady=8)

    def _tab_data(self, tab) -> None:
        tab.pack(fill="both", expand=True)
        box = section(tab, "Importer")
        button(box, "📂  Importer des fichiers…", self.import_files).pack(fill="x", padx=12, pady=(12, 4))
        ctk.CTkLabel(box, text="Session .json, tableau LidarScope, scans CSV\n(TP, RobotControl, MapTool, LD06)",
                     font=font(10), text_color=T.MUTED).pack(pady=(0, 10))
        box = section(tab, "Exporter")
        button(box, "💾  Session complète (JSON ouvert)", self.export_session, primary=True).pack(fill="x", padx=12, pady=(12, 2))
        ctk.CTkLabel(box, text="Recommandé : relisible ici et compréhensible par tout\nprogramme, site web ou IA "
                     "(unités, repères et colonnes\ndécrits dans le fichier).", font=font(10),
                     text_color=T.MUTED, justify="left").pack(padx=12, pady=(0, 8), anchor="w")
        for txt, fn in (("📊  Tableau de points (CSV)", self.export_table),
                        ("🖼  Image de la carte (PNG)", self.export_png),
                        ("📐  Segments (TXT, format du module)", self.export_segments),
                        ("📁  Fichiers pour le TP → data/scans", self.export_tp)):
            button(box, txt, fn, anchor="w").pack(fill="x", padx=12, pady=3)
        ctk.CTkFrame(box, fg_color="transparent", height=8).pack()

    # ================================================================== connexion
    def refresh_ports(self) -> None:
        ports = list_ports() or ["—"]
        self.port_menu.configure(values=ports)
        if self.port_menu.get() not in ports:
            self.port_menu.set(ports[0])
        self.status(f"{len(ports)} port(s) série trouvé(s)." if ports != ["—"] else "Aucun port série : branche l'ESP32.")

    def toggle_connect(self) -> None:
        if self.source:
            self.disconnect()
            return
        port = self.port_menu.get()
        if port == "—":
            messagebox.showwarning("Port série", "Aucun port sélectionné. Branche l'ESP32 puis clique sur ↻.")
            return
        self._start(SerialSource(port, int(self.baud_menu.get()), self.events))

    def start_demo(self) -> None:
        self.disconnect()
        self._start(DemoSource(self.events))

    def _start(self, source) -> None:
        self.source, self.scan_count, self._framed = source, 0, False
        source.start()
        self.connect_btn.configure(text="Déconnecter", fg_color=T.DANGER, hover_color="#c0392b")
        self.led.configure(text_color=T.WARN)

    def disconnect(self) -> None:
        if self.source:
            self.source.stop()
            self.source = None
        self.connect_btn.configure(text="Connecter", fg_color=T.ACCENT, hover_color=T.ACCENT_HOVER)
        self.led.configure(text_color=T.MUTED)

    def quit(self) -> None:
        self.disconnect()
        for job in (self._tick_job, self._analysis_job):
            if job:
                self.root.after_cancel(job)
        self.root.quit()
        self.root.destroy()

    # ================================================================== boucle temps réel
    def _tick(self) -> None:
        got_scan = False
        while True:
            try:
                ev = self.events.get_nowait()
            except queue.Empty:
                break
            if ev.kind == "scan":
                got_scan = True
                self.scan_count += 1
                self.scan_times.append(time.monotonic())
                if not self.paused:
                    self.recent.append(ev.payload)
            elif ev.kind == "telemetry":
                self.telemetry[ev.payload[0]] = ev.payload[1]
            elif ev.kind == "status":
                self.conn_label.configure(text=ev.payload)
                self.status(ev.payload)
            elif ev.kind == "error":
                self.led.configure(text_color=T.DANGER)
                self.disconnect()
                messagebox.showerror("Port série", f"{ev.payload}\n\nLe moniteur série Arduino est-il encore ouvert ?")
        if got_scan:
            self.led.configure(text_color=T.ACCENT)
            if not self.paused:
                self._redraw_live()
        self._tick_job = self.root.after(REFRESH_MS, self._tick)

    @property
    def live(self) -> Scan | None:
        return self.recent[-1] if self.recent else None

    def _pose(self, entry: PoseEntry, quiet: bool = True) -> Pose | None:
        try:
            return Pose(*entry.get())
        except ValueError:
            if not quiet:
                messagebox.showwarning("Pose", "x, y et θ doivent être des nombres (ex. 1.25).")
            return None

    def _redraw_live(self) -> None:
        live = self.live
        pose = self._pose(self.pose_entry) or Pose()
        self.map.draw_robot(pose)
        if live is None:
            return
        shown = self.filters.apply(live).with_pose(pose)
        segs = scan_to_segments(shown) if self.live_seg_switch.get() else []
        self.map.draw_live(shown, self.color_mode.get() == "Intensité", segs)
        if not self._framed:
            self.recenter()
            self._framed = True
        self._update_live_panel(live)

    def _update_live_panel(self, live: Scan) -> None:
        st = scan_stats(live, self.filters)
        freq = 0.0
        if len(self.scan_times) >= 2:
            freq = (len(self.scan_times) - 1) / max(self.scan_times[-1] - self.scan_times[0], 1e-6)
        self.cards["Points"].set(f"{st['points']}", T.INK if st["points"] >= 300 else T.WARN)
        self.cards["Tours / s"].set(f"{freq:.1f}")
        self.cards["Sans écho"].set(f"{st['no_echo_pct']:.0f} %", T.INK if st["no_echo_pct"] < 30 else T.WARN)
        if "nearest" in st:
            r, a = st["nearest"]
            self.cards["Plus proche"].set(f"{r:.2f} m", T.DANGER if r < 0.2 else T.INK)
            for name, val in st["directions"].items():
                self.dir_labels[name].configure(text=f"{val:.2f} m" if val is not None else "—")
        self.tel_label.configure(text="\n".join(f"{k:<16s}{v:>10.1f}" for k, v in self.telemetry.items()) or "—")

    # ================================================================== actions carte
    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_btn.configure(text="▶  Reprendre" if self.paused else "⏸  Pause",
                                 fg_color=T.WARN if self.paused else T.SURFACE_2,
                                 text_color="#000" if self.paused else T.INK)
        self.status("Image figée : tu peux la capturer (C)." if self.paused else "Direct repris.")

    def toggle_ruler(self) -> None:
        on = not self.map.ruler_mode
        self.map.set_ruler(on)
        self.ruler_btn.configure(fg_color=T.WARN if on else T.SURFACE_2, text_color="#000" if on else T.INK)
        self.status("Règle : clique deux points sur la carte." if on else "Règle désactivée.")

    def recenter(self) -> None:
        self.map.frame(self._pose(self.pose_entry) or Pose(), self.view_range.get())

    # ================================================================== captures
    def capture(self) -> None:
        if not self.recent:
            self.status("Rien à capturer : aucun scan reçu.")
            return
        pose = self._pose(self.pose_entry, quiet=False)
        if pose is None:
            return
        n_avg = int(round(self.avg_slider.get()))
        scan = average_scans(list(self.recent)[-n_avg:]).with_pose(pose)
        cap = self._add_capture(scan)
        self.status(f"{cap.title} capturé : {len(scan)} points, moyenne de {min(n_avg, len(self.recent))} tour(s), "
                    f"pose ({pose.x:.2f}, {pose.y:.2f}, {pose.theta_deg:.0f}°).")

    def _add_capture(self, scan: Scan, label: str = "") -> Capture:
        n = max((c.number for c in self.captures), default=0) + 1
        cap = Capture(n, scan, T.CAPTURE_COLORS[(n - 1) % len(T.CAPTURE_COLORS)], label, True, now_utc())
        self.captures = self.captures + [cap]
        self._refresh_capture_list()
        self._draw_capture(cap)
        self.select(n)
        return cap

    def _draw_capture(self, cap: Capture) -> None:
        self.map.draw_capture(cap.number, self.filters.apply(cap.scan) if cap.visible else None, cap.color)
        self.schedule_analysis()

    def _find(self, number: int | None) -> Capture | None:
        return next((c for c in self.captures if c.number == number), None)

    def _replace(self, new: Capture) -> None:
        self.captures = [new if c.number == new.number else c for c in self.captures]
        self._draw_capture(new)
        self._refresh_capture_list()

    def _refresh_capture_list(self) -> None:
        for w in self.capture_list.winfo_children():
            w.destroy()
        if not self.captures:
            ctk.CTkLabel(self.capture_list, text="Aucune capture.\nOnglet Direct → Capturer, ou Données → Importer.",
                         font=font(12), text_color=T.MUTED).pack(pady=20)
            return
        for cap in self.captures:
            sel = cap.number == self.selected
            row = ctk.CTkFrame(self.capture_list, fg_color=T.SURFACE_2, corner_radius=10, border_width=2,
                               border_color=cap.color if sel else T.SURFACE_2)
            row.pack(fill="x", pady=3)
            ctk.CTkLabel(row, text="●", font=font(20), text_color=cap.color if cap.visible else T.MUTED,
                         width=24).pack(side="left", padx=(10, 4))
            info = ctk.CTkFrame(row, fg_color="transparent")
            info.pack(side="left", fill="x", expand=True, pady=6)
            p = cap.scan.pose
            ctk.CTkLabel(info, text=cap.title, font=font(13, "bold"), text_color=T.INK, anchor="w").pack(fill="x")
            ctk.CTkLabel(info, text=f"{int((cap.scan.ranges_m > 0).sum())} pts · ({p.x:.2f}, {p.y:.2f}, {p.theta_deg:.0f}°)",
                         font=font(10, mono=True), text_color=T.MUTED, anchor="w").pack(fill="x")
            button(row, "🗑", lambda n=cap.number: self.delete_capture(n), width=32, height=30).pack(side="right", padx=(2, 8))
            button(row, "👁" if cap.visible else "◌", lambda n=cap.number: self.toggle_visible(n),
                   width=32, height=30).pack(side="right")
            for w in (row, info, *info.winfo_children()):
                w.bind("<Button-1>", lambda e, n=cap.number: self.select(n))

    def select(self, number: int) -> None:
        cap = self._find(number)
        if cap is None:
            return
        self.selected = number
        p = cap.scan.pose
        self.cap_pose.set(p.x, p.y, p.theta_deg)
        self.sel_label.configure(text=f"● {cap.title}", text_color=cap.color)
        st = scan_stats(cap.scan, self.filters)
        txt = [f"Points           {st['points']}", f"Avec écho        {st['kept']}",
               f"Sans écho        {st['no_echo_pct']:.0f} %"]
        if "nearest" in st:
            txt += [f"Plus proche      {st['nearest'][0]:.2f} m à {st['nearest'][1]:.0f}°",
                    f"Plus loin        {st['max_range']:.2f} m", f"Intensité moy.   {st['mean_intensity']:.0f}"]
        txt.append(f"Capturé          {cap.captured_utc[11:19] or '—'} UTC")
        self.cap_stats.configure(text="\n".join(txt))
        self._refresh_capture_list()

    def nudge(self, dx: int, dy: int, dtheta: int) -> None:
        cap = self._find(self.selected)
        if cap is None:
            self.status("Sélectionne d'abord une capture.")
            return
        step_m, step_deg = NUDGES[self.nudge_mode.get()]
        new = replace(cap, scan=cap.scan.with_pose(cap.scan.pose.moved(dx * step_m, dy * step_m, dtheta * step_deg)))
        self._replace(new)
        p = new.scan.pose
        self.cap_pose.set(p.x, p.y, p.theta_deg)

    def apply_pose(self) -> None:
        cap, pose = self._find(self.selected), self._pose(self.cap_pose, quiet=False)
        if cap and pose:
            self._replace(replace(cap, scan=cap.scan.with_pose(pose)))

    def toggle_visible(self, number: int) -> None:
        cap = self._find(number)
        if cap:
            self._replace(replace(cap, visible=not cap.visible))

    def delete_capture(self, number: int) -> None:
        cap = self._find(number)
        if cap and messagebox.askyesno("Supprimer", f"Supprimer {cap.title} ?"):
            self.captures = [c for c in self.captures if c.number != number]
            self.map.draw_capture(number, None)
            self.selected = None if self.selected == number else self.selected
            self._refresh_capture_list()
            self.schedule_analysis()

    # ================================================================== traitement
    def _filters_changed(self, _v: float) -> None:
        self.filters = Filters(self.f_min.get(), max(self.f_max.get(), self.f_min.get() + 0.01), int(self.f_int.get()))
        if self._analysis_job:
            self.root.after_cancel(self._analysis_job)
        self._analysis_job = self.root.after(250, self._refilter)

    def _refilter(self) -> None:
        for cap in self.captures:
            self.map.draw_capture(cap.number, self.filters.apply(cap.scan) if cap.visible else None, cap.color)
        self._redraw_live()
        self.run_analysis()

    def schedule_analysis(self) -> None:
        if self._analysis_job:
            self.root.after_cancel(self._analysis_job)
        self._analysis_job = self.root.after(150, self.run_analysis)

    def run_analysis(self) -> None:
        self._analysis_job = None
        scans = [self.filters.apply(c.scan) for c in self.captures if c.visible]
        scans = [s for s in scans if len(s)]
        grid, segs, lines = None, [], []
        if scans and self.grid_switch.get():
            grid = grid_for_scans(scans, GRID_RES[self.grid_res.get()])
            for s in scans:
                grid = update_grid(grid, s)
            ny, nx = grid.shape
            lines += [f"Grid map  {nx} × {ny} cellules", f"  occupées  {int(grid.occupied_mask().sum())}",
                      f"  libres    {int(grid.free_mask().sum())}"]
        if scans and self.seg_switch.get():
            vmap = build_vector_map(scans)
            segs = vmap.segments
            total = sum(s.length for s in segs)
            lines += [f"Segments  {len(segs)}", f"  longueur totale  {total:.2f} m"]
        self.last_segments = segs
        self.map.draw_analysis(grid, segs)
        self.analysis_label.configure(text="\n".join(lines) or "Active la grid map ou les segments.")

    # ================================================================== import / export
    def import_files(self) -> None:
        paths = filedialog.askopenfilenames(title="Importer", filetypes=[("Données lidar", "*.json *.csv *.txt"),
                                                                          ("Tous", "*.*")])
        added = 0
        for path in paths:
            try:
                for label, scan in read_any(path):
                    self._add_capture(scan, label)
                    added += 1
            except (ValueError, OSError, KeyError) as exc:
                messagebox.showerror("Import", f"{Path(path).name} :\n{exc}")
        if added:
            self.status(f"{added} scan(s) importé(s).")

    def _need_captures(self) -> bool:
        if not self.captures:
            messagebox.showinfo("Export", "Aucune capture. Capture au moins un scan (onglet Direct).")
            return False
        return True

    def _ask_save(self, ext: str, name: str, kind: str) -> str:
        return filedialog.asksaveasfilename(defaultextension=ext, initialfile=name, filetypes=[(kind, f"*{ext}")])

    def export_session(self) -> None:
        if self._need_captures() and (path := self._ask_save(".json", "session_salle.lidar.json", "Session LidarScope")):
            extra = {"display_filters": self.filters.as_dict(),
                     "segments_xyxy_m": [list(s.as_tuple()) for s in self.last_segments]}
            save_session(self.captures, path, extra)
            self.status(f"Session enregistrée : {path}")

    def export_table(self) -> None:
        if self._need_captures() and (path := self._ask_save(".csv", "points_salle.csv", "Tableau CSV")):
            save_table(self.captures, path)
            self.status(f"Tableau enregistré : {path}")

    def export_png(self) -> None:
        if path := self._ask_save(".png", "carte_salle.png", "Image PNG"):
            self.map.fig.savefig(path, dpi=200, facecolor=T.BG)
            self.status(f"Image enregistrée : {path}")

    def export_segments(self) -> None:
        if not self.last_segments:
            messagebox.showinfo("Segments", "Active d'abord « Extraire les murs » dans l'onglet Traitement.")
            return
        if path := self._ask_save(".txt", "carte_vectorielle.txt", "Segments"):
            from ..vectormap import Feature
            save_vector_map(VectorMap(tuple(Feature(s) for s in self.last_segments), len(self.captures)), path)
            self.status(f"Segments enregistrés : {path}")

    def export_tp(self) -> None:
        if not self._need_captures():
            return
        dialog = ctk.CTkInputDialog(text="Nom des fichiers (ex. salle) :", title="Fichiers pour le TP")
        prefix = (dialog.get_input() or "").strip()
        if prefix:
            paths = [save_scan(c.scan, Path("data/scans") / f"{prefix}_{c.number:02d}.csv") for c in self.captures]
            self.status(f"{len(paths)} fichier(s) écrits dans {Path('data/scans').resolve()}")

    # ================================================================== divers
    def status(self, text: str) -> None:
        self.status_var.set(text)

    @staticmethod
    def _typing(event) -> bool:
        return "entry" in str(event.widget).lower()


def main() -> None:
    ctk.set_appearance_mode("dark")
    root = ctk.CTk()
    LidarScope(root)
    root.mainloop()


if __name__ == "__main__":
    main()
