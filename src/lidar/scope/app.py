"""LidarScope : acquisition en direct, captures, cartographie et échange de données du lidar LD06.

    .venv/Scripts/python -m lidar.scope        (ou double-clic sur LidarScope.bat)

Raccourcis : Espace = pause · C = capturer (récolte N tours, C de nouveau = arrêter)
            Entrée = garder la capture · Échap = la jeter · M = règle · R = recentrer
Captures (bouton ✋) : glisser = déplacer · clic droit glissé = tourner · Ctrl + clic = sélection multiple
            Ctrl + molette = ±1° · flèches / A E = déplacer / tourner · Ctrl+Z = annuler
"""

from __future__ import annotations

import collections
import queue
import time
from dataclasses import replace
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk
import numpy as np

from ..extraction import scan_to_segments
from ..geometry import Pose, move_rigid
from ..scan import Scan
from ..segments import Segment
from ..vectormap import load_vector_map
from . import theme as T
from .carto_panel import CartoPanel
from .collector import COMBINE_MODES, Collection, finish
from .export_dialog import ExportDialog
from .exporter import ExportOptions, export_bundle
from .mapping import beam_image, build_maps
from .mapview import Editor, MapView
from .processing import scan_stats
from .session import Capture, now_utc, read_any, save_session
from .sources import FORMAT_TELEPLOT, DemoSource, Frame, SerialSource, list_ports
from .widgets import LabeledSlider, PoseEntry, StatCard, button, font, section

REFRESH_MS = 100
BAUDS = ["115200", "230400", "460800", "921600"]
NUDGES = {"Fin": (0.01, 0.5), "Normal": (0.05, 2.0), "Gros": (0.20, 5.0)}
FORMAT_INFO = {
    "printScanCSV": ("printScanCSV · données complètes ✓", T.ACCENT),
    FORMAT_TELEPLOT: ("printScanTeleplot · intensité non transmise ⚠\nPour tout avoir : remplacer par "
                      "ld06.printScanCSV(Serial); dans le .ino", T.WARN),
}


class LidarScope:
    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.events: queue.Queue = queue.Queue()
        self.source = None
        self.paused = False
        self.recent: collections.deque[Frame] = collections.deque(maxlen=10)   # derniers tours bruts reçus
        self.captures: list[Capture] = []
        self.reference: list[Segment] = []     # carte de référence importée (ex. murs dessinés dans MapTool)
        self.selected: int | None = None
        self.sel: set[int] = set()                    # captures déplacées / tournées ensemble
        self.history: list[dict[int, Pose]] = []      # poses avant chaque déplacement (Ctrl+Z)
        self.scan_times: collections.deque = collections.deque(maxlen=20)
        self.scan_count = 0
        self.telemetry: dict[str, float] = {}
        self._framed = False
        self.collection: Collection | None = None   # récolte de tours en cours
        self.preview: tuple | None = None           # (scan, bilan, lignes brutes, format) en attente de « Garder »
        self._analysis_job = None
        self.last_result = None

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
                        ("r", self.recenter), ("<Return>", self.keep_preview), ("<Escape>", self.drop_preview)):
            root.bind(key, lambda e, f=fn: None if self._typing(e) else f())
        for key, d in {"<Left>": (-1, 0, 0), "<Right>": (1, 0, 0), "<Up>": (0, 1, 0), "<Down>": (0, -1, 0),
                       "a": (0, 0, 1), "e": (0, 0, -1)}.items():
            root.bind(key, lambda ev, d=d: None if self._typing(ev) or self.map.editor is None else self.nudge(*d))
        root.bind("<Control-z>", lambda ev: self.undo())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.refresh_ports()
        self._tick_job = root.after(REFRESH_MS, self._tick)
        root.after(400, self.recenter)

    # ================================================================== construction
    def _build_topbar(self) -> None:
        bar = ctk.CTkFrame(self.root, fg_color=T.SURFACE, corner_radius=0, height=60)
        bar.pack(fill="x", pady=(0, 12))
        brand = ctk.CTkFrame(bar, fg_color="transparent")
        brand.pack(side="left", padx=16, pady=8)
        ctk.CTkLabel(brand, text="◉ LidarScope", font=font(18, "bold"), text_color=T.ACCENT).pack(anchor="w")
        ctk.CTkLabel(brand, text="LD06 · acquisition & cartographie", font=font(10), text_color=T.MUTED).pack(anchor="w")
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
        ctk.CTkLabel(tools, text="Direct", font=font(11), text_color=T.MUTED).pack(side="left", padx=(18, 6))
        self.color_mode = ctk.CTkSegmentedButton(tools, values=["Masqué", "Uni", "Intensité"],
                                                 command=lambda _: self._redraw_live(), selected_color=T.ACCENT,
                                                 selected_hover_color=T.ACCENT_HOVER, unselected_color=T.SURFACE_2)
        self.color_mode.set("Uni")
        self.color_mode.pack(side="left")
        self.live_seg_switch = ctk.CTkSwitch(tools, text="Segments en direct", font=font(11),
                                             progress_color=T.ACCENT, command=self._redraw_live)
        self.live_seg_switch.pack(side="left", padx=14)
        self.edit_btn = button(tools, "✋  Déplacer les captures", self.toggle_edit, width=170)
        self.edit_btn.pack(side="right")
        button(tools, "↶", self.undo, width=34).pack(side="right", padx=6)
        holder = ctk.CTkFrame(card, fg_color=T.BG, corner_radius=8)
        holder.pack(fill="both", expand=True, padx=10)
        self.cursor_var = ctk.StringVar(value="Connecte le robot, lance la démo, ou importe des données.")
        self.map = MapView(holder, on_cursor=self.cursor_var.set)
        self.map.widget.pack(fill="both", expand=True, padx=2, pady=2)
        ctk.CTkLabel(card, textvariable=self.cursor_var, font=font(11, mono=True), text_color=T.MUTED,
                     anchor="w").pack(fill="x", padx=14, pady=(4, 0))
        ctk.CTkLabel(card, text="molette : zoom · glisser le vide : vue · ✋ actif : glisser une capture = la déplacer, "
                     "clic droit glissé = la tourner, Ctrl+clic = plusieurs, Ctrl+Z = annuler", font=font(10),
                     text_color=T.MUTED, anchor="w").pack(fill="x", padx=14, pady=(0, 8))

    def _build_sidebar(self, parent) -> None:
        tabs = ctk.CTkTabview(parent, width=400, fg_color=T.SURFACE, corner_radius=12, command=self._tab_changed,
                              segmented_button_selected_color=T.ACCENT,
                              segmented_button_selected_hover_color=T.ACCENT_HOVER,
                              segmented_button_unselected_color=T.SURFACE_2, text_color=T.INK)
        tabs.grid(row=0, column=1, sticky="ns")
        self.tabs = tabs
        frames = {}
        for name in ("Direct", "Captures", "Cartographie", "Données"):
            tabs.add(name)
            frames[name] = ctk.CTkScrollableFrame(tabs.tab(name), fg_color="transparent")
            frames[name].pack(fill="both", expand=True)
        self._tab_direct(frames["Direct"])
        self._tab_captures(frames["Captures"])
        self.carto = CartoPanel(frames["Cartographie"], on_change=self.run_analysis, on_filters=self._refilter)
        self._tab_data(frames["Données"])
        self.status_var = ctk.StringVar(value="")
        ctk.CTkLabel(parent, textvariable=self.status_var, font=font(11), text_color=T.MUTED, anchor="w",
                     wraplength=1300).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _tab_direct(self, tab) -> None:
        grid = ctk.CTkFrame(tab, fg_color="transparent")
        grid.pack(fill="x")
        self.cards = {}
        for k, name in enumerate(("Points", "Tours / s", "Sans écho", "Plus proche")):
            c = StatCard(grid, name)
            c.grid(row=k // 2, column=k % 2, sticky="ew", padx=3, pady=3)
            grid.grid_columnconfigure(k % 2, weight=1)
            self.cards[name] = c
        self.format_label = ctk.CTkLabel(tab, text="Format reçu : —", font=font(10), text_color=T.MUTED,
                                         justify="left", wraplength=340)
        self.format_label.pack(anchor="w", padx=6, pady=(6, 0))
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
        box = section(tab, "Capture (récolte de plusieurs tours)")
        self.avg_slider = LabeledSlider(box, "Tours à récolter", 1, 30, 10, "{:.0f}", lambda v: None, steps=29)
        self.avg_slider.pack(fill="x", padx=12, pady=(10, 4))
        ctk.CTkLabel(box, text="Combiner les tours", font=font(11), text_color=T.MUTED).pack(anchor="w", padx=12)
        self.combine_mode = ctk.CTkSegmentedButton(box, values=list(COMBINE_MODES), selected_color=T.ACCENT,
                                                   selected_hover_color=T.ACCENT_HOVER, unselected_color=T.SURFACE)
        self.combine_mode.set(COMBINE_MODES[0])
        self.combine_mode.pack(anchor="w", padx=12, pady=(0, 4))
        ctk.CTkLabel(box, text="Médiane : un point par angle, bruit réduit · Tous : nuage plus dense", font=font(10),
                     text_color=T.MUTED).pack(anchor="w", padx=12)
        self.view_range = LabeledSlider(box, "Taille de la vue (Recentrer)", 1, 12, 5, "± {:.1f} m", lambda v: None)
        self.view_range.pack(fill="x", padx=12, pady=(6, 10))
        self.capture_btn = button(tab, "📸   Capturer (récolter les tours)     C", self.capture, primary=True,
                                  height=46)
        self.capture_btn.pack(fill="x", pady=(16, 4))
        self.collect_bar = ctk.CTkProgressBar(tab, progress_color=T.ACCENT, fg_color=T.SURFACE_2, height=8)
        self.collect_bar.set(0)
        self.collect_bar.pack(fill="x", pady=(2, 2))
        self.collect_label = ctk.CTkLabel(tab, text="Le robot doit rester immobile pendant la récolte.",
                                          font=font(10), text_color=T.MUTED)
        self.collect_label.pack()
        self.preview_box = ctk.CTkFrame(tab, fg_color=T.SURFACE_2, corner_radius=10, border_width=2,
                                        border_color=T.WARN)
        ctk.CTkLabel(self.preview_box, text="APERÇU DE LA CAPTURE (jaune sur la carte)", font=font(10, "bold"),
                     text_color=T.WARN).pack(anchor="w", padx=12, pady=(8, 0))
        self.preview_text = ctk.CTkLabel(self.preview_box, text="", font=font(11, mono=True), text_color=T.INK,
                                         justify="left")
        self.preview_text.pack(anchor="w", padx=12, pady=4)
        row = ctk.CTkFrame(self.preview_box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 10))
        button(row, "✓ Garder  ⏎", self.keep_preview, primary=True, width=110).pack(side="left", padx=3)
        button(row, "↻ Refaire", self.redo_preview, width=90).pack(side="left", padx=3)
        button(row, "✕ Jeter  Échap", self.drop_preview, width=110).pack(side="left", padx=3)
        box = section(tab, "Télémétrie")
        self.tel_label = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.tel_label.pack(anchor="w", padx=12, pady=8)

    def _tab_captures(self, tab) -> None:
        self.capture_list = ctk.CTkFrame(tab, fg_color="transparent")
        self.capture_list.pack(fill="x")
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
        self._refresh_capture_list()

    def _tab_data(self, tab) -> None:
        box = section(tab, "Importer")
        button(box, "📂  Importer des fichiers…", self.import_files).pack(fill="x", padx=12, pady=(12, 4))
        ctk.CTkLabel(box, text="• Scans : session .lidar.json, tableau LidarScope, CSV du TP,\n"
                     "  RobotControl, MapTool, LD06 (printScanCSV)\n"
                     "• Carte de référence : .txt x1,y1,x2,y2 (export MapTool)", font=font(10),
                     text_color=T.MUTED, justify="left").pack(anchor="w", padx=12, pady=(0, 10))
        box = section(tab, "Exporter")
        button(box, "📦  Exporter…  (chaque scan / ensemble)", self.open_export, primary=True).pack(
            fill="x", padx=12, pady=(12, 2))
        ctk.CTkLabel(box, text="Crée un dossier avec, au choix : session JSON auto-décrite, tableau CSV,\n"
                     "rendu PNG, grid map (CSV MapTool / ternaire / proba), carte vectorielle,\n"
                     "fichiers TP et un LISEZMOI.md qui explique tout.", font=font(10),
                     text_color=T.MUTED, justify="left").pack(anchor="w", padx=12, pady=(0, 8))
        button(box, "💾  Session rapide (.lidar.json)", self.export_session).pack(fill="x", padx=12, pady=3)
        button(box, "🖼  Capture d'écran de la carte (PNG)", self.export_png).pack(fill="x", padx=12, pady=(3, 12))
        box = section(tab, "Pourquoi le JSON ?")
        ctk.CTkLabel(box, text="Il décrit lui-même ses unités, ses repères, la formule de la pose et chaque "
                     "colonne, et garde les lignes série BRUTES reçues du robot : un autre programme, un site "
                     "ou une IA peut tout relire de bout en bout, sans explication.", font=font(10),
                     text_color=T.MUTED, justify="left", wraplength=330).pack(anchor="w", padx=12, pady=8)

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

    # ================================================================== temps réel
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
                if self.collection is not None:
                    self.collection = self.collection.add(ev.payload)
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
        if self.collection is not None:
            self._collect_step()
        self._tick_job = self.root.after(REFRESH_MS, self._tick)

    @property
    def live(self) -> Scan | None:
        return self.recent[-1].scan if self.recent else None

    def _pose(self, entry: PoseEntry, quiet: bool = True) -> Pose | None:
        try:
            return Pose(*entry.get())
        except ValueError:
            if not quiet:
                messagebox.showwarning("Pose", "x, y et θ doivent être des nombres (ex. 1.25).")
            return None

    def _redraw_live(self) -> None:
        pose = self._pose(self.pose_entry) or Pose()
        self.map.draw_robot(pose)
        live = self.live
        if live is None:
            return
        mode = self.color_mode.get()
        hidden = mode == "Masqué" or self.preview is not None     # l'aperçu reste seul : pas de clignotement
        shown = self.carto.filters().apply(live).with_pose(pose) if not hidden else None
        segs = scan_to_segments(shown) if shown is not None and self.live_seg_switch.get() else []
        self.map.draw_live(shown, mode == "Intensité" and not np.isnan(live.intensities).all(), segs)
        if not self._framed:
            self.recenter()
            self._framed = True
        self._update_live_panel(live)

    def _update_live_panel(self, live: Scan) -> None:
        st = scan_stats(live, self.carto.filters())
        freq = 0.0
        if len(self.scan_times) >= 2:
            freq = (len(self.scan_times) - 1) / max(self.scan_times[-1] - self.scan_times[0], 1e-6)
        self.cards["Points"].set(f"{st['points']}", T.INK if st["points"] >= 300 else T.WARN)
        self.cards["Tours / s"].set(f"{freq:.1f}")
        self.cards["Sans écho"].set(f"{st['no_echo_pct']:.0f} %", T.INK if st["no_echo_pct"] < 30 else T.WARN)
        if "nearest" in st:
            r, _ = st["nearest"]
            self.cards["Plus proche"].set(f"{r:.2f} m", T.DANGER if r < 0.2 else T.INK)
            for name, val in st["directions"].items():
                self.dir_labels[name].configure(text=f"{val:.2f} m" if val is not None else "—")
        text, color = FORMAT_INFO.get(self.recent[-1].fmt, ("—", T.MUTED))
        self.format_label.configure(text=f"Format reçu : {text}", text_color=color)
        self.tel_label.configure(text="\n".join(f"{k:<16s}{v:>10.1f}" for k, v in self.telemetry.items()) or "—")

    # ================================================================== outils carte
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
        """C : démarre la récolte de N tours, ou l'arrête si elle est en cours.
        En pause, on capture tout de suite les tours déjà reçus (image figée)."""
        if self.collection is not None:
            self._end_collection()
            return
        pose = self._pose(self.pose_entry, quiet=False)
        if pose is None:
            return
        target = int(round(self.avg_slider.get()))
        if self.paused:
            if not self.recent:
                self.status("Rien à capturer : aucun scan reçu.")
                return
            col = Collection(target, pose)
            for f in list(self.recent)[-target:]:
                col = col.add(f)
            self.collection = col
            self._end_collection()
            return
        if self.source is None:
            self.status("Connecte le robot (ou lance la démo) avant de capturer.")
            return
        self.drop_preview()
        self.collection = Collection(target, pose)
        self.capture_btn.configure(text="■   Arrêter la récolte     C", fg_color=T.WARN, hover_color="#d4ac0d")
        self._collect_step()

    def _collect_step(self) -> None:
        col = self.collection
        self.collect_bar.set(col.progress)
        self.collect_label.configure(text=f"Récolte : {col.count} / {col.target} tours… ne bouge pas le robot",
                                     text_color=T.WARN)
        if col.done or col.stalled():
            self._end_collection()

    def _end_collection(self) -> None:
        col, self.collection = self.collection, None
        self.capture_btn.configure(text="📸   Capturer (récolter les tours)     C", fg_color=T.ACCENT,
                                   hover_color=T.ACCENT_HOVER)
        self.collect_bar.set(0)
        if col is None or not col.count:
            self.collect_label.configure(text="Aucun tour reçu : récolte annulée.", text_color=T.DANGER)
            return
        scan, info = finish(col, self.combine_mode.get())
        self.preview = (scan, info, tuple(f.raw for f in col.frames), col.frames[-1].fmt)
        self.map.draw_capture(0, self.carto.filters().apply(scan), T.WARN, True, True)
        self.map.draw_live(None, False, [])
        noise = info["bruit_m"]
        quality = ("1 seul tour" if noise is None else "bon" if noise < 0.02 else "moyen : quelque chose bouge ?")
        short = "  ⚠ arrêtée avant la fin" if info["tours"] < info["voulus"] else ""
        self.preview_text.configure(text="\n".join([
            f"Tours récoltés  {info['tours']} / {info['voulus']}{short}",
            f"Durée           {info['duree_s']} s",
            f"Points          {info['points']}  ({info['points_par_tour']} / tour)",
            f"Bruit distances {'—' if noise is None else f'{noise * 100:.1f} cm'}  → {quality}",
            f"Combinaison     {info['mode']}"]))
        self.preview_box.pack(fill="x", pady=(8, 4), after=self.collect_label)
        self.collect_label.configure(text="Garder (Entrée), refaire, ou jeter (Échap).", text_color=T.MUTED)

    def keep_preview(self) -> None:
        if self.preview is None:
            return
        scan, info, raw, fmt = self.preview
        self._clear_preview()
        cap = self._add_capture(scan, source=fmt, raw=raw)
        p = scan.pose
        self.status(f"{cap.title} gardé : {info['points']} points, {info['tours']} tour(s) "
                    f"({info['mode'].lower()}), pose ({p.x:.2f}, {p.y:.2f}, {p.theta_deg:.0f}°).")

    def redo_preview(self) -> None:
        self._clear_preview()
        if self.paused:
            self.toggle_pause()
        self.capture()

    def drop_preview(self) -> None:
        if self.preview is not None:
            self._clear_preview()
            self.status("Capture jetée.")

    def _clear_preview(self) -> None:
        self.preview = None
        self.preview_box.pack_forget()
        self.map.draw_capture(0, None)
        self._redraw_live()

    def _add_capture(self, scan: Scan, label: str = "", source: str = "fichier", raw: tuple = ()) -> Capture:
        n = max((c.number for c in self.captures), default=0) + 1
        cap = Capture(n, scan, T.CAPTURE_COLORS[(n - 1) % len(T.CAPTURE_COLORS)], label, True, now_utc(),
                      source, raw)
        self.captures = self.captures + [cap]
        self._draw_capture(cap)
        self.select(n)
        return cap

    def _draw_capture(self, cap: Capture) -> None:
        self.map.draw_capture(cap.number, self.carto.filters().apply(cap.scan) if cap.visible else None, cap.color,
                              bool(self.carto.show_points.get()), bool(self.carto.show_poses.get()))
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
            sel = cap.number in self.sel
            row = ctk.CTkFrame(self.capture_list, fg_color=T.SURFACE_2, corner_radius=10,
                               border_width=3 if cap.number == self.selected else 2,
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
                w.bind("<Control-Button-1>", lambda e, n=cap.number: self.select(n, additive=True))

    def select(self, number: int, additive: bool = False) -> None:
        cap = self._find(number)
        if cap is None:
            return
        if additive and number in self.sel and len(self.sel) > 1:
            self.sel = self.sel - {number}
            number = self.selected if self.selected in self.sel else min(self.sel)
            cap = self._find(number)
        else:
            self.sel = (self.sel | {number}) if additive else {number}
        self.selected = number
        p = cap.scan.pose
        self.cap_pose.set(p.x, p.y, p.theta_deg)
        extra = f"  + {len(self.sel) - 1} autre(s)" if len(self.sel) > 1 else ""
        self.sel_label.configure(text=f"● {cap.title}{extra}", text_color=cap.color)
        st = scan_stats(cap.scan, self.carto.filters())
        txt = [f"Points           {st['points']}", f"Gardés (filtres) {st['kept']}",
               f"Sans écho        {st['no_echo_pct']:.0f} %"]
        if "nearest" in st:
            mi = st["mean_intensity"]
            txt += [f"Plus proche      {st['nearest'][0]:.2f} m à {st['nearest'][1]:.0f}°",
                    f"Plus loin        {st['max_range']:.2f} m",
                    f"Intensité moy.   {'inconnue' if mi is None else f'{mi:.0f}'}"]
        txt += [f"Source           {cap.source}", f"Tours moyennés   {max(len(cap.raw_tours), 1)}",
                f"Capturé          {cap.captured_utc[11:19] or '—'} UTC"]
        self.cap_stats.configure(text="\n".join(txt))
        self._refresh_capture_list()
        if self.carto.only_selected or self.carto.beams.get():
            self.schedule_analysis()

    def nudge(self, dx: int, dy: int, dtheta: int) -> None:
        """Déplace / tourne toute la sélection (rotation autour de son centre)."""
        if not self.sel:
            self.status("Sélectionne d'abord une capture.")
            return
        step_m, step_deg = NUDGES[self.nudge_mode.get()]
        self._move(set(self.sel), dx * step_m, dy * step_m, dtheta * step_deg, self._pivot(self.sel))

    def apply_pose(self) -> None:
        cap, pose = self._find(self.selected), self._pose(self.cap_pose, quiet=False)
        if cap and pose:
            self._remember()
            self._replace(replace(cap, scan=cap.scan.with_pose(pose)))

    # ------------------------------------------------------------------ souris : déplacer les captures
    def _editor(self) -> Editor:
        return Editor(pick=self._pick, selection=lambda: self.sel, select=self.select, pivot=self._pivot,
                      commit=self._move)

    def toggle_edit(self, on: bool | None = None) -> None:
        on = (self.map.editor is None) if on is None else on
        self.map.set_editor(self._editor() if on else None)
        self.edit_btn.configure(fg_color=T.ACCENT if on else T.SURFACE_2, text_color="#06120b" if on else T.INK)
        if on:
            self.status("Glisse une capture pour la déplacer, clic droit glissé pour la tourner (Ctrl+Z annule).")

    def _tab_changed(self) -> None:
        if self.tabs.get() == "Captures" and self.captures and self.map.editor is None:
            self.toggle_edit(True)

    def _pick(self, x: float, y: float, tol: float) -> int | None:
        best, best_d, click = None, tol, np.array([x, y])
        filters = self.carto.filters()
        for c in self.captures:
            if not c.visible:
                continue
            pts = np.vstack([filters.apply(c.scan).global_points(), [[c.scan.pose.x, c.scan.pose.y]]])
            d = float(np.min(np.hypot(*(pts - click).T)))
            if d < best_d:
                best, best_d = c.number, d
        return best

    def _pivot(self, keys) -> tuple[float, float]:
        poses = [c.scan.pose for c in self.captures if c.number in keys]
        return (float(np.mean([p.x for p in poses])), float(np.mean([p.y for p in poses]))) if poses else (0.0, 0.0)

    def _remember(self) -> None:
        self.history = (self.history + [{c.number: c.scan.pose for c in self.captures}])[-60:]

    def _move(self, keys, dx: float, dy: float, dth: float, pivot) -> None:
        self._remember()
        self._set_poses({c.number: move_rigid(c.scan.pose, dx, dy, dth, pivot)
                         for c in self.captures if c.number in keys})

    def _set_poses(self, poses: dict[int, Pose]) -> None:
        self.captures = [replace(c, scan=c.scan.with_pose(poses[c.number])) if c.number in poses else c
                         for c in self.captures]
        for c in self.captures:
            if c.number in poses:
                self._draw_capture(c)
        cap = self._find(self.selected)
        if cap:
            p = cap.scan.pose
            self.cap_pose.set(p.x, p.y, p.theta_deg)
        self._refresh_capture_list()

    def undo(self) -> None:
        if not self.history:
            self.status("Rien à annuler.")
            return
        last, self.history = self.history[-1], self.history[:-1]
        self._set_poses({n: p for n, p in last.items() if self._find(n)})
        self.status("Déplacement annulé.")

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
            self.sel = self.sel - {number}
            self._refresh_capture_list()
            self.schedule_analysis()

    # ================================================================== cartographie
    def _included(self) -> list[Capture]:
        if self.carto.only_selected:
            cap = self._find(self.selected)
            return [cap] if cap else []
        return [c for c in self.captures if c.visible]

    def _refilter(self) -> None:
        for cap in self.captures:
            self._draw_capture(cap)
        self._redraw_live()

    def schedule_analysis(self) -> None:
        if self._analysis_job:
            self.root.after_cancel(self._analysis_job)
        self._analysis_job = self.root.after(150, self.run_analysis)

    def run_analysis(self) -> None:
        self._analysis_job = None
        params = self.carto.params()
        for cap in self.captures:   # couches points / poses
            self.map.draw_capture(cap.number, params.filters.apply(cap.scan) if cap.visible else None, cap.color,
                                  bool(self.carto.show_points.get()), bool(self.carto.show_poses.get()))
        result = build_maps([c.scan for c in self._included()], params)
        self.last_result = result
        image = result.grid_image(params.grid.model)
        extent = result.grid.extent() if result.grid is not None else None
        sel = self._find(self.selected)
        beams = beam_image(sel.scan, params) if (sel and self.carto.beams.get()) else None
        ref = self.reference if self.carto.show_ref.get() else []
        self.map.draw_analysis(image, extent, result.segments, ref, beams)
        self.carto.show_result(result.stats)

    # ================================================================== import / export
    def import_files(self) -> None:
        paths = filedialog.askopenfilenames(title="Importer", filetypes=[("Données lidar", "*.json *.csv *.txt"),
                                                                          ("Tous", "*.*")])
        added = 0
        for path in paths:
            try:
                if Path(path).suffix.lower() == ".txt" and self._looks_like_vector_map(path):
                    self.reference = self.reference + load_vector_map(path).segments
                    self.status(f"Carte de référence : {len(self.reference)} segment(s) (violet pointillé).")
                    continue
                for label, scan in read_any(path):
                    self._add_capture(scan, label)
                    added += 1
            except (ValueError, OSError, KeyError) as exc:
                messagebox.showerror("Import", f"{Path(path).name} :\n{exc}")
        if added:
            self.status(f"{added} scan(s) importé(s).")
        self.schedule_analysis()

    @staticmethod
    def _looks_like_vector_map(path: str) -> bool:
        lines = [ln for ln in Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
                 if ln.strip() and not ln.startswith("#")]
        return bool(lines) and all(len(ln.split(",")) == 4 for ln in lines[:5])

    def _need_captures(self) -> bool:
        if not self.captures:
            messagebox.showinfo("Export", "Aucune capture. Capture ou importe au moins un scan.")
            return False
        return True

    def open_export(self) -> None:
        if self._need_captures():
            ExportDialog(self.root, len(self._included()), self.selected is not None, self._do_export)

    def _do_export(self, folder: str, options: ExportOptions) -> None:
        caps = self._included() if options.scope != "selection" else self.captures
        if not caps:
            messagebox.showinfo("Export", "Aucune capture incluse (vérifie les 👁 dans l'onglet Captures).")
            return
        try:
            root = export_bundle(folder, caps, self.selected, options, self.carto.params(), self.reference)
        except (ValueError, OSError) as exc:
            messagebox.showerror("Export", str(exc))
            return
        self.status(f"Export terminé : {root}")
        messagebox.showinfo("Export terminé", f"Dossier créé :\n{root}\n\nOuvre LISEZMOI.md pour le détail.")

    def export_session(self) -> None:
        if not self._need_captures():
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="session_salle.lidar.json",
                                            filetypes=[("Session LidarScope", "*.json")])
        if path:
            extra = {"map_params": self.carto.params().as_dict(),
                     "map_stats": self.last_result.stats if self.last_result else {}}
            save_session(self.captures, path, extra)
            self.status(f"Session enregistrée : {path}")

    def export_png(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".png", initialfile="carte_ecran.png",
                                            filetypes=[("Image PNG", "*.png")])
        if path:
            self.map.fig.savefig(path, dpi=200, facecolor=T.BG)
            self.status(f"Image enregistrée : {path}")

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
