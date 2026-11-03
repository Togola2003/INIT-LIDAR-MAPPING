"""Étape 1 · Récolter : relier le robot, placer sa pose, récolter N tours, garder ou refaire. Ou importer."""

from __future__ import annotations

import customtkinter as ctk

from ...geometry import Pose
from ...scope.collector import COMBINE_MODES, Collection, finish
from ...scope.sources import FORMAT_TELEPLOT
from .. import project as P
from .. import style as S
from ..tools import PoseTool
from .base import Step

BAUDS = ["115200", "230400", "460800", "921600"]
SOURCES = ["Câble USB", "Wi-Fi", "Démo"]


class CollectStep(Step):
    title = "Récolter"
    subtitle = "robot, captures, import"
    headline = "Le scan en direct apparaît en noir à la pose du robot ; les captures gardées en couleur."
    course = ("Un scan = un tour du lidar (≈ 450 points). Le robot doit rester immobile pendant la capture : "
              "on récolte plusieurs tours au même endroit, puis on les combine pour réduire le bruit. "
              "La pose (x, y, θ) place le scan dans la carte : P_carte = R(θ)·P_lidar + (x, y).")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.robot = Pose()
        self.collection: Collection | None = None
        self.preview = None

    # ------------------------------------------------------------------ panneau
    def build(self, panel) -> None:
        card = S.Card(panel, "Robot", "Relie l'ESP32 par câble USB, ou essaie la démo sans robot.")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.source = S.segmented(b, SOURCES, SOURCES[0], lambda v: self._source_changed())
        self.source.pack(fill="x", pady=(0, 8))
        row = ctk.CTkFrame(b, fg_color="transparent")
        row.pack(fill="x")
        self.port = ctk.CTkOptionMenu(row, values=["—"], width=110, height=30, font=S.font(12), fg_color=S.SURFACE_2,
                                      button_color=S.BORDER, button_hover_color=S.FAINT, text_color=S.INK,
                                      dropdown_font=S.font(12))
        self.port.pack(side="left")
        S.button(row, "↻", self.refresh_ports, width=34).pack(side="left", padx=6)
        self.baud = ctk.CTkOptionMenu(row, values=BAUDS, width=100, height=30, font=S.font(12), fg_color=S.SURFACE_2,
                                      button_color=S.BORDER, button_hover_color=S.FAINT, text_color=S.INK)
        self.baud.pack(side="left")
        self.connect_btn = S.button(b, "Connecter", self.toggle_connect, "primary")
        self.connect_btn.pack(fill="x", pady=(10, 2))
        self.source_hint = S.hint(b, "")
        self.live_info = S.KeyValues(b)
        self.live_info.pack(fill="x", pady=(4, 0))

        card = S.Card(panel, "Capture", "C = récolter · C de nouveau = arrêter · Entrée = garder · Échap = jeter")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.tours = S.Slider(b, "Tours à récolter", 1, 30, 10, "{:.0f}", lambda v: None, steps=29)
        self.tours.pack(fill="x")
        self.combine = S.segmented(b, list(COMBINE_MODES), COMBINE_MODES[0], lambda v: None)
        self.combine.pack(fill="x", pady=(2, 6))
        S.hint(b, "Médiane : un point par angle, bruit réduit. Tous : nuage plus dense.")
        self.capture_btn = S.button(b, "Capturer", self.capture, "primary", height=42, font=S.font(14, "bold"))
        self.capture_btn.pack(fill="x", pady=(6, 4))
        self.bar = ctk.CTkProgressBar(b, height=6, progress_color=S.ACCENT, fg_color=S.SURFACE_2, corner_radius=3)
        self.bar.set(0)
        self.bar.pack(fill="x", pady=(2, 4))
        self.preview_box = ctk.CTkFrame(b, fg_color="#FFF6E5", corner_radius=12)
        ctk.CTkLabel(self.preview_box, text="Aperçu (orange sur la carte)", font=S.font(12, "bold"),
                     text_color=S.WARN).pack(anchor="w", padx=12, pady=(10, 0))
        self.preview_info = S.KeyValues(self.preview_box)
        self.preview_info.pack(fill="x", padx=12, pady=4)
        row = ctk.CTkFrame(self.preview_box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 10))
        S.button(row, "Garder", self.keep, "primary", width=90).pack(side="left", padx=3)
        S.button(row, "Refaire", self.redo, width=80).pack(side="left", padx=3)
        S.button(row, "Jeter", self.drop, "danger", width=70).pack(side="left", padx=3)

        card = S.Card(panel, "Pose du robot", "Clique sur la carte pour le placer, glisse pour l'orienter.")
        card.pack(fill="x", pady=(0, 10))
        self.pose_field = S.PoseField(card.body)
        self.pose_field.pack(anchor="w")
        S.button(card.body, "Appliquer", self.apply_pose_field, "ghost", width=90).pack(anchor="e", pady=(6, 0))

        card = S.Card(panel, "Importer", "Session .lidar.json, CSV du TP, RobotControl, MapTool…")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Ouvrir des scans…", self.app.open_files).pack(fill="x")
        self.refresh_ports()
        self._source_changed()

    # ------------------------------------------------------------------ connexion
    def _source_changed(self) -> None:
        src = self.source.get()
        if src == "Wi-Fi":
            self.source_hint.configure(text="Pilotage et scans par Wi-Fi (RobotControl) : prévu au Jour 3.",
                                       text_color=S.WARN)
            self.connect_btn.configure(state="disabled")
        else:
            self.source_hint.configure(text="Démo : une salle simulée, sans robot." if src == "Démo" else
                                       "Ferme le moniteur série d'Arduino avant de connecter.", text_color=S.MUTED)
            self.connect_btn.configure(state="normal")
        for w in (self.port, self.baud):
            w.configure(state="normal" if src == "Câble USB" else "disabled")

    def refresh_ports(self) -> None:
        ports = self.app.ports() or ["—"]
        self.port.configure(values=ports)
        if self.port.get() not in ports:
            self.port.set(ports[0])

    def toggle_connect(self) -> None:
        if self.app.source is not None:
            self.app.disconnect()
            self.connect_btn.configure(text="Connecter")
            return
        if self.source.get() == "Démo":
            self.app.demo()
        elif self.port.get() == "—":
            self.app.status("Aucun port série : branche l'ESP32 puis ↻.")
            return
        else:
            self.app.connect(self.port.get(), int(self.baud.get()))
        self.connect_btn.configure(text="Déconnecter")

    # ------------------------------------------------------------------ pose
    def enter(self) -> None:
        self.app.set_tool(PoseTool(self.app.map, self.set_robot, S.ROBOT))

    def set_robot(self, pose: Pose) -> None:
        self.robot = pose
        self.pose_field.set(pose.x, pose.y, pose.theta_deg)
        self.render()

    def apply_pose_field(self) -> None:
        try:
            self.set_robot(Pose(*self.pose_field.get()))
        except ValueError:
            self.app.status("x, y et θ doivent être des nombres.")

    # ------------------------------------------------------------------ récolte
    def capture(self) -> None:
        if self.collection is not None:
            self._finish()
            return
        if self.app.source is None:
            self.app.status("Connecte le robot (ou la démo) pour capturer, ou importe des scans.")
            return
        self.drop()
        self.collection = Collection(int(round(self.tours.get())), self.robot)
        self.capture_btn.configure(text="Arrêter la récolte", fg_color=S.WARN, hover_color="#E68600")

    def on_frame(self, frame) -> None:
        if self.collection is not None:
            self.collection = self.collection.add(frame)
            self.bar.set(self.collection.progress)
            self.app.status(f"Récolte : {self.collection.count} / {self.collection.target} tours… ne bouge pas le robot.")
            if self.collection.done:
                self._finish()

    def _finish(self) -> None:
        col, self.collection = self.collection, None
        self.capture_btn.configure(text="Capturer", fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER)
        self.bar.set(0)
        if col is None or not col.count:
            self.app.status("Aucun tour reçu : récolte annulée.")
            return
        scan, info = finish(col, self.combine.get())
        self.preview = (scan, info, tuple(f.raw for f in col.frames), col.frames[-1].fmt)
        noise = info["bruit_m"]
        quality = "—" if noise is None else ("bon" if noise < 0.02 else "moyen")
        self.preview_info.show([("Tours", f"{info['tours']} / {info['voulus']}"), ("Durée", f"{info['duree_s']} s"),
                                ("Points", f"{info['points']}"),
                                ("Bruit", "—" if noise is None else f"{noise * 100:.1f} cm ({quality})")])
        self.preview_box.pack(fill="x", pady=(6, 0))
        self.app.status("Aperçu prêt : Garder (Entrée), Refaire, ou Jeter (Échap).")
        self.render()

    def keep(self) -> None:
        if self.preview is None:
            return
        scan, info, raw, fmt = self.preview
        self.drop(quiet=True)
        new, nums = P.add_scans(self.project, [("", scan)], source=fmt, raw=raw)
        self.app.sel, self.app.primary = {nums[0]}, nums[0]
        self.app.apply(new, f"Scan {nums[0]} gardé : {info['points']} points, {info['tours']} tours.")

    def redo(self) -> None:
        self.drop(quiet=True)
        self.capture()

    def drop(self, quiet: bool = False) -> None:
        if self.preview is not None:
            self.preview = None
            self.preview_box.pack_forget()
            if not quiet:
                self.app.status("Capture jetée.")
            self.render()

    def on_key(self, key: str) -> None:
        k = key.lower()
        if k == "c":
            self.capture()
        elif k == "return":
            self.keep()
        elif k == "escape":
            self.drop()

    def reset(self) -> None:
        self.drop(quiet=True)
        self.collection = None

    # ------------------------------------------------------------------ carte
    def on_live(self, frame) -> None:
        f = self.app.last_frame
        if f is None:
            return
        fmt = "intensité ✓" if f.fmt != FORMAT_TELEPLOT else "sans intensité (Teleplot)"
        self.live_info.show([("Points du tour", str(int((f.scan.ranges_m > 0).sum()))), ("Format", fmt)])
        self.render()

    def render(self) -> None:
        app = self.app
        pts, poses = app.scan_layers(big=False)
        app.map.draw_overlay(pts, [], poses)
        app.map.draw_analysis(None, None, [], [], None)
        app.map.draw_robot(self.robot)
        frame = app.last_frame
        live = None
        if frame is not None and self.preview is None:
            live = app.filters().apply(frame.scan).with_pose(self.robot)
        app.map.draw_live(live, False, [])
        if self.preview is not None:
            app.map.draw_capture(0, app.filters().apply(self.preview[0]), S.INITIAL, True, True)
        else:
            app.map.draw_capture(0, None)
        self.legend("● noir = direct · ● couleurs = scans gardés · ● orange = aperçu · clic = placer le robot")

    def leave(self) -> None:
        self.app.map.draw_live(None, False, [])
        self.app.map.draw_capture(0, None)
        self.app.map.robot_dot.set_data([], [])
        self.app.map.robot_line.set_data([], [])
