"""LidarStudio : cartographie hors ligne, pas à pas, en suivant les étapes du cours.

    .venv/Scripts/python -m lidar.studio [session.lidar.json]      (ou double-clic sur LidarStudio.bat)

Étapes : 1 Données · 2 Prétraitement · 3 Recalage · 4 Grid map · 5 Carte vectorielle · 6 Résultat

Déplacer les scans (bouton ✋ actif) : clic = sélectionner, Ctrl + clic = ajouter à la sélection,
glisser = déplacer, clic droit glissé = tourner autour du centre de la sélection, Ctrl + molette = 1°.
Clavier : flèches = déplacer, A / E = tourner, Ctrl+A = tout sélectionner, Ctrl+Z = annuler.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk
import numpy as np

from ..extraction import ExtractionParams, extraction_stages
from ..geometry import Pose, move_rigid
from ..matching import align_best, align_sequence, consistency, icp, refine_global, voxel_downsample
from ..scan import Scan
from ..segments import Segment
from ..vectormap import load_vector_map
from ..scope import theme as T
from ..scope.carto_panel import RESOLUTIONS
from ..scope.export_dialog import ExportDialog
from ..scope.exporter import export_bundle
from ..scope.mapping import GridParams, MapParams, VectorParams, beam_image, build_maps
from ..scope.mapview import Editor, MapView
from ..scope.processing import Filters
from ..scope.render import render_png
from ..scope.session import Capture, read_any, save_session
from ..scope.widgets import LabeledSlider, PoseEntry, button, font, section
from .steps import STEPS, VECTOR_STAGES

PALETTE = ["#2ecc71", "#e67e22", "#3498db", "#e84393", "#f1c40f", "#9b59b6", "#1abc9c", "#e5534b",
           "#00d2ff", "#ff9ff3", "#c8d6e5", "#feca57"]
NUDGES = {"Fin": (0.01, 0.5), "Normal": (0.05, 2.0), "Gros": (0.20, 5.0)}
LOW_OVERLAP = 0.30   # en dessous, le recalage ICP est jugé peu fiable
HISTORY = 60         # nombre de retours en arrière possibles (Ctrl+Z)


def _seg_button(master, values, value, command) -> ctk.CTkSegmentedButton:
    s = ctk.CTkSegmentedButton(master, values=values, command=lambda _: command(), selected_color=T.ACCENT,
                               selected_hover_color=T.ACCENT_HOVER, unselected_color=T.SURFACE)
    s.set(value)
    return s


def _switch(master, text, command, on=False) -> ctk.CTkSwitch:
    sw = ctk.CTkSwitch(master, text=text, font=font(12), progress_color=T.ACCENT, command=command)
    if on:
        sw.select()
    return sw


class LidarStudio:
    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.captures: list[Capture] = []
        self.originals: dict[int, Pose] = {}
        self.icp_info: dict[int, str] = {}
        self.reference: list[Segment] = []
        self.selected: int | None = None      # scan « principal » (ICP, étapes de la carte vectorielle)
        self.sel: set[int] = set()            # tous les scans sélectionnés (déplacés / tournés ensemble)
        self.history: list[dict[int, Pose]] = []
        self.step = 0
        self._job = None

        root.title("LidarStudio · cartographie pas à pas")
        root.geometry("1560x920")
        root.minsize(1250, 760)
        root.configure(fg_color=T.BG)
        self._build_topbar()
        body = ctk.CTkFrame(root, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=12, pady=(0, 8))
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)
        self._build_rail(body)
        self._build_map(body)
        self._build_panels(body)
        self.status_var = ctk.StringVar(value="Ouvre une session (.lidar.json) ou des scans CSV pour commencer.")
        ctk.CTkLabel(root, textvariable=self.status_var, font=font(11), text_color=T.MUTED, anchor="w").pack(
            fill="x", padx=16, pady=(0, 8))
        root.protocol("WM_DELETE_WINDOW", self.quit)
        keys = {"<Left>": (-1, 0, 0), "<Right>": (1, 0, 0), "<Up>": (0, 1, 0), "<Down>": (0, -1, 0),
                "a": (0, 0, 1), "e": (0, 0, -1)}
        for key, d in keys.items():
            root.bind(key, lambda ev, d=d: None if self._typing(ev) else self.nudge(*d))
        root.bind("<Control-z>", lambda ev: self.undo())
        root.bind("<Control-a>", lambda ev: None if self._typing(ev) else self.select_all())
        self.map.set_editor(self._editor())
        self.go(0)

    # ================================================================== structure
    def _build_topbar(self) -> None:
        bar = ctk.CTkFrame(self.root, fg_color=T.SURFACE, corner_radius=0, height=58)
        bar.pack(fill="x", pady=(0, 12))
        box = ctk.CTkFrame(bar, fg_color="transparent")
        box.pack(side="left", padx=16, pady=8)
        ctk.CTkLabel(box, text="◆ LidarStudio", font=font(18, "bold"), text_color=T.ACCENT).pack(anchor="w")
        ctk.CTkLabel(box, text="cartographie pas à pas · grid map & carte vectorielle", font=font(10),
                     text_color=T.MUTED).pack(anchor="w")
        button(bar, "📂  Ouvrir…", self.open_files).pack(side="left", padx=(24, 6))
        button(bar, "💾  Enregistrer le projet", self.save_project).pack(side="left", padx=6)
        button(bar, "📦  Exporter…", self.open_export, primary=True).pack(side="left", padx=6)
        self.count_label = ctk.CTkLabel(bar, text="", font=font(12), text_color=T.MUTED)
        self.count_label.pack(side="right", padx=16)

    def _build_rail(self, parent) -> None:
        rail = ctk.CTkFrame(parent, fg_color=T.SURFACE, corner_radius=12, width=200)
        rail.grid(row=0, column=0, sticky="ns", padx=(0, 12))
        rail.grid_propagate(False)
        ctk.CTkLabel(rail, text="ÉTAPES DU COURS", font=font(10, "bold"), text_color=T.MUTED).pack(
            anchor="w", padx=14, pady=(14, 8))
        self.rail_items = []
        for k, (num, title, sub, _) in enumerate(STEPS):
            item = ctk.CTkFrame(rail, fg_color="transparent", corner_radius=10)
            item.pack(fill="x", padx=8, pady=3)
            badge = ctk.CTkLabel(item, text=num, width=28, height=28, corner_radius=14, fg_color=T.SURFACE_2,
                                 font=font(12, "bold"), text_color=T.INK)
            badge.pack(side="left", padx=(8, 8), pady=8)
            txt = ctk.CTkFrame(item, fg_color="transparent")
            txt.pack(side="left", fill="x")
            t1 = ctk.CTkLabel(txt, text=title, font=font(13, "bold"), text_color=T.INK, anchor="w")
            t1.pack(anchor="w")
            t2 = ctk.CTkLabel(txt, text=sub, font=font(10), text_color=T.MUTED, anchor="w")
            t2.pack(anchor="w")
            for w in (item, badge, txt, t1, t2):
                w.bind("<Button-1>", lambda e, k=k: self.go(k))
            self.rail_items.append((item, badge))
        nav = ctk.CTkFrame(rail, fg_color="transparent")
        nav.pack(side="bottom", fill="x", padx=10, pady=12)
        button(nav, "◀", lambda: self.go(self.step - 1), width=50).pack(side="left")
        button(nav, "Suivant ▶", lambda: self.go(self.step + 1), primary=True).pack(side="right")

    def _build_map(self, parent) -> None:
        card = ctk.CTkFrame(parent, fg_color=T.SURFACE, corner_radius=12)
        card.grid(row=0, column=1, sticky="nsew", padx=(0, 12))
        tools = ctk.CTkFrame(card, fg_color="transparent")
        tools.pack(fill="x", padx=10, pady=(10, 4))
        self.view_title = ctk.CTkLabel(tools, text="", font=font(14, "bold"), text_color=T.INK)
        self.view_title.pack(side="left", padx=4)
        button(tools, "⌖  Tout voir", self.fit_view, width=100).pack(side="right")
        self.ruler_btn = button(tools, "📏  Règle", self.toggle_ruler, width=90)
        self.ruler_btn.pack(side="right", padx=6)
        button(tools, "↶  Annuler", self.undo, width=90).pack(side="right")
        self.edit_btn = button(tools, "✋  Déplacer les scans : oui", self.toggle_edit, width=190,
                               fg_color=T.ACCENT, text_color="#06120b", hover_color=T.ACCENT_HOVER)
        self.edit_btn.pack(side="right", padx=6)
        holder = ctk.CTkFrame(card, fg_color=T.BG, corner_radius=8)
        holder.pack(fill="both", expand=True, padx=10)
        self.cursor_var = ctk.StringVar(value="")
        self.map = MapView(holder, on_cursor=self.cursor_var.set)
        self.map.widget.pack(fill="both", expand=True, padx=2, pady=2)
        bottom = ctk.CTkFrame(card, fg_color="transparent")
        bottom.pack(fill="x", padx=14, pady=(4, 8))
        ctk.CTkLabel(bottom, textvariable=self.cursor_var, font=font(11, mono=True), text_color=T.MUTED).pack(side="left")
        ctk.CTkLabel(card, text="✋ glisser un scan = déplacer · clic droit glissé = tourner · Ctrl+clic = sélection "
                     "multiple · Ctrl+molette = ±1° · flèches / A E au clavier · Ctrl+Z = annuler",
                     font=font(10), text_color=T.MUTED).pack(anchor="w", padx=14, pady=(0, 6))
        self.legend = ctk.CTkLabel(bottom, text="", font=font(11), text_color=T.MUTED)
        self.legend.pack(side="right")

    def _build_panels(self, parent) -> None:
        side = ctk.CTkFrame(parent, fg_color=T.SURFACE, corner_radius=12, width=410)
        side.grid(row=0, column=2, sticky="ns")
        side.grid_propagate(False)
        head = ctk.CTkFrame(side, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(14, 0))
        self.panel_title = ctk.CTkLabel(head, text="", font=font(17, "bold"), text_color=T.INK, anchor="w")
        self.panel_title.pack(anchor="w")
        course = ctk.CTkFrame(side, fg_color=T.SURFACE_2, corner_radius=10, border_width=1, border_color=T.BORDER)
        course.pack(fill="x", padx=14, pady=10)
        ctk.CTkLabel(course, text="📘  CE QUE DIT LE COURS", font=font(10, "bold"), text_color=T.ACCENT).pack(
            anchor="w", padx=12, pady=(8, 0))
        self.course_text = ctk.CTkLabel(course, text="", font=font(11), text_color=T.INK, justify="left",
                                        wraplength=360, anchor="w")
        self.course_text.pack(anchor="w", padx=12, pady=(2, 10))
        self.panels = []
        for builder in (self._panel_data, self._panel_filters, self._panel_align, self._panel_grid,
                        self._panel_vector, self._panel_result):
            frame = ctk.CTkScrollableFrame(side, fg_color="transparent")
            builder(frame)
            self.panels.append(frame)

    # ------------------------------------------------------------------ panneaux
    def _panel_data(self, p) -> None:
        row = ctk.CTkFrame(p, fg_color="transparent")
        row.pack(fill="x", pady=(0, 6))
        button(row, "Tout cocher", lambda: self._set_all(True), width=100).pack(side="left")
        button(row, "Tout décocher", lambda: self._set_all(False), width=100).pack(side="left", padx=6)
        button(row, "Tout sélect.", self.select_all, width=100).pack(side="left")
        ctk.CTkLabel(p, text="Clic = sélectionner · Ctrl + clic = ajouter à la sélection (pour déplacer / tourner "
                     "plusieurs scans ensemble)", font=font(10), text_color=T.MUTED, wraplength=360,
                     justify="left").pack(anchor="w", pady=(0, 6))
        self.scan_list = ctk.CTkFrame(p, fg_color="transparent")
        self.scan_list.pack(fill="x")

    def _panel_filters(self, p) -> None:
        box = section(p, "Portée et intensité")
        self.f_min = LabeledSlider(box, "Portée minimale", 0, 1, 0.15, "{:.2f} m", self._later)
        self.f_max = LabeledSlider(box, "Portée maximale", 1, 12, 10, "{:.1f} m", self._later)
        self.f_int = LabeledSlider(box, "Intensité minimale", 0, 255, 0, "{:.0f}", self._later, steps=255)
        for s in (self.f_min, self.f_max, self.f_int):
            s.pack(fill="x", padx=12, pady=5)
        self.int_note = ctk.CTkLabel(box, text="", font=font(10), text_color=T.WARN, wraplength=340, justify="left")
        self.int_note.pack(anchor="w", padx=12, pady=(0, 8))
        box = section(p, "Voisinage (points isolés)")
        self.f_nb = LabeledSlider(box, "Voisins minimum", 0, 6, 2, "{:.0f}", self._later, steps=6)
        self.f_rad = LabeledSlider(box, "Rayon de recherche à 0 m", 0.02, 0.30, 0.10, "{:.2f} m", self._later)
        for s in (self.f_nb, self.f_rad):
            s.pack(fill="x", padx=12, pady=5)
        ctk.CTkLabel(box, text="Le rayon grandit de 4 cm par mètre de distance (les points d'un mur lointain sont "
                     "plus espacés). 0 voisin = filtre désactivé.", font=font(10), text_color=T.MUTED,
                     wraplength=340, justify="left").pack(anchor="w", padx=12, pady=(0, 8))
        box = section(p, "Bilan")
        self.filter_stats = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.filter_stats.pack(anchor="w", padx=12, pady=8)

    def _panel_align(self, p) -> None:
        box = section(p, "Sélection (déplacée / tournée en bloc)")
        self.sel_label = ctk.CTkLabel(box, text="Clique un scan sur la carte", font=font(13, "bold"),
                                      text_color=T.MUTED, wraplength=360)
        self.sel_label.pack(pady=(10, 4))
        rowb = ctk.CTkFrame(box, fg_color="transparent")
        rowb.pack(pady=(0, 4))
        button(rowb, "Tout sélectionner", self.select_all, width=140).pack(side="left", padx=3)
        button(rowb, "Seulement le principal", self.select_primary_only, width=160).pack(side="left", padx=3)
        self.nudge_mode = _seg_button(box, list(NUDGES), "Normal", lambda: None)
        self.nudge_mode.pack(pady=4)
        pad = ctk.CTkFrame(box, fg_color="transparent")
        pad.pack(pady=4)
        for (r, c, txt, d) in ((0, 1, "↑", (0, 1, 0)), (1, 0, "←", (-1, 0, 0)), (1, 2, "→", (1, 0, 0)),
                               (2, 1, "↓", (0, -1, 0)), (0, 0, "⟲", (0, 0, 1)), (0, 2, "⟳", (0, 0, -1))):
            button(pad, txt, lambda d=d: self.nudge(*d), width=44, height=34, font=font(16)).grid(row=r, column=c, padx=3, pady=3)
        self.pose_entry = PoseEntry(box)
        self.pose_entry.pack(padx=10, pady=6)
        button(box, "Appliquer cette pose", self.apply_pose).pack(fill="x", padx=12, pady=(0, 10))
        box = section(p, "Recalage automatique")
        button(box, "⚡  Recalage complet (le meilleur des deux)", self.align_full, primary=True).pack(
            fill="x", padx=12, pady=(12, 2))
        ctk.CTkLabel(box, text="Toutes les poses corrigées ENSEMBLE (ICP point à droite), en partant soit des poses "
                     "actuelles, soit de l'ICP scan par scan : on garde le départ qui donne la carte la plus "
                     "cohérente. Ancre = scan principal.", font=font(10),
                     text_color=T.MUTED, wraplength=350, justify="left").pack(anchor="w", padx=12, pady=(0, 6))
        button(box, "🎯  Recaler la sélection (en bloc) sur les autres", self.icp_selected).pack(
            fill="x", padx=12, pady=4)
        button(box, "🧩  ICP scan par scan (ancre = principal)", self.icp_all).pack(fill="x", padx=12, pady=4)
        button(box, "🚀  Optimisation globale seule", self.optimize_global).pack(fill="x", padx=12, pady=4)
        button(box, "↺  Revenir aux poses d'origine", self.reset_poses).pack(fill="x", padx=12, pady=(4, 6))
        button(box, "📏  Mesurer la cohérence des scans", self.measure).pack(fill="x", padx=12, pady=(0, 12))
        box = section(p, "Résultats")
        self.align_text = ctk.CTkLabel(box, text="—", font=font(10, mono=True), text_color=T.INK, justify="left")
        self.align_text.pack(anchor="w", padx=12, pady=8)

    def _panel_grid(self, p) -> None:
        box = section(p, "① Grille")
        rows = (("Résolution", "g_res", list(RESOLUTIONS), "5 cm"),
                ("État initial", "g_init", ["inconnu", "libre", "occupé"], "inconnu"),
                ("Modèle", "g_model", ["probabiliste", "binaire"], "probabiliste"))
        for label, attr, values, val in rows:
            ctk.CTkLabel(box, text=label, font=font(11), text_color=T.MUTED).pack(anchor="w", padx=12, pady=(6, 0))
            w = _seg_button(box, values, val, self._later_now)
            w.pack(anchor="w", padx=12, pady=(0, 4))
            setattr(self, attr, w)
        box = section(p, "③ ④ Mise à jour des cellules")
        self.g_pocc = LabeledSlider(box, "p(occupé | impact)", 0.55, 0.95, 0.70, "{:.2f}", self._later)
        self.g_pfree = LabeledSlider(box, "p(occupé | faisceau traverse)", 0.05, 0.45, 0.40, "{:.2f}", self._later)
        for s in (self.g_pocc, self.g_pfree):
            s.pack(fill="x", padx=12, pady=5)
        self.g_beams = _switch(box, "Cellules du scan ajouté (bleu traversées, rouge impact)", self._later_now, on=True)
        self.g_beams.pack(anchor="w", padx=12, pady=(4, 10))
        box = section(p, "⑤ Mise à jour scan après scan + nettoyage")
        self.g_step = LabeledSlider(box, "Scans intégrés", 1, 2, 2, "{:.0f}", self._later, steps=1)
        self.g_step.pack(fill="x", padx=12, pady=(10, 4))
        self.g_iso = _switch(box, "Supprimer les cellules occupées isolées", self._later_now)
        self.g_iso.pack(anchor="w", padx=12, pady=(4, 6))
        self.g_points = _switch(box, "Afficher les points", self._later_now)
        self.g_points.pack(anchor="w", padx=12, pady=(0, 10))
        box = section(p, "Bilan")
        self.grid_stats = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.grid_stats.pack(anchor="w", padx=12, pady=8)

    def _panel_vector(self, p) -> None:
        box = section(p, "Étape affichée")
        self.v_stage = _seg_button(box, VECTOR_STAGES, "Clusters", self._later_now)
        self.v_stage.pack(padx=8, pady=10)
        ctk.CTkLabel(box, text="Étapes 1 à 4 : le scan sélectionné. « Carte » : tous les scans cochés.",
                     font=font(10), text_color=T.MUTED, wraplength=340).pack(anchor="w", padx=12, pady=(0, 8))
        box = section(p, "Réglages")
        self.v_jump = LabeledSlider(box, "① Saut qui sépare deux objets", 0.02, 0.40, 0.08, "{:.2f} m", self._later)
        self.v_split = LabeledSlider(box, "② Tolérance point-droite", 0.01, 0.12, 0.04, "{:.2f} m", self._later)
        self.v_minpts = LabeledSlider(box, "Points minimum par segment", 3, 20, 6, "{:.0f}", self._later, steps=17)
        self.v_len = LabeledSlider(box, "Longueur minimale", 0.05, 1.0, 0.20, "{:.2f} m", self._later)
        self.v_angle = LabeledSlider(box, "④ Fusion si angle ≤", 2, 25, 10, "{:.0f}°", self._later)
        for s in (self.v_jump, self.v_split, self.v_minpts, self.v_len, self.v_angle):
            s.pack(fill="x", padx=12, pady=5)
        box = section(p, "Bilan")
        self.vec_stats = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.vec_stats.pack(anchor="w", padx=12, pady=8)

    def _panel_result(self, p) -> None:
        box = section(p, "Couches")
        self.r_grid = _switch(box, "Grid map", self._later_now, on=True)
        self.r_seg = _switch(box, "Carte vectorielle (segments)", self._later_now, on=True)
        self.r_pts = _switch(box, "Points des scans", self._later_now)
        self.r_poses = _switch(box, "Poses du robot", self._later_now, on=True)
        self.r_ref = _switch(box, "Carte de référence (MapTool)", self._later_now, on=True)
        for sw in (self.r_grid, self.r_seg, self.r_pts, self.r_poses, self.r_ref):
            sw.pack(anchor="w", padx=12, pady=4)
        button(box, "Importer une carte de référence (.txt)…", self.import_reference).pack(fill="x", padx=12, pady=10)
        box = section(p, "Sorties")
        button(box, "📦  Exporter… (chaque scan / ensemble)", self.open_export, primary=True).pack(fill="x", padx=12, pady=(12, 4))
        button(box, "🖼  Image de la carte (fond blanc)", self.export_png).pack(fill="x", padx=12, pady=4)
        button(box, "💾  Enregistrer le projet (poses corrigées)", self.save_project).pack(fill="x", padx=12, pady=(4, 12))
        box = section(p, "Bilan de la carte")
        self.result_stats = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.result_stats.pack(anchor="w", padx=12, pady=8)

    # ================================================================== navigation et rendu
    def go(self, k: int) -> None:
        self.step = max(0, min(len(STEPS) - 1, k))
        for i, (item, badge) in enumerate(self.rail_items):
            active = i == self.step
            item.configure(fg_color=T.SURFACE_2 if active else "transparent")
            badge.configure(fg_color=T.ACCENT if active else T.SURFACE_2, text_color="#06120b" if active else T.INK)
        for i, frame in enumerate(self.panels):
            if i == self.step:
                frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
            else:
                frame.pack_forget()
        num, title, sub, text = STEPS[self.step]
        self.panel_title.configure(text=f"{num}. {title}")
        self.course_text.configure(text=text)
        self.view_title.configure(text=f"Étape {num} · {sub}")
        self.render()

    def _later(self, _v=None) -> None:
        if self._job:
            self.root.after_cancel(self._job)
        self._job = self.root.after(250, self.render)

    def _later_now(self) -> None:
        self._later()

    def render(self) -> None:
        self._job = None
        if not self.captures:
            self.map.draw_overlay([], [])
            self.map.draw_analysis(None, None, [], [], None)
            return
        [self._render_data, self._render_filters, self._render_align, self._render_grid,
         self._render_vector, self._render_result][self.step]()

    # ------------------------------------------------------------------ réglages
    def filters(self) -> Filters:
        lo = self.f_min.get()
        return Filters(lo, max(self.f_max.get(), lo + 0.05), int(self.f_int.get()), int(round(self.f_nb.get())),
                       self.f_rad.get())

    def params(self, grid=True, vector=True) -> MapParams:
        return MapParams(
            filters=self.filters(),
            grid=GridParams(grid, RESOLUTIONS[self.g_res.get()], self.g_init.get(), self.g_model.get(),
                            round(self.g_pocc.get(), 3), round(self.g_pfree.get(), 3), bool(self.g_iso.get())),
            vector=VectorParams(vector, round(self.v_jump.get(), 3), round(self.v_split.get(), 3),
                                round(self.v_len.get(), 3), round(self.v_angle.get(), 1)),
        )

    def extraction_params(self) -> ExtractionParams:
        return ExtractionParams(jump_base_m=self.v_jump.get(), split_tol_m=self.v_split.get(),
                                min_points=int(round(self.v_minpts.get())), min_length_m=self.v_len.get(),
                                merge_angle_deg=self.v_angle.get())

    def included(self) -> list[Capture]:
        return [c for c in self.captures if c.visible]

    def _pts(self, cap: Capture) -> np.ndarray:
        return self.filters().apply(cap.scan).global_points()

    def _local(self, cap: Capture) -> np.ndarray:
        return self.filters().apply(cap.scan).local_points()

    def _poses(self, caps) -> list[tuple]:
        return [(c.scan.pose, c.color, c.number in self.sel, c.number) for c in caps]

    # ------------------------------------------------------------------ rendus par étape
    def _render_data(self) -> None:
        caps = self.included()
        self.map.draw_analysis(None, None, [], [], None)
        self.map.draw_overlay([(c.scan.filtered().global_points(), c.color, 9 if c.number in self.sel else 4, "o",
                                c.number) for c in caps], [], self._poses(caps))
        self.legend.configure(text="une couleur par scan · ▲ = pose du robot (contour blanc = sélection)")

    def _render_filters(self) -> None:
        caps, f = self.included(), self.filters()
        kept, rej_range, rej_nb = [], [], []
        for c in caps:
            raw = c.scan.filtered(0.0001, 1e9, 0)                    # tous les points avec écho
            k = f.apply(c.scan)
            kept.append((k.global_points(), c.color, 4, "o", c.number))
            out_range = raw.ranges_m < f.min_range_m
            out_range |= raw.ranges_m > f.max_range_m
            out_range |= raw.intensities < f.min_intensity          # NaN -> False
            rej_range.append(raw.global_points()[out_range])
            if f.min_neighbors > 0:
                rej_nb.append(f.rejected(c.scan).global_points())
        rr = np.vstack(rej_range) if rej_range else np.empty((0, 2))
        rn = np.vstack(rej_nb) if rej_nb else np.empty((0, 2))
        self.map.draw_analysis(None, None, [], [], None)
        self.map.draw_overlay(kept + [(rr, T.DANGER, 22, "x"), (rn, "#ff9f43", 22, "x")], [], self._poses(caps))
        total = sum(len(c.scan.filtered(0.0001, 1e9, 0)) for c in caps)
        nkept = sum(len(k[0]) for k in kept)
        no_int = all(np.isnan(c.scan.intensities).all() for c in caps)
        self.int_note.configure(text="Intensité non transmise par ces scans (Teleplot) : ce curseur n'a pas "
                                     "d'effet. Utilise le filtre de voisinage." if no_int else "")
        self.filter_stats.configure(text=f"Points avec écho   {total}\nGardés             {nkept}\n"
                                         f"Hors portée/intens. {len(rr)}\nIsolés (voisinage) {len(rn)}\n"
                                         f"Écartés            {100 * (1 - nkept / max(total, 1)):.1f} %")
        self.legend.configure(text="✕ rouge = hors portée / intensité · ✕ orange = isolé (voisinage)")

    def _render_align(self) -> None:
        caps = self.included()
        self.map.draw_analysis(None, None, [], [], None)
        self.map.draw_overlay([(self._pts(c), c.color, 9 if c.number in self.sel else 3, "o", c.number)
                               for c in caps], [], self._poses(caps))
        lines = [f"{c.title:<10s} {self.icp_info.get(c.number, '—')}" for c in caps]
        self.align_text.configure(text="\n".join(lines) or "—")
        self.legend.configure(text="gros points = scan sélectionné · « ⚠ » = recouvrement faible, à vérifier")

    def _render_grid(self) -> None:
        caps = self.included()
        if not caps:
            return
        n = len(caps)
        if getattr(self, "_grid_n", None) != n:      # nouveau nombre de scans : on repart de « tous »
            self._grid_n = n
            self.g_step.slider.configure(to=max(n, 2), number_of_steps=max(n - 1, 1))
            self.g_step.slider.set(n)
        k = max(1, min(int(round(self.g_step.get())), n))
        self.g_step.val.configure(text=f"{k} / {n}")
        params = self.params(vector=False)
        res = build_maps([c.scan for c in caps[:k]], params)
        beams = beam_image(caps[k - 1].scan, params) if self.g_beams.get() else None
        pts = [(self._pts(c), c.color, 3, "o", c.number) for c in caps[:k]] if self.g_points.get() else []
        self.map.draw_analysis(res.grid_image(params.grid.model), res.grid.extent() if res.grid else None, [], [], beams)
        self.map.draw_overlay(pts, [], self._poses(caps[:k]))
        g = res.stats.get("grid", {})
        self.grid_stats.configure(text=f"Scans intégrés  {k} / {n}  (dernier : {caps[k - 1].title})\n"
                                       f"Cellules        {g.get('cells', '—')}\nOccupées        {g.get('occupied', '—')}\n"
                                       f"Libres          {g.get('free', '—')}\nInconnues       {g.get('unknown', '—')}\n"
                                       f"Surface vue     {g.get('explored_area_m2', '—')} m²")
        self.legend.configure(text="blanc = libre · noir = occupé · gris = inconnu · bleu/rouge = cellules du dernier scan")

    def _render_vector(self) -> None:
        stage = self.v_stage.get()
        caps = self.included()
        self.map.draw_analysis(None, None, [], [], None)
        if stage == "Carte":
            res = build_maps([c.scan for c in caps], self.params(grid=False))
            faint = [(self._pts(c), "#3d4654", 3, "o", c.number) for c in caps]
            self.map.draw_overlay(faint, [(res.segments, "#ff6b6b", 2.6)], self._poses(caps))
            v = res.stats.get("vector", {})
            self.vec_stats.configure(text=f"Scans           {len(caps)}\nSegments        {v.get('segments', 0)}\n"
                                          f"Longueur totale {v.get('total_length_m', 0)} m\nPlus long       {v.get('longest_m', 0)} m")
            self.legend.configure(text="rouge = carte vectorielle (association + fusion + absences sur tous les scans)")
            return
        cap = self._find(self.selected) or (caps[0] if caps else None)
        if cap is None:
            return
        s = self.filters().apply(cap.scan).sorted_by_angle()
        st = extraction_stages(s.global_points(), s.ranges_m, self.extraction_params())
        color = lambda i: PALETTE[i % len(PALETTE)]  # noqa: E731
        pts, segs = [], []
        if stage == "Clusters":
            pts = [(c, color(i), 7, "o") for i, c in enumerate(st.clusters)]
        elif stage == "Split":
            pts = [(c, color(i), 7, "o") for i, c in enumerate(st.pieces)] + \
                  [(c, "#57606a", 7, "x") for c in st.rejected]
        elif stage == "Ajustement":
            pts = [(c, "#57606a", 4, "o") for c in st.pieces]
            segs = [([sg], color(i), 2.5) for i, sg in enumerate(st.fitted)]
        else:   # Fusion
            pts = [(c, "#3d4654", 4, "o") for c in st.pieces]
            segs = [(st.merged, "#ff6b6b", 3)]
        self.map.draw_overlay(pts, segs, [(cap.scan.pose, cap.color, True)])
        self.vec_stats.configure(text=f"Scan            {cap.title}\nClusters        {len(st.clusters)}\n"
                                      f"Morceaux (split) {len(st.pieces)}  (+{len(st.rejected)} écartés)\n"
                                      f"Segments ajustés {len(st.fitted)}\nAprès fusion    {len(st.merged)}")
        self.legend.configure(text={"Clusters": "une couleur par cluster (groupe de points contigus)",
                                    "Split": "une couleur par morceau rectiligne · ✕ gris = écarté",
                                    "Ajustement": "une droite ajustée par morceau",
                                    "Fusion": "segments du scan après fusion des voisins alignés"}[stage])

    def _render_result(self) -> None:
        caps = self.included()
        params = self.params(grid=bool(self.r_grid.get()), vector=bool(self.r_seg.get()))
        res = build_maps([c.scan for c in caps], params)
        self.last_result = res
        img = res.grid_image(params.grid.model)
        ref = self.reference if self.r_ref.get() else []
        self.map.draw_analysis(img, res.grid.extent() if res.grid else None, [], ref, None)
        pts = [(self._pts(c), c.color, 3, "o", c.number) for c in caps] if self.r_pts.get() else []
        poses = self._poses(caps) if self.r_poses.get() else []
        self.map.draw_overlay(pts, [(res.segments, "#ff6b6b", 2.4)], poses)
        st = res.stats
        lines = [f"Scans utilisés  {st.get('scans', 0)}", f"Points          {st.get('points', 0)}"]
        if "grid" in st:
            g = st["grid"]
            lines += [f"Grid map        {g['cells']}", f"  occupées/libres {g['occupied']} / {g['free']}",
                      f"  surface vue   {g['explored_area_m2']} m²"]
        if "vector" in st:
            v = st["vector"]
            lines += [f"Segments        {v['segments']}", f"  longueur      {v['total_length_m']} m"]
        self.result_stats.configure(text="\n".join(lines))
        self.legend.configure(text="carte finale · violet pointillé = référence MapTool")

    # ================================================================== données
    def open_files(self, paths=None) -> None:
        paths = paths or filedialog.askopenfilenames(title="Ouvrir des scans", initialdir="data",
                                                     filetypes=[("Scans lidar", "*.json *.csv"), ("Tous", "*.*")])
        added = 0
        for path in paths:
            try:
                for label, scan in read_any(path):
                    n = max((c.number for c in self.captures), default=0) + 1
                    cap = Capture(n, scan, PALETTE[(n - 1) % len(PALETTE)], label)
                    self.captures = self.captures + [cap]
                    self.originals[n] = scan.pose
                    added += 1
            except (ValueError, OSError, KeyError) as exc:
                messagebox.showerror("Ouvrir", f"{Path(path).name} :\n{exc}")
        if added:
            self.selected = self.selected or self.captures[0].number
            self.sel = self.sel or {self.selected}
            self.status(f"{added} scan(s) chargé(s).")
            self._refresh_list()
            self.render()
            self.fit_view()

    def _refresh_list(self) -> None:
        for w in self.scan_list.winfo_children():
            w.destroy()
        self.count_label.configure(text=f"{len(self.included())} / {len(self.captures)} scans utilisés")
        for cap in self.captures:
            sel = cap.number in self.sel
            row = ctk.CTkFrame(self.scan_list, fg_color=T.SURFACE_2, corner_radius=10,
                               border_width=3 if cap.number == self.selected else 2,
                               border_color=cap.color if sel else T.SURFACE_2)
            row.pack(fill="x", pady=3)
            var = ctk.BooleanVar(value=cap.visible)
            ctk.CTkCheckBox(row, text="", variable=var, width=24, fg_color=cap.color, hover_color=cap.color,
                            command=lambda n=cap.number, v=var: self._toggle(n, v.get())).pack(side="left", padx=(10, 2))
            info = ctk.CTkFrame(row, fg_color="transparent")
            info.pack(side="left", fill="x", expand=True, pady=5)
            p = cap.scan.pose
            moved = "  ✎" if p != self.originals.get(cap.number, p) else ""
            ctk.CTkLabel(info, text=f"{cap.title}{moved}", font=font(12, "bold"), text_color=cap.color, anchor="w").pack(fill="x")
            ctk.CTkLabel(info, text=f"{int((cap.scan.ranges_m > 0).sum())} pts · ({p.x:.2f}, {p.y:.2f}, {p.theta_deg:.1f}°)",
                         font=font(10, mono=True), text_color=T.MUTED, anchor="w").pack(fill="x")
            for w in (row, info, *info.winfo_children()):
                w.bind("<Button-1>", lambda e, n=cap.number: self.select(n))
                w.bind("<Control-Button-1>", lambda e, n=cap.number: self.select(n, additive=True))

    def _toggle(self, number: int, on: bool) -> None:
        self.captures = [replace(c, visible=on) if c.number == number else c for c in self.captures]
        self._refresh_list()
        self._later()

    def _set_all(self, on: bool) -> None:
        self.captures = [replace(c, visible=on) for c in self.captures]
        self._refresh_list()
        self._later()

    def _find(self, number) -> Capture | None:
        return next((c for c in self.captures if c.number == number), None)

    def select(self, number: int, additive: bool = False) -> None:
        """Clic : ce scan seul. Ctrl + clic : l'ajouter à la sélection (ou l'en retirer)."""
        cap = self._find(number)
        if not cap:
            return
        if additive and number in self.sel and len(self.sel) > 1:
            self.sel = self.sel - {number}
            if self.selected == number:
                self.selected = min(self.sel)
        else:
            self.sel = (self.sel | {number}) if additive else {number}
            self.selected = number
        self._show_selection()
        self._refresh_list()
        self._later()

    def select_all(self) -> None:
        caps = self.included()
        if caps:
            self.sel = {c.number for c in caps}
            self.selected = self.selected if self.selected in self.sel else caps[0].number
            self._show_selection()
            self._refresh_list()
            self._later()

    def select_primary_only(self) -> None:
        if self.selected is not None:
            self.select(self.selected)

    def _show_selection(self) -> None:
        cap = self._find(self.selected)
        if cap is None:
            return
        p = cap.scan.pose
        self.pose_entry.set(p.x, p.y, p.theta_deg)
        extra = f"  + {len(self.sel) - 1} autre(s)" if len(self.sel) > 1 else ""
        self.sel_label.configure(text=f"● {cap.title}{extra}", text_color=cap.color)

    # ------------------------------------------------------------------ souris sur la carte
    def _editor(self) -> Editor:
        return Editor(pick=self._pick, selection=lambda: self.sel, select=self.select,
                      pivot=self._pivot, commit=self._commit_move)

    def _pick(self, x: float, y: float, tol: float) -> int | None:
        """Scan dont un point (ou la pose) est le plus proche du clic, à moins de tol mètres."""
        best, best_d = None, tol
        click = np.array([x, y])
        for c in self.included():
            pose = np.array([[c.scan.pose.x, c.scan.pose.y]])
            pts = np.vstack([self._pts(c), pose])
            d = float(np.min(np.hypot(*(pts - click).T)))
            if d < best_d or (d == best_d and c.number in self.sel):
                best, best_d = c.number, d
        return best

    def _pivot(self, keys) -> tuple[float, float]:
        poses = [c.scan.pose for c in self.captures if c.number in keys]
        return (float(np.mean([p.x for p in poses])), float(np.mean([p.y for p in poses]))) if poses else (0.0, 0.0)

    def _commit_move(self, keys, dx: float, dy: float, dth: float, pivot) -> None:
        moves = {c.number: move_rigid(c.scan.pose, dx, dy, dth, pivot) for c in self.captures if c.number in keys}
        self._set_poses(moves, "manuel")

    def toggle_edit(self) -> None:
        on = self.map.editor is None
        self.map.set_editor(self._editor() if on else None)
        self.edit_btn.configure(text=f"✋  Déplacer les scans : {'oui' if on else 'non'}",
                                fg_color=T.ACCENT if on else T.SURFACE_2, text_color="#06120b" if on else T.INK)
        self.status("Glisse un scan pour le déplacer, clic droit glissé pour le tourner." if on else
                    "Déplacement des scans coupé : glisser déplace seulement la vue.")

    # ================================================================== recalage
    def _set_poses(self, poses: dict[int, Pose], info: str | dict | None = None, remember: bool = True) -> None:
        """Change plusieurs poses d'un coup (un seul rafraîchissement) ; info = texte commun ou par scan."""
        if not poses:
            return
        if remember:
            self.history = (self.history + [{c.number: c.scan.pose for c in self.captures}])[-HISTORY:]
        self.captures = [replace(c, scan=c.scan.with_pose(poses[c.number])) if c.number in poses else c
                         for c in self.captures]
        for n in poses:
            text = info.get(n) if isinstance(info, dict) else info
            if text is not None:
                self.icp_info[n] = text
        self._show_selection()
        self._refresh_list()
        self._later()

    def undo(self) -> None:
        if not self.history:
            self.status("Rien à annuler.")
            return
        last, self.history = self.history[-1], self.history[:-1]
        self._set_poses({n: p for n, p in last.items() if self._find(n)}, remember=False)
        self.status(f"Annulé (encore {len(self.history)} retour(s) possible(s)).")

    def nudge(self, dx, dy, dth) -> None:
        """Flèches / boutons : déplace ou tourne TOUTE la sélection (rotation autour de son centre)."""
        if not self.sel:
            return
        m, d = NUDGES[self.nudge_mode.get()]
        pivot = self._pivot(self.sel)
        self._commit_move(set(self.sel), dx * m, dy * m, dth * d, pivot)

    def apply_pose(self) -> None:
        cap = self._find(self.selected)
        try:
            pose = Pose(*self.pose_entry.get())
        except ValueError:
            messagebox.showwarning("Pose", "x, y et θ doivent être des nombres.")
            return
        if cap:
            self._set_poses({cap.number: pose}, "manuel")

    @staticmethod
    def _describe(res, before: Pose) -> str:
        if not res.converged:
            return "ICP refusé (n'améliorait pas l'accord) : pose gardée"
        warn = " ⚠" if res.inlier_ratio < LOW_OVERLAP else ""
        p = res.pose
        return (f"ICP rms {res.rms_m * 100:4.1f}cm rec.{res.inlier_ratio:4.0%}{warn} "
                f"Δ({p.x - before.x:+.2f},{p.y - before.y:+.2f},{p.theta_deg - before.theta_deg:+.1f}°)")

    def icp_selected(self) -> None:
        """Recale la sélection EN BLOC (ses scans gardent leur placement relatif) sur les autres scans."""
        group = [c for c in self.included() if c.number in self.sel]
        others = [c for c in self.included() if c.number not in self.sel]
        if not group or not others:
            messagebox.showinfo("ICP", "Il faut au moins un scan sélectionné et un autre scan coché, non sélectionné.")
            return
        ref = voxel_downsample(np.vstack([self._pts(c) for c in others]), 0.03)
        block = voxel_downsample(np.vstack([self._pts(c) for c in group]), 0.02)
        res = icp(block, Pose(), ref)                 # points déjà dans la carte : pose initiale = identité
        names = ", ".join(c.title for c in group)
        if not res.converged:
            self.status(f"ICP impossible pour {names} (pas assez de points communs).")
            return
        if res.inlier_ratio < LOW_OVERLAP and not messagebox.askyesno(
                "Recouvrement faible", f"Seulement {res.inlier_ratio:.0%} des points de {names} retrouvent un "
                "voisin : le recalage est peu fiable. L'appliquer quand même ?"):
            return
        t = res.pose
        moves = {c.number: move_rigid(c.scan.pose, t.x, t.y, t.theta_deg, (0.0, 0.0)) for c in group}
        self._set_poses(moves, f"ICP bloc rms {res.rms_m * 100:4.1f}cm rec.{res.inlier_ratio:4.0%} "
                               f"Δ({t.x:+.2f},{t.y:+.2f},{t.theta_deg:+.1f}°)")
        self.status(f"{names} recalé(s) en bloc : erreur résiduelle {res.rms_m * 100:.1f} cm.")

    def _anchor(self, caps) -> int:
        return next((i for i, c in enumerate(caps) if c.number == self.selected), 0)

    def icp_all(self) -> None:
        caps = self.included()
        if len(caps) < 2:
            return
        anchor = self._anchor(caps)
        self.status(f"Recalage de {len(caps) - 1} scans sur {caps[anchor].title}…")
        self.root.update_idletasks()
        out = align_sequence([self._local(c) for c in caps], [c.scan.pose for c in caps], anchor)
        self._set_poses({c.number: pose for c, (pose, _) in zip(caps, out)},
                        {c.number: "ancre (fixe)" if res is None else self._describe(res, c.scan.pose)
                         for c, (_, res) in zip(caps, out)})
        weak = [c.title for c, (_, r) in zip(caps, out) if r is not None and r.inlier_ratio < LOW_OVERLAP]
        self.status("ICP scan par scan terminé." + (f" À vérifier (recouvrement faible) : {', '.join(weak)}."
                                                    if weak else ""))

    def optimize_global(self, quiet: bool = False) -> None:
        """Toutes les poses corrigées ensemble (matching.refine_global)."""
        caps = self.included()
        if len(caps) < 2:
            return
        anchor = self._anchor(caps)
        self.status("Optimisation globale des poses…")
        self.root.update_idletasks()
        res = refine_global([self._local(c) for c in caps], [c.scan.pose for c in caps], anchor)
        self._set_poses({c.number: p for c, p in zip(caps, res.poses)},
                        {c.number: "ancre (fixe)" if k == anchor else
                         f"global : écart {b * 100:4.1f} → {a * 100:4.1f} cm"
                         for k, (c, b, a) in enumerate(zip(caps, res.gap_before_m, res.gap_after_m))})
        if not quiet:
            self.status(f"Optimisation globale : écart médian entre scans {res.median_before * 100:.1f} → "
                        f"{res.median_after * 100:.1f} cm ({res.iterations} itérations). Ctrl+Z pour annuler.")
        return res

    def align_full(self) -> None:
        """Le recalage « boosté » : optimisation globale depuis deux départs, on garde le meilleur."""
        caps = self.included()
        if len(caps) < 2:
            return
        anchor = self._anchor(caps)
        self.status("Recalage complet : deux départs essayés, puis optimisation globale…")
        self.root.update_idletasks()
        res, how = align_best([self._local(c) for c in caps], [c.scan.pose for c in caps], anchor)
        self._set_poses({c.number: p for c, p in zip(caps, res.poses)},
                        {c.number: "ancre (fixe)" if k == anchor else
                         f"complet : écart {b * 100:4.1f} → {a * 100:4.1f} cm"
                         for k, (c, b, a) in enumerate(zip(caps, res.gap_before_m, res.gap_after_m))})
        self.status(f"Recalage complet ({how}) : écart médian entre scans {res.median_before * 100:.1f} → "
                    f"{res.median_after * 100:.1f} cm. Ctrl+Z pour revenir en arrière.")

    def reset_poses(self) -> None:
        self._set_poses({c.number: self.originals[c.number] for c in self.captures}, "—")
        self.status("Poses d'origine restaurées (Ctrl+Z pour annuler).")

    def measure(self) -> None:
        caps = self.included()
        if len(caps) < 2:
            return
        med = consistency([self._local(c) for c in caps], [c.scan.pose for c in caps])
        k = int(np.nanargmax(med))
        self.status(f"Cohérence : écart médian entre chaque scan et les autres = {np.nanmedian(med) * 100:.1f} cm "
                    f"(pire : {med[k] * 100:.1f} cm, {caps[k].title}).")

    # ================================================================== sorties
    def import_reference(self) -> None:
        path = filedialog.askopenfilename(title="Carte de référence", filetypes=[("Carte vectorielle", "*.txt")])
        if path:
            self.reference = load_vector_map(path).segments
            self.status(f"Référence : {len(self.reference)} segment(s).")
            self._later()

    def save_project(self) -> None:
        if not self.captures:
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="projet_salle.lidar.json",
                                            initialdir="data/salle", filetypes=[("Session LidarScope", "*.json")])
        if path:
            extra = {"map_params": self.params().as_dict(), "icp": {str(k): v for k, v in self.icp_info.items()},
                     "original_poses": {str(k): [p.x, p.y, p.theta_deg] for k, p in self.originals.items()}}
            save_session(self.captures, path, extra)
            self.status(f"Projet enregistré : {path}")

    def open_export(self) -> None:
        if self.included():
            ExportDialog(self.root, len(self.included()), self.selected is not None, self._do_export)

    def _do_export(self, folder, options) -> None:
        caps = self.included() if options.scope != "selection" else self.captures
        root = export_bundle(folder, caps, self.selected, options, self.params(), self.reference)
        self.status(f"Export terminé : {root}")
        messagebox.showinfo("Export", f"Dossier créé :\n{root}")

    def export_png(self) -> None:
        if not self.included():
            return
        path = filedialog.asksaveasfilename(defaultextension=".png", initialfile="carte_salle.png",
                                            filetypes=[("Image PNG", "*.png")])
        if path:
            params = self.params(grid=bool(self.r_grid.get()), vector=bool(self.r_seg.get()))
            caps = self.included()
            res = build_maps([c.scan for c in caps], params)
            render_png(path, caps, res, params.grid.model, f"Carte de la salle ({len(caps)} scans)",
                       self.reference, show_points=bool(self.r_pts.get()))
            self.status(f"Image enregistrée : {path}")

    # ================================================================== divers
    def fit_view(self) -> None:
        caps = self.included()
        if caps:
            self.map.fit(np.vstack([self._pts(c) for c in caps] + [np.array([[c.scan.pose.x, c.scan.pose.y]]) for c in caps]))

    def toggle_ruler(self) -> None:
        on = not self.map.ruler_mode
        self.map.set_ruler(on)
        self.ruler_btn.configure(fg_color=T.WARN if on else T.SURFACE_2, text_color="#000" if on else T.INK)

    def status(self, text: str) -> None:
        self.status_var.set(text)

    @staticmethod
    def _typing(event) -> bool:
        return "entry" in str(event.widget).lower()

    def quit(self) -> None:
        if self._job:
            self.root.after_cancel(self._job)
        self.root.quit()
        self.root.destroy()


def main() -> None:
    ctk.set_appearance_mode("dark")
    root = ctk.CTk()
    app = LidarStudio(root)
    if len(sys.argv) > 1:
        root.after(300, lambda: app.open_files(sys.argv[1:]))
    root.mainloop()


if __name__ == "__main__":
    main()
