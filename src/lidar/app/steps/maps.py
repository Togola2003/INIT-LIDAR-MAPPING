"""Étape 4 · Cartes : grid map et carte vectorielle selon le cours, plus retouches à la main des murs."""

from __future__ import annotations

from dataclasses import replace
from tkinter import filedialog

import customtkinter as ctk

from ...scope.mapping import GridParams, MapParams, VectorParams
from ...vectormap import load_vector_map
from .. import project as P
from .. import style as S
from ..tools import WallTool
from .base import Step

RESOLUTIONS = {"2 cm": 0.02, "5 cm": 0.05, "10 cm": 0.10}
SHOW = ["Les deux", "Grid map", "Vectorielle"]
EDIT = ["Aucune", "Tracer", "Prolonger", "Supprimer"]


class MapsStep(Step):
    title = "Cartes"
    subtitle = "grid map, segments"
    headline = "Fond : blanc = libre, noir = occupé, gris = inconnu. En rouge : les murs de la carte vectorielle."
    course = ("Grid map : chaque case porte une probabilité d'occupation (log-odds). Un faisceau rend libres les "
              "cases qu'il traverse et occupée la case d'impact. Carte vectorielle : nuage → clusters → découpe "
              "(split) → droites ajustées → fusion, puis association avec la carte et détection d'absence.")

    def build(self, panel) -> None:
        card = S.Card(panel, "Affichage")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.show = S.segmented(b, SHOW, SHOW[0], lambda v: self._changed())
        self.show.pack(fill="x", pady=(0, 8))
        self.points = S.toggle(b, "Points des scans", self._changed)
        self.poses = S.toggle(b, "Poses du robot", self._changed, on=True)
        self.ref = S.toggle(b, "Carte de référence (violet)", self._changed, on=True)
        for w in (self.points, self.poses, self.ref):
            w.pack(anchor="w", pady=2)
        S.button(b, "Importer une carte de référence (.txt)…", self.import_reference, "ghost").pack(fill="x", pady=(6, 0))

        card = S.Card(panel, "Grid map")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        for label, attr, values, val in (("Résolution", "res", list(RESOLUTIONS), "5 cm"),
                                         ("État initial", "init", ["inconnu", "libre", "occupé"], "inconnu"),
                                         ("Modèle", "model", ["probabiliste", "binaire"], "probabiliste")):
            ctk.CTkLabel(b, text=label, font=S.font(12), text_color=S.INK, anchor="w").pack(fill="x")
            w = S.segmented(b, values, val, lambda v: self._changed())
            w.pack(fill="x", pady=(2, 8))
            setattr(self, attr, w)
        self.p_occ = S.Slider(b, "p(occupé | impact)", 0.55, 0.95, 0.70, "{:.2f}", self._changed)
        self.p_free = S.Slider(b, "p(occupé | traversée)", 0.05, 0.45, 0.40, "{:.2f}", self._changed)
        self.p_occ.pack(fill="x")
        self.p_free.pack(fill="x")
        self.iso = S.toggle(b, "Supprimer les cases occupées isolées", self._changed)
        self.iso.pack(anchor="w", pady=(4, 0))

        card = S.Card(panel, "Carte vectorielle")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.jump = S.Slider(b, "Saut entre deux objets", 0.02, 0.40, 0.08, "{:.2f} m", self._changed)
        self.split = S.Slider(b, "Tolérance point-droite", 0.01, 0.12, 0.04, "{:.2f} m", self._changed)
        self.min_len = S.Slider(b, "Longueur minimale", 0.05, 1.0, 0.30, "{:.2f} m", self._changed)
        self.angle = S.Slider(b, "Fusion si angle ≤", 2, 25, 10, "{:.0f}°", self._changed)
        for s in (self.jump, self.split, self.min_len, self.angle):
            s.pack(fill="x")

        card = S.Card(panel, "Retouches des murs", "Tracer : glisser d'un point à un autre. Prolonger : attraper "
                                                   "une extrémité. Les extrémités proches s'aimantent.")
        card.pack(fill="x", pady=(0, 10))
        self.edit = S.segmented(card.body, EDIT, EDIT[0], lambda v: self._edit_mode())
        self.edit.pack(fill="x", pady=(0, 8))
        self.edit_info = S.hint(card.body, "")
        S.button(card.body, "Repartir de la carte calculée", self.forget_edits, "ghost").pack(fill="x")

        card = S.Card(panel, "Bilan")
        card.pack(fill="x", pady=(0, 10))
        self.stats = S.KeyValues(card.body)
        self.stats.pack(fill="x")

    # ------------------------------------------------------------------ réglages
    def params(self, grid: bool = True, vector: bool = True) -> MapParams:
        return MapParams(
            filters=self.app.filters(),
            grid=GridParams(grid, RESOLUTIONS[self.res.get()], self.init.get(), self.model.get(),
                            round(self.p_occ.get(), 3), round(self.p_free.get(), 3), bool(self.iso.get())),
            vector=VectorParams(vector, round(self.jump.get(), 3), round(self.split.get(), 3),
                                round(self.min_len.get(), 3), round(self.angle.get(), 1)))

    def _changed(self, _v=None) -> None:
        self.app.render(150)

    def import_reference(self) -> None:
        path = filedialog.askopenfilename(title="Carte de référence", filetypes=[("Carte vectorielle", "*.txt")])
        if path:
            segs = load_vector_map(path).segments
            self.app.apply(replace(self.project, reference=tuple(segs)), f"Carte de référence : {len(segs)} segment(s).")

    # ------------------------------------------------------------------ retouches
    def segments(self) -> list:
        if self.project.manual_segments is not None:
            return list(self.project.manual_segments)
        return list(self.app.maps(params=self.params(grid=False)).segments) if self.project.visible else []

    def _set_segments(self, segs) -> None:
        self.app.apply(P.set_manual_segments(self.project, segs))

    def _edit_mode(self) -> None:
        mode = self.edit.get().lower()
        if mode == "aucune":
            self.app.set_tool(None)
        else:
            self.app.set_tool(WallTool(self.app.map, mode, self.segments, self._set_segments))
            self.app.status({"tracer": "Glisse d'un point à un autre pour ajouter un mur.",
                             "prolonger": "Attrape une extrémité de mur et déplace-la.",
                             "supprimer": "Clique sur un mur pour le supprimer."}[mode])

    def forget_edits(self) -> None:
        self.app.apply(P.set_manual_segments(self.project, None), "Carte vectorielle recalculée (retouches oubliées).")

    def enter(self) -> None:
        self._edit_mode()

    # ------------------------------------------------------------------ carte
    def render(self) -> None:
        app, show = self.app, self.show.get()
        caps = self.project.visible
        if not caps:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        want_grid, want_vec = show != "Vectorielle", show != "Grid map"
        params = self.params(grid=want_grid, vector=want_vec and self.project.manual_segments is None)
        res = app.maps(params=params)
        img = res.grid_image(params.grid.model) if want_grid else None
        segs = (list(self.project.manual_segments) if self.project.manual_segments is not None else res.segments) \
            if want_vec else []
        ref = list(self.project.reference) if self.ref.get() else []
        app.map.draw_analysis(img, res.grid.extent() if img is not None else None, segs, ref, None)
        pts, poses = app.scan_layers(caps, big=False)
        app.map.draw_overlay(pts if self.points.get() else [], [], poses if self.poses.get() else [])
        manual = self.project.manual_segments is not None
        self.edit_info.configure(text=f"Carte retouchée à la main : {len(self.project.manual_segments)} murs."
                                 if manual else "Carte calculée automatiquement (pas de retouche).",
                                 text_color=S.ACCENT if manual else S.MUTED)
        rows = [("Scans", str(len(caps)))]
        g = res.stats.get("grid")
        if g:
            rows += [("Grille", g["cells"]), ("Cases occupées", str(g["occupied"])),
                     ("Surface vue", f"{g['explored_area_m2']} m²")]
        if want_vec:
            rows += [("Murs (segments)", str(len(segs))),
                     ("Longueur totale", f"{sum(s.length for s in segs):.1f} m")]
        self.stats.show(rows)
        self.legend("blanc = libre · noir = occupé · gris = inconnu · rouge = murs · violet = référence")
