"""Étape 4 · Cartes : grid map et carte vectorielle selon le cours, retouches à la main, validation.

Les retouches (barre d'outils au-dessus de la carte : tracer, prolonger, effacer un mur) sont des
opérations rejouées sur les DEUX cartes. « Valider » fige le résultat comme carte de référence :
c'est sur elle que l'étape Localiser travaille.
"""

from __future__ import annotations

from dataclasses import replace
from tkinter import filedialog

import customtkinter as ctk

from ...refmap import build_reference
from ...scope.mapping import GridParams, MapParams, VectorParams
from ...vectormap import load_vector_map
from .. import project as P
from .. import style as S
from .base import Step

RESOLUTIONS = {"2 cm": 0.02, "5 cm": 0.05, "10 cm": 0.10}
SHOW = ["Les deux", "Grid map", "Murs"]


class MapsStep(Step):
    title = "Cartes"
    subtitle = "grid map, murs, validation"
    headline = "Blanc = libre, noir = occupé, gris = inconnu, rouge = murs. Retouches : barre d'outils ci-dessus."
    course = ("Grid map : chaque case porte une probabilité d'occupation (log-odds). Un faisceau rend libres les "
              "cases qu'il traverse et occupée la case d'impact. Carte vectorielle : nuage → clusters → découpe "
              "(split) → droites ajustées → fusion. Une fois la carte bonne, on la valide : c'est la référence "
              "sur laquelle le robot se localise.")

    def build(self, panel) -> None:
        card = S.Card(panel, "Carte de référence", "Quand la carte te convient, valide-la : la localisation "
                                                  "se fera sur elle (retouches comprises).")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "✓  Valider cette carte comme référence", self.validate, "primary", height=42,
                 font=S.font(14, "bold")).pack(fill="x")
        self.ref_info = S.hint(card.body, "")

        card = S.Card(panel, "Affichage")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.show = S.segmented(b, SHOW, SHOW[0], lambda v: self._changed())
        self.show.pack(fill="x", pady=(0, 8))
        self.points = S.toggle(b, "Points des scans", self._changed)
        self.poses = S.toggle(b, "Poses du robot", self._changed, on=True)
        self.ref = S.toggle(b, "Carte importée (violet)", self._changed, on=True)
        for w in (self.points, self.poses, self.ref):
            w.pack(anchor="w", pady=2)

        card = S.Card(panel, "Retouches à la main", "Outils au-dessus de la carte : ✎ Tracer, ↔ Prolonger, "
                                                    "✕ Effacer. Chaque retouche modifie la grid map ET les murs.")
        card.pack(fill="x", pady=(0, 10))
        self.edit_info = S.hint(card.body, "")
        S.button(card.body, "Annuler toutes les retouches", self.forget_edits, "ghost").pack(fill="x")

        card = S.Card(panel, "Bilan")
        card.pack(fill="x", pady=(0, 10))
        self.stats = S.KeyValues(card.body)
        self.stats.pack(fill="x")

        adv = S.Collapsible(panel, "Réglages de la grid map")
        adv.pack(fill="x", pady=(0, 10))
        b = adv.body
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

        adv = S.Collapsible(panel, "Réglages de la carte vectorielle")
        adv.pack(fill="x", pady=(0, 10))
        b = adv.body
        self.jump = S.Slider(b, "Saut entre deux objets", 0.02, 0.40, 0.08, "{:.2f} m", self._changed)
        self.split = S.Slider(b, "Tolérance point-droite", 0.01, 0.12, 0.04, "{:.2f} m", self._changed)
        self.min_len = S.Slider(b, "Longueur minimale", 0.05, 1.0, 0.30, "{:.2f} m", self._changed)
        self.angle = S.Slider(b, "Fusion si angle ≤", 2, 25, 10, "{:.0f}°", self._changed)
        for s in (self.jump, self.split, self.min_len, self.angle):
            s.pack(fill="x")
        S.button(b, "Importer une carte à comparer (.txt MapTool)…", self.import_reference, "ghost").pack(fill="x", pady=(6, 0))

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
        path = filedialog.askopenfilename(title="Carte à comparer", filetypes=[("Carte vectorielle", "*.txt")])
        if path:
            segs = load_vector_map(path).segments
            self.app.apply(replace(self.project, reference=tuple(segs)), f"Carte importée : {len(segs)} segment(s).")

    def forget_edits(self) -> None:
        if self.project.wall_edits:
            self.app.apply(P.clear_edits(self.project), "Retouches annulées (Ctrl+Z pour les retrouver).")

    # ------------------------------------------------------------------ validation
    def validate(self) -> None:
        caps = self.project.visible
        if not caps:
            self.app.status("Rien à valider : aucun scan visible.")
            return
        p = self.params()
        ref = build_reference([self.app.filtered(c) for c in caps], [c.title for c in caps], self.project.wall_edits,
                              p.grid.resolution_m, p.vector.to_vectormap(),
                              name=f"{self.project.name} ({len(caps)} scans)")
        self.app.apply(P.set_reference_map(self.project, ref),
                       f"Carte validée comme référence : {ref.describe()}. Elle sert maintenant à la localisation.")

    def on_project(self) -> None:
        ref = self.project.reference_map
        self.ref_info.configure(text=f"Référence actuelle : {ref.describe()}" if ref else
                                "Aucune carte validée pour l'instant.", text_color=S.SUCCESS if ref else S.MUTED)
        n = len(self.project.wall_edits)
        self.edit_info.configure(text=f"{n} retouche(s) appliquée(s) aux deux cartes." if n else "Aucune retouche.",
                                 text_color=S.ACCENT if n else S.MUTED)

    def enter(self) -> None:
        self.on_project()

    # ------------------------------------------------------------------ carte
    def render(self) -> None:
        app, show = self.app, self.show.get()
        caps = self.project.visible
        if not caps:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        want_grid, want_vec = show != "Murs", show != "Grid map"
        params = self.params(grid=want_grid, vector=want_vec)
        res = app.final_maps(params)
        img = res.grid_image(params.grid.model) if want_grid else None
        segs = res.segments if want_vec else []
        ref = list(self.project.reference) if self.ref.get() else []
        app.map.draw_analysis(img, res.grid.extent() if img is not None else None, segs, ref, None)
        pts, poses = app.scan_layers(caps, big=False)
        app.map.draw_overlay(pts if self.points.get() else [], [], poses if self.poses.get() else [])
        rows = [("Scans", str(len(caps)))]
        g = res.stats.get("grid")
        if g and res.grid is not None:
            t = res.grid.occupied_mask().sum()
            rows += [("Grille", g["cells"]), ("Cases occupées", str(int(t))), ("Surface vue", f"{g['explored_area_m2']} m²")]
        if want_vec:
            rows += [("Murs (segments)", str(len(segs))), ("Longueur totale", f"{sum(s.length for s in segs):.1f} m")]
        rows.append(("Retouches", str(len(self.project.wall_edits))))
        self.stats.show(rows)
        self.legend("blanc = libre · noir = occupé · gris = inconnu · rouge = murs · violet = carte importée")
