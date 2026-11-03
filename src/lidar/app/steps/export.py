"""Étape 6 · Exporter : projet, dossier complet (chaque scan / ensemble), image, carte vectorielle retouchée."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk

from ...refmap import to_dict
from ...scope.exporter import ITEMS, SCOPES, ExportOptions, export_bundle
from ...scope.render import render_png
from .. import style as S
from .base import Step


class ExportStep(Step):
    title = "Exporter"
    subtitle = "fichiers, images"
    headline = "Aperçu de la carte finale, telle qu'elle sera exportée."
    course = ("Les exports suivent les formats du module : scans CSV (angle, distance, intensité), grid map CSV "
              "(1 = occupé, comme MapTool), carte vectorielle x1,y1,x2,y2. La session .lidar.json décrit elle-même "
              "ses unités et ses repères : n'importe quel autre programme peut la relire.")

    def build(self, panel) -> None:
        card = S.Card(panel, "Projet", "Tout le travail : scans, poses, carte retouchée. Se rouvre dans LidarLab.")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Enregistrer le projet", self.app.save_project, "primary").pack(fill="x")
        S.button(card.body, "Enregistrer sous…", self.save_as, "ghost").pack(fill="x", pady=(4, 0))

        card = S.Card(panel, "Dossier d'export", "Un dossier daté avec un LISEZMOI.md qui explique chaque fichier.")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.scope = ctk.StringVar(value="chaque+ensemble")
        for key, label in SCOPES.items():
            ctk.CTkRadioButton(b, text=label, value=key, variable=self.scope, font=S.font(12), text_color=S.INK,
                               fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER, border_color=S.FAINT).pack(anchor="w", pady=2)
        ctk.CTkFrame(b, fg_color=S.BORDER, height=1).pack(fill="x", pady=8)
        self.items = {}
        for key, label in ITEMS.items():
            var = ctk.BooleanVar(value=True)
            ctk.CTkCheckBox(b, text=label, variable=var, font=S.font(12), text_color=S.INK, fg_color=S.ACCENT,
                            hover_color=S.ACCENT_HOVER, border_color=S.FAINT, corner_radius=6).pack(anchor="w", pady=2)
            self.items[key] = var
        S.button(b, "Choisir le dossier et exporter", self.export, "primary").pack(fill="x", pady=(10, 0))

        card = S.Card(panel, "Pour le compte rendu")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Image de la carte (PNG, fond blanc)", self.export_png).pack(fill="x")
        S.button(card.body, "Carte vectorielle (.txt, format MapTool)", self.export_vector).pack(fill="x", pady=(6, 0))

        card = S.Card(panel, "Résumé")
        card.pack(fill="x", pady=(0, 10))
        self.summary = S.KeyValues(card.body)
        self.summary.pack(fill="x")


    def save_as(self) -> None:
        self.app.store.apply(replace(self.project, path=None), remember=False)
        self.app.save_project()

    def export(self) -> None:
        caps = self.project.visible
        items = frozenset(k for k, v in self.items.items() if v.get())
        if not caps or not items:
            self.app.status("Rien à exporter : il faut au moins un scan visible et un type de fichier coché.")
            return
        folder = filedialog.askdirectory(title="Dossier où créer l'export", initialdir="out")
        if not folder:
            return
        try:
            root = export_bundle(folder, caps, self.app.primary, ExportOptions(self.scope.get(), items),
                                 self.app.map_params(), list(self.project.reference))
        except (ValueError, OSError) as exc:
            messagebox.showerror("Export", str(exc))
            return
        if self.project.wall_edits:
            self._write_segments(Path(root) / "carte_vectorielle_retouchee.txt", self.app.final_segments())
        if self.project.reference_map is not None:
            (Path(root) / "carte_reference.json").write_text(
                json.dumps(to_dict(self.project.reference_map), ensure_ascii=False), encoding="utf-8")
        self.app.status(f"Export terminé : {root}")

    def export_png(self) -> None:
        caps = self.project.visible
        if not caps:
            return
        path = filedialog.asksaveasfilename(defaultextension=".png", initialfile="carte_salle.png",
                                            filetypes=[("Image PNG", "*.png")])
        if path:
            params = self.app.map_params()
            res = self.app.final_maps(params)
            render_png(path, caps, res, params.grid.model, f"{self.project.name} · {len(caps)} scans",
                       list(self.project.reference), show_points=False)
            self.app.status(f"Image enregistrée : {path}")

    def export_vector(self) -> None:
        if not self.project.visible:
            return
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile="carte_vectorielle.txt",
                                            filetypes=[("Carte vectorielle", "*.txt")])
        if path:
            self._write_segments(Path(path), self.app.final_segments())
            self.app.status(f"Carte vectorielle enregistrée : {path}")

    @staticmethod
    def _write_segments(path: Path, segs) -> None:
        lines = [f"# Carte vectorielle LidarLab - {len(segs)} segment(s)", "# x1,y1,x2,y2 (mètres)"]
        lines += [",".join(f"{v:.4f}" for v in s.as_tuple()) for s in segs]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def render(self) -> None:
        app = self.app
        caps = self.project.visible
        if not caps:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            self.summary.show([("Scans", "0")])
            return
        params = app.map_params()
        res = app.final_maps(params)
        segs = res.segments
        app.map.draw_analysis(res.grid_image(params.grid.model), res.grid.extent() if res.grid else None, segs,
                              list(self.project.reference), None)
        _, poses = app.scan_layers(caps, big=False)
        app.map.draw_overlay([], [], poses)
        g = res.stats.get("grid", {})
        self.summary.show([("Projet", self.project.name), ("Scans visibles", f"{len(caps)} / {len(self.project.captures)}"),
                           ("Grid map", f"{g.get('cells', '—')} · {g.get('explored_area_m2', '—')} m²"),
                           ("Murs", f"{len(segs)} ({len(self.project.wall_edits)} retouche(s))"),
                           ("Carte de référence", "validée" if self.project.reference_map else "non validée"),
                           ("Enregistré", "non" if app.store.dirty else "oui")])
        self.legend("carte finale : grid map + murs (rouge) + poses du robot")
