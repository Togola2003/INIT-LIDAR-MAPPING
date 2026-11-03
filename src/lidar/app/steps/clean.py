"""Étape 2 · Nettoyer : filtres du cours (portée, intensité, points isolés), gomme à points, suppression de scans."""

from __future__ import annotations

from tkinter import messagebox

import numpy as np

from ...scope.processing import Filters
from .. import project as P
from .. import style as S
from ..tools import RectTool
from .base import Step

ERASE_IN = ["Scans sélectionnés", "Tous les scans visibles"]


class CleanStep(Step):
    title = "Nettoyer"
    subtitle = "filtres, gomme"
    headline = "Les points écartés sont marqués d'une croix : rouge = portée / intensité, orange = isolés."
    course = ("Prétraitement : on retire les points sans écho, hors portée ou d'intensité trop faible (reflets). "
              "Sans intensité (format Teleplot), le filtre de voisinage enlève les points isolés : un vrai mur "
              "donne des points serrés. La gomme enlève à la main un passant ou un objet temporaire.")

    def build(self, panel) -> None:
        card = S.Card(panel, "Portée et intensité")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.f_min = S.Slider(b, "Portée minimale", 0, 1, 0.15, "{:.2f} m", self._changed)
        self.f_max = S.Slider(b, "Portée maximale", 1, 12, 12, "{:.1f} m", self._changed)
        self.f_int = S.Slider(b, "Intensité minimale", 0, 255, 0, "{:.0f}", self._changed, steps=255)
        for s in (self.f_min, self.f_max, self.f_int):
            s.pack(fill="x")
        self.int_note = S.hint(b, "")

        card = S.Card(panel, "Points isolés", "Le rayon de recherche grandit de 4 cm par mètre de distance.")
        card.pack(fill="x", pady=(0, 10))
        self.f_nb = S.Slider(card.body, "Voisins minimum (0 = désactivé)", 0, 6, 2, "{:.0f}", self._changed, steps=6)
        self.f_rad = S.Slider(card.body, "Rayon à 0 m", 0.02, 0.30, 0.10, "{:.2f} m", self._changed)
        self.f_nb.pack(fill="x")
        self.f_rad.pack(fill="x")

        card = S.Card(panel, "Gomme", "Glisse un rectangle sur la carte : les points dedans sont effacés "
                                      "(Ctrl+Z pour annuler).")
        card.pack(fill="x", pady=(0, 10))
        self.erase_in = S.segmented(card.body, ERASE_IN, ERASE_IN[1], lambda v: None)
        self.erase_in.pack(fill="x", pady=(0, 8))
        self.eraser = S.toggle(card.body, "Gomme active", self._toggle_eraser)
        self.eraser.pack(anchor="w")

        card = S.Card(panel, "Scans")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Supprimer les scans sélectionnés", self.delete_selected, "danger").pack(fill="x")

        card = S.Card(panel, "Bilan")
        card.pack(fill="x", pady=(0, 10))
        self.stats = S.KeyValues(card.body)
        self.stats.pack(fill="x")

    def filters(self) -> Filters:
        lo = self.f_min.get()
        return Filters(round(lo, 3), round(max(self.f_max.get(), lo + 0.05), 2), int(self.f_int.get()),
                       int(round(self.f_nb.get())), round(self.f_rad.get(), 3))

    def _changed(self, _v=None) -> None:
        self.app.render(200)

    # ------------------------------------------------------------------ gomme / suppression
    def enter(self) -> None:
        if self.eraser.get():
            self._toggle_eraser()

    def _toggle_eraser(self) -> None:
        if self.eraser.get():
            self.app.set_tool(RectTool(self.app.map, self.erase))
            self.app.status("Gomme active : glisse un rectangle sur les points à effacer.")
        else:
            self.app.set_tool(None)

    def erase(self, box) -> None:
        if self.erase_in.get() == ERASE_IN[0]:
            nums = set(self.app.sel)
        else:
            nums = {c.number for c in self.project.visible}
        if not nums:
            self.app.status("Aucun scan concerné : sélectionne un scan ou choisis « tous les scans visibles ».")
            return
        new, n = P.erase_points(self.project, nums, box)
        if n:
            self.app.apply(new, f"{n} point(s) effacé(s). Ctrl+Z pour annuler.")
        else:
            self.app.status("Aucun point dans ce rectangle.")

    def delete_selected(self) -> None:
        sel = set(self.app.sel)
        if not sel:
            self.app.status("Sélectionne d'abord des scans dans la liste à gauche.")
            return
        if messagebox.askyesno("Supprimer", f"Supprimer {len(sel)} scan(s) du projet ? (Ctrl+Z pour annuler)"):
            self.app.apply(P.remove(self.project, sel), f"{len(sel)} scan(s) supprimé(s).")

    def on_key(self, key: str) -> None:
        if key == "Delete":
            self.delete_selected()

    # ------------------------------------------------------------------ carte
    def render(self) -> None:
        app, f = self.app, self.filters()
        caps = self.project.visible
        kept, out, iso = [], [], []
        total = 0
        for c in caps:
            raw = c.scan.filtered(0.0001, 1e9, 0)
            total += len(raw)
            g = raw.global_points()
            bad = (raw.ranges_m < f.min_range_m) | (raw.ranges_m > f.max_range_m) | (raw.intensities < f.min_intensity)
            out.append(g[bad])
            if f.min_neighbors > 0:
                iso.append(f.rejected(c.scan).global_points())
        pts, poses = app.scan_layers(caps, big=False)
        rr = np.vstack(out) if out else np.empty((0, 2))
        ri = np.vstack(iso) if iso else np.empty((0, 2))
        app.map.draw_analysis(None, None, [], [], None)
        app.map.draw_overlay(pts + [(rr, S.DANGER, 18, "x"), (ri, S.WARN, 18, "x")], [], poses)
        nkept = sum(len(p[0]) for p in pts)
        no_int = bool(caps) and all(np.isnan(c.scan.intensities).all() for c in caps)
        self.int_note.configure(text="Ces scans n'ont pas d'intensité (Teleplot) : ce réglage est sans effet."
                                if no_int else "", text_color=S.WARN)
        self.stats.show([("Points avec écho", str(total)), ("Gardés", str(nkept)),
                         ("Hors portée / intensité", str(len(rr))), ("Isolés", str(len(ri))),
                         ("Écartés", f"{100 * (1 - nkept / max(total, 1)):.1f} %")])
        self.legend("✕ rouge = hors portée ou intensité · ✕ orange = isolé · Suppr = supprimer les scans sélectionnés")
