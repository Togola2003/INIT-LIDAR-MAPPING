"""Onglet « Cartographie » : les réglages suivent les étapes des documents du cours."""

from __future__ import annotations

from typing import Callable

import customtkinter as ctk

from . import theme as T
from .mapping import GridParams, MapParams, VectorParams
from .processing import Filters
from .widgets import LabeledSlider, font, section

RESOLUTIONS = {"2 cm": 0.02, "5 cm": 0.05, "10 cm": 0.10}


def _seg(master, values: list[str], value: str, command) -> ctk.CTkSegmentedButton:
    s = ctk.CTkSegmentedButton(master, values=values, command=lambda _: command(), selected_color=T.ACCENT,
                               selected_hover_color=T.ACCENT_HOVER, unselected_color=T.SURFACE)
    s.set(value)
    return s


def _switch(master, text: str, command, on: bool = False) -> ctk.CTkSwitch:
    sw = ctk.CTkSwitch(master, text=text, font=font(12), progress_color=T.ACCENT, command=command)
    if on:
        sw.select()
    return sw


def _hint(master, text: str) -> None:
    ctk.CTkLabel(master, text=text, font=font(10), text_color=T.MUTED, justify="left",
                 wraplength=320).pack(anchor="w", padx=12, pady=(0, 8))


class CartoPanel:
    """Construit l'onglet et expose les réglages (MapParams) + les couches à afficher."""

    def __init__(self, tab, on_change: Callable[[], None], on_filters: Callable[[], None]) -> None:
        self.on_change = on_change
        ch, fl = self._changed, self._filters_changed
        self._on_filters = on_filters
        self._job = None
        self.tab = tab

        box = section(tab, "Carte construite à partir de")
        self.scope = _seg(box, ["Toutes les captures", "Capture sélectionnée"], "Toutes les captures", ch)
        self.scope.pack(padx=12, pady=(10, 4))
        _hint(box, "Inclure / exclure une capture : œil 👁 dans l'onglet Captures. "
                   "Importer d'autres données : onglet Données.")

        box = section(tab, "① Prétraitement (filtrer les points aberrants)")
        self.f_min = LabeledSlider(box, "Portée minimale", 0, 1, 0.05, "{:.2f} m", fl)
        self.f_max = LabeledSlider(box, "Portée maximale", 0.5, 12, 12, "{:.1f} m", fl)
        self.f_int = LabeledSlider(box, "Intensité minimale", 0, 255, 0, "{:.0f}", fl, steps=255)
        for s in (self.f_min, self.f_max, self.f_int):
            s.pack(fill="x", padx=12, pady=5)
        _hint(box, "Les exports gardent toujours les mesures brutes ; les réglages y sont notés.")

        box = section(tab, "② Grid map (carte d'occupation)")
        self.grid_on = _switch(box, "Construire la grid map", ch, on=True)
        self.grid_on.pack(anchor="w", padx=12, pady=(10, 6))
        rows = (("Résolution", "res", list(RESOLUTIONS), "5 cm"),
                ("État initial", "init", ["inconnu", "libre", "occupé"], "inconnu"),
                ("Modèle", "model", ["probabiliste", "binaire"], "probabiliste"))
        for label, attr, values, val in rows:
            ctk.CTkLabel(box, text=label, font=font(11), text_color=T.MUTED).pack(anchor="w", padx=12)
            w = _seg(box, values, val, ch)
            w.pack(anchor="w", padx=12, pady=(0, 6))
            setattr(self, attr, w)
        self.p_occ = LabeledSlider(box, "p(occupé | impact)", 0.55, 0.95, 0.70, "{:.2f}", lambda v: self._changed())
        self.p_free = LabeledSlider(box, "p(occupé | faisceau traverse)", 0.05, 0.45, 0.40, "{:.2f}",
                                    lambda v: self._changed())
        for s in (self.p_occ, self.p_free):
            s.pack(fill="x", padx=12, pady=4)
        self.isolated = _switch(box, "Nettoyage : supprimer les cellules isolées", ch)
        self.isolated.pack(anchor="w", padx=12, pady=6)
        self.beams = _switch(box, "Montrer cellules traversées / d'impact (capture sélectionnée)", ch)
        self.beams.pack(anchor="w", padx=12, pady=(0, 4))
        _hint(box, "Bleu = cellules traversées par le faisceau (libres), rouge = cellules d'impact (occupées).")

        box = section(tab, "③ Carte vectorielle (segments)")
        self.vec_on = _switch(box, "Extraire les segments (murs)", ch, on=True)
        self.vec_on.pack(anchor="w", padx=12, pady=(10, 6))
        self.jump = LabeledSlider(box, "Saut qui sépare deux objets", 0.02, 0.30, 0.05, "{:.2f} m",
                                  lambda v: self._changed())
        self.split = LabeledSlider(box, "Tolérance point-droite (split)", 0.01, 0.10, 0.03, "{:.2f} m",
                                   lambda v: self._changed())
        self.min_len = LabeledSlider(box, "Longueur minimale", 0.05, 0.60, 0.10, "{:.2f} m", lambda v: self._changed())
        self.merge = LabeledSlider(box, "Fusion si angle ≤", 2, 25, 10, "{:.0f}°", lambda v: self._changed())
        for s in (self.jump, self.split, self.min_len, self.merge):
            s.pack(fill="x", padx=12, pady=4)

        box = section(tab, "Couches affichées")
        self.show_points = _switch(box, "Points des captures", ch, on=True)
        self.show_poses = _switch(box, "Poses du robot", ch, on=True)
        self.show_ref = _switch(box, "Carte de référence importée (MapTool)", ch, on=True)
        for sw in (self.show_points, self.show_poses, self.show_ref):
            sw.pack(anchor="w", padx=12, pady=4)

        box = section(tab, "Résultat")
        self.result = ctk.CTkLabel(box, text="—", font=font(11, mono=True), text_color=T.INK, justify="left")
        self.result.pack(anchor="w", padx=12, pady=8)

    # ------------------------------------------------------------------
    def _changed(self) -> None:
        """Les curseurs envoient beaucoup d'événements : on attend 250 ms de calme avant de recalculer."""
        if self._job:
            self.tab.after_cancel(self._job)
        self._job = self.tab.after(250, self._fire)

    def _fire(self) -> None:
        self._job = None
        self.on_change()

    def _filters_changed(self, _v: float) -> None:
        self._on_filters()
        self._changed()

    @property
    def only_selected(self) -> bool:
        return self.scope.get() == "Capture sélectionnée"

    def filters(self) -> Filters:
        lo = self.f_min.get()
        return Filters(lo, max(self.f_max.get(), lo + 0.01), int(self.f_int.get()))

    def params(self) -> MapParams:
        return MapParams(
            filters=self.filters(),
            grid=GridParams(bool(self.grid_on.get()), RESOLUTIONS[self.res.get()], self.init.get(),
                            self.model.get(), round(self.p_occ.get(), 3), round(self.p_free.get(), 3),
                            bool(self.isolated.get())),
            vector=VectorParams(bool(self.vec_on.get()), round(self.jump.get(), 3), round(self.split.get(), 3),
                                round(self.min_len.get(), 3), round(self.merge.get(), 1)),
        )

    def show_result(self, stats: dict) -> None:
        lines = []
        if stats:
            lines.append(f"{stats['scans']} scan(s), {stats['points']} points")
        if "grid" in stats:
            g = stats["grid"]
            lines += [f"Grid map  {g['cells']} cellules", f"  occupées   {g['occupied']}",
                      f"  libres     {g['free']}", f"  inconnues  {g['unknown']}",
                      f"  explorée   {g['explored_area_m2']} m²"]
        if "vector" in stats:
            v = stats["vector"]
            lines += [f"Segments  {v['segments']}", f"  longueur totale  {v['total_length_m']} m",
                      f"  plus long        {v['longest_m']} m"]
        self.result.configure(text="\n".join(lines) or "Aucune capture incluse.")
