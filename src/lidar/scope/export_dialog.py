"""Fenêtre « Exporter… » : on choisit la portée (chaque scan / ensemble / sélection) et le contenu."""

from __future__ import annotations

from tkinter import filedialog
from typing import Callable

import customtkinter as ctk

from . import theme as T
from .exporter import ITEMS, SCOPES, ExportOptions
from .widgets import button, font


class ExportDialog(ctk.CTkToplevel):
    def __init__(self, master, n_captures: int, has_selection: bool,
                 on_export: Callable[[str, ExportOptions], None]) -> None:
        super().__init__(master, fg_color=T.SURFACE)
        self.title("Exporter")
        self.geometry("520x610")
        self.resizable(False, False)
        self.transient(master)
        self.on_export = on_export

        ctk.CTkLabel(self, text="Exporter les données et les cartes", font=font(17, "bold"),
                     text_color=T.INK).pack(anchor="w", padx=20, pady=(18, 2))
        ctk.CTkLabel(self, text=f"{n_captures} capture(s) incluse(s) · un dossier daté est créé, avec un "
                     "LISEZMOI.md qui explique chaque fichier.", font=font(11), text_color=T.MUTED,
                     wraplength=470, justify="left").pack(anchor="w", padx=20)

        ctk.CTkLabel(self, text="PORTÉE", font=font(10, "bold"), text_color=T.MUTED).pack(anchor="w", padx=20, pady=(16, 4))
        self.scope = ctk.StringVar(value="chaque+ensemble")
        for key, label in SCOPES.items():
            rb = ctk.CTkRadioButton(self, text=label, value=key, variable=self.scope, font=font(12),
                                    fg_color=T.ACCENT, hover_color=T.ACCENT_HOVER)
            if key == "selection" and not has_selection:
                rb.configure(state="disabled")
            rb.pack(anchor="w", padx=28, pady=3)

        ctk.CTkLabel(self, text="CONTENU", font=font(10, "bold"), text_color=T.MUTED).pack(anchor="w", padx=20, pady=(16, 4))
        self.items = {}
        for key, label in ITEMS.items():
            var = ctk.BooleanVar(value=True)
            ctk.CTkCheckBox(self, text=label, variable=var, font=font(12), fg_color=T.ACCENT,
                            hover_color=T.ACCENT_HOVER).pack(anchor="w", padx=28, pady=3)
            self.items[key] = var

        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.pack(fill="x", side="bottom", padx=20, pady=18)
        button(bar, "Annuler", self.destroy, width=110).pack(side="right")
        button(bar, "Choisir le dossier et exporter", self._go, primary=True).pack(side="right", padx=8)
        self.after(100, self.grab_set)

    def _go(self) -> None:
        items = frozenset(k for k, v in self.items.items() if v.get())
        if not items:
            return
        folder = filedialog.askdirectory(parent=self, title="Dossier où créer l'export", initialdir="out")
        if folder:
            self.destroy()
            self.on_export(folder, ExportOptions(self.scope.get(), items))
