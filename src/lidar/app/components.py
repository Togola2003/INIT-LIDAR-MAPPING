"""Composants partagés de LidarLab (au-delà des briques de style.py)."""

from __future__ import annotations

from typing import Callable

import customtkinter as ctk

from . import style as S


class ScanList(ctk.CTkScrollableFrame):
    """Liste compacte des scans du projet : pastille de couleur, nom, pose, œil (visible ou non).
    Clic = sélectionner, Ctrl + clic = ajouter / retirer de la sélection."""

    def __init__(self, master, on_select: Callable[[int, bool], None], on_toggle: Callable[[int], None]) -> None:
        super().__init__(master, fg_color="transparent", scrollbar_button_color=S.BORDER)
        self.on_select, self.on_toggle = on_select, on_toggle
        self._sig = None
        self.empty = ctk.CTkLabel(self, text="Aucun scan.\nÉtape 1 : récolter ou importer.", font=S.font(12),
                                  text_color=S.FAINT, justify="center")
        self.empty.pack(pady=24)

    def show(self, captures, sel: set[int], primary: int | None) -> None:
        sig = (tuple((c.number, c.visible, c.scan.pose, c.label) for c in captures), frozenset(sel), primary)
        if sig == self._sig:
            return
        self._sig = sig
        for w in self.winfo_children():
            if w is not self.empty:
                w.destroy()
        if not captures:
            self.empty.pack(pady=24)
            return
        self.empty.pack_forget()
        for c in captures:
            on = c.number in sel
            row = ctk.CTkFrame(self, fg_color=S.ACCENT_SOFT if on else "transparent", corner_radius=9, height=40)
            row.pack(fill="x", pady=1)
            dot = ctk.CTkLabel(row, text="●", font=S.font(16), width=18,
                               text_color=c.color if c.visible else S.FAINT)
            dot.pack(side="left", padx=(8, 6))
            box = ctk.CTkFrame(row, fg_color="transparent")
            box.pack(side="left", fill="x", expand=True, pady=4)
            name = c.title if len(c.title) <= 26 else c.title[:25] + "…"
            weight = "bold" if c.number == primary else "normal"
            t1 = ctk.CTkLabel(box, text=name, font=S.font(12, weight), text_color=S.INK if c.visible else S.FAINT,
                              anchor="w")
            t1.pack(fill="x")
            p = c.scan.pose
            t2 = ctk.CTkLabel(box, text=f"{p.x:+.2f}  {p.y:+.2f}  {p.theta_deg:+.0f}°", font=S.font(10, mono=True),
                              text_color=S.FAINT, anchor="w")
            t2.pack(fill="x")
            eye = ctk.CTkButton(row, text="◉" if c.visible else "○", width=26, height=24, corner_radius=8,
                                fg_color="transparent", hover_color=S.BORDER, text_color=S.MUTED, font=S.font(13),
                                command=lambda n=c.number: self.on_toggle(n))
            eye.pack(side="right", padx=6)
            for w in (row, dot, box, t1, t2):
                w.bind("<Button-1>", lambda e, n=c.number: self.on_select(n, False))
                w.bind("<Control-Button-1>", lambda e, n=c.number: self.on_select(n, True))
