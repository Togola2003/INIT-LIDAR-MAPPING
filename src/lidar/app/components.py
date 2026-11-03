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


class PilotPad(ctk.CTkFrame):
    """Pilotage du robot en Wi-Fi : croix de déplacement, rotations, arrêt, durée d'un mouvement.
    Clavier (quand le pavé est affiché) : I/K avancer/reculer, J/L gauche/droite, U/O tourner, Espace stop."""

    KEYS = {"i": "forward", "k": "backward", "j": "left", "l": "right", "u": "rotateleft", "o": "rotateright"}

    def __init__(self, master, get_client: Callable[[], object], on_status: Callable[[str], None]) -> None:
        super().__init__(master, fg_color="transparent")
        self.get_client, self.on_status = get_client, on_status
        pad = ctk.CTkFrame(self, fg_color="transparent")
        pad.pack()
        layout = ((0, 0, "⟲", "rotateleft"), (0, 1, "↑", "forward"), (0, 2, "⟳", "rotateright"),
                  (1, 0, "←", "left"), (1, 2, "→", "right"), (2, 1, "↓", "backward"))
        for r, c, txt, action in layout:
            S.button(pad, txt, lambda a=action: self.move(a), width=50, height=42, font=S.font(18)).grid(
                row=r, column=c, padx=3, pady=3)
        S.button(pad, "■", self.stop, "danger", width=50, height=42, font=S.font(16)).grid(row=1, column=1, padx=3, pady=3)
        self.duration = S.Slider(self, "Durée d'un mouvement", 100, 2000, 400, "{:.0f} ms", lambda v: None, steps=19)
        self.duration.pack(fill="x", pady=(8, 0))
        S.hint(self, "Clavier : I K (avancer, reculer) · J L (côtés) · U O (tourner) · Espace = stop")

    def move(self, action: str) -> None:
        client = self.get_client()
        if client is None:
            self.on_status("Pilotage impossible : le robot n'est pas connecté en Wi-Fi (étape 1).")
            return
        if not client.move(action, int(self.duration.get())):
            self.on_status("Le robot exécute encore la commande précédente.")
        elif client.last_error:
            self.on_status(f"Commande non reçue : {client.last_error}")

    def stop(self) -> None:
        client = self.get_client()
        if client is not None:
            client.stop()
            self.on_status("Arrêt demandé.")

    def key(self, keysym: str) -> bool:
        k = keysym.lower()
        if k == "space":
            self.stop()
            return True
        if k in self.KEYS:
            self.move(self.KEYS[k])
            return True
        return False
