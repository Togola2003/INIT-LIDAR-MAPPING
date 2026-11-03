"""Style de LidarLab : clair, aéré, une seule couleur d'accent (inspiré des interfaces d'Apple).

Tout ce qui touche à l'apparence est ici : couleurs, polices et petits composants réutilisés.
Changer le thème = changer ce fichier, sans toucher aux étapes.
"""

from __future__ import annotations

from typing import Callable

import customtkinter as ctk

# ---------------------------------------------------------------------- couleurs
BG = "#F5F5F7"            # fond de la fenêtre
SURFACE = "#FFFFFF"       # cartes
SURFACE_2 = "#F2F2F7"     # champs, zones secondaires
BORDER = "#E5E5EA"
INK = "#1D1D1F"
MUTED = "#6E6E73"
FAINT = "#AEAEB2"
ACCENT = "#007AFF"
ACCENT_HOVER = "#0066D6"
ACCENT_SOFT = "#E8F1FF"
SUCCESS = "#34C759"
WARN = "#FF9500"
DANGER = "#FF3B30"

# couleurs de la carte (lues par MapView via theme=…)
MAP_BG = "#FFFFFF"
GRID = "#F0F0F3"
LIVE = "#1D1D1F"
ROBOT = "#FF3B30"
SEGMENT = "#FF3B30"
REFERENCE = "#AF52DE"
INITIAL = "#FF9500"
FOUND = "#34C759"

FONT = "Segoe UI Variable Text"
FONT_DISPLAY = "Segoe UI Variable Display"
MONO = "Cascadia Mono"
RADIUS = 14


def font(size: int = 13, weight: str = "normal", display: bool = False, mono: bool = False) -> ctk.CTkFont:
    family = MONO if mono else FONT_DISPLAY if display else FONT
    return ctk.CTkFont(family=family, size=size, weight=weight)


# ---------------------------------------------------------------------- composants
class Card(ctk.CTkFrame):
    """Bloc blanc arrondi avec un titre et, en option, une phrase d'explication."""

    def __init__(self, master, title: str = "", subtitle: str = "", **kw) -> None:
        super().__init__(master, fg_color=SURFACE, corner_radius=RADIUS, border_width=1, border_color=BORDER, **kw)
        if title:
            ctk.CTkLabel(self, text=title, font=font(14, "bold"), text_color=INK, anchor="w").pack(
                fill="x", padx=16, pady=(14, 0))
        if subtitle:
            ctk.CTkLabel(self, text=subtitle, font=font(11), text_color=MUTED, anchor="w", justify="left",
                         wraplength=320).pack(fill="x", padx=16, pady=(2, 0))
        self.body = ctk.CTkFrame(self, fg_color="transparent")
        self.body.pack(fill="x", padx=16, pady=(10, 14))


def button(master, text: str, command: Callable, kind: str = "secondary", **kw) -> ctk.CTkButton:
    styles = {
        "primary": dict(fg_color=ACCENT, hover_color=ACCENT_HOVER, text_color="#FFFFFF", font=font(13, "bold")),
        "secondary": dict(fg_color=SURFACE_2, hover_color=BORDER, text_color=INK, font=font(13)),
        "ghost": dict(fg_color="transparent", hover_color=SURFACE_2, text_color=ACCENT, font=font(13)),
        "danger": dict(fg_color="#FFEBEA", hover_color="#FFD6D3", text_color=DANGER, font=font(13)),
    }
    style = {**styles[kind], "height": 34, "corner_radius": 10}
    style.update(kw)
    return ctk.CTkButton(master, text=text, command=command, **style)


def segmented(master, values: list[str], value: str, command: Callable[[str], None]) -> ctk.CTkSegmentedButton:
    s = ctk.CTkSegmentedButton(master, values=values, command=command, font=font(12), height=30,
                               fg_color=SURFACE_2, selected_color=SURFACE, selected_hover_color=SURFACE,
                               unselected_color=SURFACE_2, unselected_hover_color=BORDER, text_color=INK,
                               corner_radius=9)
    s.set(value)
    return s


def toggle(master, text: str, command: Callable[[], None], on: bool = False) -> ctk.CTkSwitch:
    sw = ctk.CTkSwitch(master, text=text, font=font(12), text_color=INK, progress_color=SUCCESS,
                       button_color=SURFACE, button_hover_color=SURFACE, fg_color=BORDER, command=command)
    if on:
        sw.select()
    return sw


def hint(master, text: str, color: str = MUTED, size: int = 11) -> ctk.CTkLabel:
    lab = ctk.CTkLabel(master, text=text, font=font(size), text_color=color, anchor="w", justify="left",
                       wraplength=320)
    lab.pack(fill="x", pady=(2, 4))
    return lab


class Slider(ctk.CTkFrame):
    def __init__(self, master, label: str, from_: float, to: float, value: float, fmt: str,
                 command: Callable[[float], None], steps: int | None = None) -> None:
        super().__init__(master, fg_color="transparent")
        self.fmt, self.command = fmt, command
        top = ctk.CTkFrame(self, fg_color="transparent")
        top.pack(fill="x")
        ctk.CTkLabel(top, text=label, font=font(12), text_color=INK).pack(side="left")
        self.val = ctk.CTkLabel(top, text=fmt.format(value), font=font(12, mono=True), text_color=MUTED)
        self.val.pack(side="right")
        self.slider = ctk.CTkSlider(self, from_=from_, to=to, number_of_steps=steps, command=self._changed, height=16,
                                    progress_color=ACCENT, button_color=SURFACE, button_hover_color=SURFACE,
                                    fg_color=BORDER, button_corner_radius=10, border_width=0)
        self.slider.set(value)
        self.slider.pack(fill="x", pady=(4, 6))

    def _changed(self, v: float) -> None:
        self.val.configure(text=self.fmt.format(v))
        self.command(v)

    def get(self) -> float:
        return float(self.slider.get())

    def set(self, v: float) -> None:
        self.slider.set(v)
        self.val.configure(text=self.fmt.format(v))


class PoseField(ctk.CTkFrame):
    """Trois champs x / y / θ."""

    def __init__(self, master) -> None:
        super().__init__(master, fg_color="transparent")
        self.vars = []
        for k, (name, unit) in enumerate((("x", "m"), ("y", "m"), ("θ", "°"))):
            ctk.CTkLabel(self, text=name, font=font(12, "bold"), text_color=MUTED).grid(row=0, column=3 * k, padx=(0, 4))
            var = ctk.StringVar(value="0")
            ctk.CTkEntry(self, textvariable=var, width=66, height=30, font=font(12, mono=True), fg_color=SURFACE_2,
                         border_color=BORDER, border_width=1, text_color=INK, corner_radius=8).grid(row=0, column=3 * k + 1)
            ctk.CTkLabel(self, text=unit, font=font(11), text_color=MUTED).grid(row=0, column=3 * k + 2, padx=(3, 10))
            self.vars.append(var)

    def get(self) -> tuple[float, float, float]:
        return tuple(float(v.get().replace(",", ".")) for v in self.vars)

    def set(self, x: float, y: float, theta: float) -> None:
        for var, v in zip(self.vars, (x, y, theta)):
            var.set(f"{v:.3f}".rstrip("0").rstrip(".") if abs(v) > 1e-9 else "0")


class KeyValues(ctk.CTkFrame):
    """Petit tableau « libellé … valeur » mis à jour d'un coup."""

    def __init__(self, master) -> None:
        super().__init__(master, fg_color="transparent")
        self.grid_columnconfigure(1, weight=1)
        self._labels: list = []

    def show(self, rows: list[tuple[str, str]], highlight: str | None = None) -> None:
        for w in self._labels:
            w.destroy()
        self._labels = []
        for r, (k, v) in enumerate(rows):
            a = ctk.CTkLabel(self, text=k, font=font(12), text_color=MUTED, anchor="w")
            b = ctk.CTkLabel(self, text=v, font=font(12, "bold" if k == highlight else "normal", mono=True),
                             text_color=INK, anchor="e")
            a.grid(row=r, column=0, sticky="w", pady=1)
            b.grid(row=r, column=1, sticky="e", pady=1)
            self._labels += [a, b]
