"""Petits composants d'interface réutilisés par LidarScope (CustomTkinter)."""

from __future__ import annotations

from typing import Callable

import customtkinter as ctk

from . import theme as T


def font(size: int = 12, weight: str = "normal", mono: bool = False) -> ctk.CTkFont:
    return ctk.CTkFont(family=T.MONO if mono else T.FONT, size=size, weight=weight)


def section(master, title: str) -> ctk.CTkFrame:
    """Bloc à fond légèrement plus clair avec un petit titre en majuscules."""
    ctk.CTkLabel(master, text=title.upper(), font=font(10, "bold"), text_color=T.MUTED,
                 anchor="w").pack(fill="x", padx=4, pady=(12, 4))
    box = ctk.CTkFrame(master, fg_color=T.SURFACE_2, corner_radius=10)
    box.pack(fill="x", padx=2)
    return box


def button(master, text: str, command: Callable, primary: bool = False, **kw) -> ctk.CTkButton:
    if primary:
        style = dict(fg_color=T.ACCENT, hover_color=T.ACCENT_HOVER, text_color="#06120b", font=font(13, "bold"))
    else:
        style = dict(fg_color=T.SURFACE_2, hover_color=T.BORDER, text_color=T.INK, border_width=1,
                     border_color=T.BORDER, font=font(12))
    style.update(kw)
    return ctk.CTkButton(master, text=text, command=command, corner_radius=8, **style)


class StatCard(ctk.CTkFrame):
    """Une valeur mise en avant (gros chiffre + libellé)."""

    def __init__(self, master, label: str) -> None:
        super().__init__(master, fg_color=T.SURFACE_2, corner_radius=10)
        self.value = ctk.CTkLabel(self, text="—", font=font(22, "bold", mono=True), text_color=T.INK)
        self.value.pack(anchor="w", padx=12, pady=(8, 0))
        ctk.CTkLabel(self, text=label, font=font(11), text_color=T.MUTED).pack(anchor="w", padx=12, pady=(0, 8))

    def set(self, text: str, color: str = T.INK) -> None:
        self.value.configure(text=text, text_color=color)


class LabeledSlider(ctk.CTkFrame):
    """Curseur avec libellé et valeur affichée."""

    def __init__(self, master, label: str, from_: float, to: float, value: float,
                 fmt: str, command: Callable[[float], None], steps: int | None = None) -> None:
        super().__init__(master, fg_color="transparent")
        self.fmt, self.command = fmt, command
        top = ctk.CTkFrame(self, fg_color="transparent")
        top.pack(fill="x")
        ctk.CTkLabel(top, text=label, font=font(12), text_color=T.INK).pack(side="left")
        self.val = ctk.CTkLabel(top, text=fmt.format(value), font=font(12, mono=True), text_color=T.ACCENT)
        self.val.pack(side="right")
        self.slider = ctk.CTkSlider(self, from_=from_, to=to, number_of_steps=steps, command=self._changed,
                                    progress_color=T.ACCENT, button_color=T.INK, button_hover_color=T.ACCENT)
        self.slider.set(value)
        self.slider.pack(fill="x", pady=(2, 0))

    def _changed(self, v: float) -> None:
        self.val.configure(text=self.fmt.format(v))
        self.command(v)

    def get(self) -> float:
        return float(self.slider.get())


class PoseEntry(ctk.CTkFrame):
    """Trois champs x / y / θ."""

    def __init__(self, master) -> None:
        super().__init__(master, fg_color="transparent")
        self.vars = []
        for k, (name, unit) in enumerate((("x", "m"), ("y", "m"), ("θ", "°"))):
            ctk.CTkLabel(self, text=name, font=font(12, "bold"), text_color=T.MUTED).grid(row=0, column=3 * k, padx=(0, 4))
            var = ctk.StringVar(value="0")
            ctk.CTkEntry(self, textvariable=var, width=62, font=font(12, mono=True), fg_color=T.SURFACE,
                         border_color=T.BORDER).grid(row=0, column=3 * k + 1)
            ctk.CTkLabel(self, text=unit, font=font(11), text_color=T.MUTED).grid(row=0, column=3 * k + 2, padx=(2, 10))
            self.vars.append(var)

    def get(self) -> tuple[float, float, float]:
        return tuple(float(v.get().replace(",", ".")) for v in self.vars)

    def set(self, x: float, y: float, theta: float) -> None:
        for var, v in zip(self.vars, (x, y, theta)):
            var.set(f"{v:.3f}".rstrip("0").rstrip(".") if v else "0")
