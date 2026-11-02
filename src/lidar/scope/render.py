"""Rendus PNG « hors écran » (sans fenêtre) : utilisés par les exports."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from ..segments import Segment
from .mapping import MapResult
from .session import Capture

BG, AX_BG, INK, MUTED, GRID = "#ffffff", "#ffffff", "#1f2328", "#57606a", "#d0d7de"


def render_png(path: str | Path, captures: list[Capture], result: MapResult | None, model: str,
               title: str, reference: list[Segment] | None = None, show_points: bool = True) -> Path:
    """Carte sur fond blanc (lisible imprimée dans un compte rendu)."""
    fig = Figure(figsize=(10, 8), facecolor=BG)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, facecolor=AX_BG)
    img = result.grid_image(model) if result else None
    if img is not None:
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=result.grid.extent(),
                  interpolation="nearest", zorder=0)
    for seg in reference or []:
        ax.plot([seg.x1, seg.x2], [seg.y1, seg.y2], "--", color="#8250df", lw=1.5, zorder=1)
    if show_points:
        for c in captures:
            pts = c.scan.filtered().global_points()
            ax.scatter(pts[:, 0], pts[:, 1], s=4, c=c.color, label=c.title, zorder=2, linewidths=0)
    for seg in (result.segments if result else []):
        ax.plot([seg.x1, seg.x2], [seg.y1, seg.y2], "-", color="#cf222e", lw=2.2, zorder=3)
    for c in captures:
        p = c.scan.pose
        ax.plot(p.x, p.y, "^", color=c.color, ms=11, mec="black", zorder=4)
        ax.plot([p.x, p.x + 0.3 * np.cos(p.theta)], [p.y, p.y + 0.3 * np.sin(p.theta)], "-", color="black", lw=1.5, zorder=4)
    ax.set_aspect("equal")
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_xlabel("x (m)", color=MUTED)
    ax.set_ylabel("y (m)", color=MUTED)
    ax.set_title(title, color=INK, fontsize=13)
    if show_points and captures:
        ax.legend(loc="upper right", fontsize=8, markerscale=3)
    legend = "Fond : blanc = libre, noir = occupé, gris = inconnu · rouge = segments · ▲ = robot"
    if reference:
        legend += " · violet pointillé = carte de référence"
    fig.text(0.5, 0.01, legend, ha="center", fontsize=8, color=MUTED)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, facecolor=BG, bbox_inches="tight")
    return path
