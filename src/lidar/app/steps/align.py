"""Étape 3 · Recaler : placer chaque scan dans la carte, à la main (souris, clavier) ou automatiquement."""

from __future__ import annotations

import customtkinter as ctk
import numpy as np

from ...geometry import Pose, move_rigid
from ...matching import align_best, align_sequence, consistency, icp, voxel_downsample
from .. import project as P
from .. import style as S
from .base import Step

NUDGES = {"Fin": (0.01, 0.5), "Normal": (0.05, 2.0), "Gros": (0.20, 5.0)}
LOW_OVERLAP = 0.30


class AlignStep(Step):
    title = "Recaler"
    subtitle = "placer les scans"
    headline = "Glisse un scan pour le déplacer, clic droit glissé pour le tourner. Ctrl+clic : plusieurs."
    course = ("Les poses mesurées au mètre sont imprécises : les murs apparaissent doublés. On corrige chaque "
              "pose, à la main ou par scan matching (ICP : associer chaque point au plus proche, puis calculer "
              "la rotation + translation qui superpose au mieux). Le recalage global corrige toutes les poses "
              "ensemble pour que l'erreur ne s'accumule pas.")

    def build(self, panel) -> None:
        card = S.Card(panel, "Sélection", "Flèches = déplacer · A / E = tourner · Ctrl+molette = ±1°")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.sel_label = ctk.CTkLabel(b, text="Aucun scan sélectionné", font=S.font(13, "bold"), text_color=S.MUTED,
                                      anchor="w")
        self.sel_label.pack(fill="x")
        self.step_size = S.segmented(b, list(NUDGES), "Normal", lambda v: None)
        self.step_size.pack(fill="x", pady=8)
        pad = ctk.CTkFrame(b, fg_color="transparent")
        pad.pack()
        for r, c, txt, d in ((0, 1, "↑", (0, 1, 0)), (1, 0, "←", (-1, 0, 0)), (1, 2, "→", (1, 0, 0)),
                             (2, 1, "↓", (0, -1, 0)), (0, 0, "⟲", (0, 0, 1)), (0, 2, "⟳", (0, 0, -1))):
            S.button(pad, txt, lambda d=d: self.nudge(*d), width=46, height=38, font=S.font(16)).grid(
                row=r, column=c, padx=3, pady=3)
        self.pose_field = S.PoseField(b)
        self.pose_field.pack(anchor="w", pady=(10, 0))
        S.button(b, "Appliquer la pose au scan principal", self.apply_pose, "ghost").pack(fill="x", pady=(6, 0))

        card = S.Card(panel, "Recalage automatique", "Le scan principal (en gras dans la liste) reste fixe.")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        S.button(b, "Recalage complet", self.align_full, "primary", height=40).pack(fill="x")
        S.hint(b, "Toutes les poses ensemble (point à droite), depuis deux départs : on garde le meilleur.")
        S.button(b, "Recaler la sélection en bloc (ICP)", self.icp_block).pack(fill="x", pady=(6, 3))
        S.button(b, "ICP scan par scan", self.icp_sequence).pack(fill="x", pady=3)
        S.button(b, "Revenir aux poses d'origine", self.reset_poses, "ghost").pack(fill="x", pady=(3, 0))

        card = S.Card(panel, "Cohérence des scans", "Écart médian entre chaque scan et les autres : plus petit = mieux.")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Mesurer", self.measure).pack(fill="x")
        self.result = S.KeyValues(card.body)
        self.result.pack(fill="x", pady=(8, 0))

    # ------------------------------------------------------------------ cycle
    def enter(self) -> None:
        self.app.map.set_editor(self.app.editor())
        self.on_selection()

    def leave(self) -> None:
        self.app.map.set_editor(None)

    def on_selection(self) -> None:
        cap = self.project.find(self.app.primary)
        if cap is None:
            self.sel_label.configure(text="Aucun scan sélectionné", text_color=S.MUTED)
            return
        more = f"  + {len(self.app.sel) - 1} autre(s)" if len(self.app.sel) > 1 else ""
        self.sel_label.configure(text=f"● {cap.title}{more}", text_color=cap.color)
        p = cap.scan.pose
        self.pose_field.set(p.x, p.y, p.theta_deg)

    on_project = on_selection

    def on_key(self, key: str) -> None:
        moves = {"Left": (-1, 0, 0), "Right": (1, 0, 0), "Up": (0, 1, 0), "Down": (0, -1, 0),
                 "a": (0, 0, 1), "e": (0, 0, -1)}
        if key in moves:
            self.nudge(*moves[key])

    # ------------------------------------------------------------------ manuel
    def nudge(self, dx, dy, dth) -> None:
        if not self.app.sel:
            self.app.status("Sélectionne un scan (clic sur la carte ou dans la liste).")
            return
        m, d = NUDGES[self.step_size.get()]
        sel = set(self.app.sel)
        self.app.apply(P.move(self.project, sel, dx * m, dy * m, dth * d, P.pivot_of(self.project, sel)))

    def apply_pose(self) -> None:
        cap = self.project.find(self.app.primary)
        try:
            pose = Pose(*self.pose_field.get())
        except ValueError:
            self.app.status("x, y et θ doivent être des nombres.")
            return
        if cap:
            self.app.apply(P.set_poses(self.project, {cap.number: pose}))

    def reset_poses(self) -> None:
        orig = {c.number: self.app.originals[c.number] for c in self.project.captures if c.number in self.app.originals}
        self.app.apply(P.set_poses(self.project, orig), "Poses d'origine restaurées (Ctrl+Z pour annuler).")

    # ------------------------------------------------------------------ automatique
    def _caps(self):
        caps = self.project.visible
        if len(caps) < 2:
            self.app.status("Il faut au moins deux scans visibles.")
            return None, 0
        anchor = next((k for k, c in enumerate(caps) if c.number == self.app.primary), 0)
        return caps, anchor

    def _local(self, c) -> np.ndarray:
        return self.app.filtered(c).local_points()

    def align_full(self) -> None:
        caps, anchor = self._caps()
        if not caps:
            return
        self.app.status("Recalage complet en cours…")
        self.app.root.update_idletasks()
        res, how = align_best([self._local(c) for c in caps], [c.scan.pose for c in caps], anchor)
        self.app.apply(P.set_poses(self.project, {c.number: p for c, p in zip(caps, res.poses)}),
                       f"Recalage complet ({how}) : écart entre scans {res.median_before * 100:.1f} → "
                       f"{res.median_after * 100:.1f} cm. Ctrl+Z pour annuler.")
        self._show_gaps(caps, res.gap_after_m)

    def icp_sequence(self) -> None:
        caps, anchor = self._caps()
        if not caps:
            return
        out = align_sequence([self._local(c) for c in caps], [c.scan.pose for c in caps], anchor)
        self.app.apply(P.set_poses(self.project, {c.number: p for c, (p, _) in zip(caps, out)}),
                       "ICP scan par scan terminé. Ctrl+Z pour annuler.")

    def icp_block(self) -> None:
        group = [c for c in self.project.visible if c.number in self.app.sel]
        others = [c for c in self.project.visible if c.number not in self.app.sel]
        if not group or not others:
            self.app.status("Sélectionne un ou plusieurs scans, en laissant au moins un autre scan visible.")
            return
        ref = voxel_downsample(np.vstack([self.app.points(c) for c in others]), 0.03)
        block = voxel_downsample(np.vstack([self.app.points(c) for c in group]), 0.02)
        res = icp(block, Pose(), ref)
        if not res.converged:
            self.app.status("ICP impossible : pas assez de points en commun.")
            return
        t = res.pose
        moves = {c.number: move_rigid(c.scan.pose, t.x, t.y, t.theta_deg, (0.0, 0.0)) for c in group}
        warn = " ⚠ recouvrement faible, vérifie." if res.inlier_ratio < LOW_OVERLAP else ""
        self.app.apply(P.set_poses(self.project, moves),
                       f"Sélection recalée en bloc : erreur résiduelle {res.rms_m * 100:.1f} cm.{warn}")

    def measure(self) -> None:
        caps, _ = self._caps()
        if caps:
            self._show_gaps(caps, consistency([self._local(c) for c in caps], [c.scan.pose for c in caps]))

    def _show_gaps(self, caps, gaps) -> None:
        gaps = np.asarray(gaps)
        k = int(np.nanargmax(gaps))
        self.result.show([("Écart médian", f"{np.nanmedian(gaps) * 100:.1f} cm"),
                          ("Pire scan", f"{caps[k].title} ({gaps[k] * 100:.1f} cm)"),
                          ("Scans", str(len(caps)))], highlight="Écart médian")

    # ------------------------------------------------------------------ carte
    def render(self) -> None:
        pts, poses = self.app.scan_layers()
        self.app.map.draw_analysis(None, None, [], [], None)
        self.app.map.draw_overlay(pts, [], poses)
        self.legend("gros points = sélection · ▲ = pose du robot · ↶ Ctrl+Z annule chaque geste")
