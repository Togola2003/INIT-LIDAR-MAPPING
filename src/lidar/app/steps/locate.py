"""Étape 5 · Localiser : retrouver la pose d'un scan sur une référence (grid map, carte vectorielle, scan voisin).

Le scan peut être un scan du projet (sa pose actuelle sert alors de « vérité » pour mesurer l'erreur,
et on peut l'exclure de la carte : c'est le contrôle du compte rendu du Jour 2) ou le scan en direct
reçu du robot (pas de vérité : on obtient sa position).
"""

from __future__ import annotations

import customtkinter as ctk
import numpy as np

from ...geometry import Pose, local_to_global
from ...localization import METHODS, References, localize, pose_error
from ...scope.session import Capture
from ...segments import Segment
from .. import project as P
from .. import style as S
from ..tools import PoseTool
from .base import Step

REFS = {"Grid map": "grid", "Carte vectorielle": "vecteur", "Scan voisin": "scans"}
LIVE = "Scan en direct (robot)"
PERTURB = {"10 cm · 5°": (0.10, 5), "25 cm · 10°": (0.25, 10), "50 cm · 20°": (0.50, 20), "1 m · 30°": (1.0, 30)}
EXPLAIN = {
    "exhaustive": "Teste toutes les poses autour du départ (pas de 5 cm et 1°) : sûr mais lent.",
    "multi-résolution": "Recherche complète sur une carte grossière, puis affinage sur des cartes plus fines.",
    "gradient": "Part du départ et descend la pente de l'erreur (Gauss-Newton) : précis mais il faut partir près.",
    "multi-résolution + gradient": "Robuste puis précis : la méthode retenue au Jour 2 (99 % de réussite).",
    "Gauss-Newton": "Fait baisser la distance des points aux segments de murs, pas à pas.",
    "calcul direct": "Apparie chaque point à son pied sur le mur le plus proche, puis calcule la transformation d'un coup.",
    "multi-résolution + Gauss-Newton": "Recherche grossière sur les murs rastérisés, puis Gauss-Newton exact.",
    "ICP point à point": "Associe chaque point au point voisin le plus proche, puis Kabsch ; on répète.",
    "ICP point à droite": "Distance à la droite du mur voisin : un point peut glisser le long du mur.",
    "multi-résolution + ICP": "Recherche grossière sur le scan voisin rastérisé, puis ICP.",
}


class LocateStep(Step):
    title = "Localiser"
    subtitle = "où est le robot ?"
    headline = "Orange = scan à la pose de départ · vert = scan à la pose trouvée · + = vraie pose (si connue)."
    course = ("Scan matching : on cherche la transformation (x, y, θ) qui superpose au mieux le scan et une "
              "référence. On note une pose (score), puis on cherche la meilleure : recherche exhaustive, "
              "multi-résolution, gradient. Il faut que le scan soit dans la zone couverte par la carte.")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.start: Pose | None = None
        self.result = None
        self._refs_key = None
        self._refs = None

    # ------------------------------------------------------------------ panneau
    def build(self, panel) -> None:
        card = S.Card(panel, "Référence", "Sur quoi aligner le scan ?")
        card.pack(fill="x", pady=(0, 10))
        self.ref = S.segmented(card.body, list(REFS), "Grid map", lambda v: self._ref_changed())
        self.ref.pack(fill="x")
        self.ref_hint = S.hint(card.body, "")

        card = S.Card(panel, "Méthode")
        card.pack(fill="x", pady=(0, 10))
        self.method_var = ctk.StringVar()
        self.method_box = ctk.CTkFrame(card.body, fg_color="transparent")
        self.method_box.pack(fill="x")
        self.method_hint = S.hint(card.body, "", S.INK)

        card = S.Card(panel, "Scan à localiser")
        card.pack(fill="x", pady=(0, 10))
        self.target = ctk.CTkOptionMenu(card.body, values=["—"], height=30, font=S.font(12), fg_color=S.SURFACE_2,
                                        button_color=S.BORDER, button_hover_color=S.FAINT, text_color=S.INK,
                                        dropdown_font=S.font(12), command=lambda v: self._target_changed())
        self.target.pack(fill="x")
        self.exclude = S.toggle(card.body, "Exclure ce scan de la carte (contrôle)", self._invalidate, on=True)
        self.exclude.pack(anchor="w", pady=(8, 0))
        S.hint(card.body, "Sa pose actuelle sert de vérité pour mesurer l'erreur. Pour le scan en direct, il n'y a "
                          "pas de vérité : on obtient simplement la position du robot.")

        card = S.Card(panel, "Pose de départ", "Clique sur la carte, glisse pour l'orientation — ou fausse "
                                              "volontairement la vraie pose pour tester.")
        card.pack(fill="x", pady=(0, 10))
        self.start_field = S.PoseField(card.body)
        self.start_field.pack(anchor="w")
        row = ctk.CTkFrame(card.body, fg_color="transparent")
        row.pack(fill="x", pady=(8, 0))
        self.perturb = ctk.CTkOptionMenu(row, values=list(PERTURB), width=130, height=30, font=S.font(12),
                                         fg_color=S.SURFACE_2, button_color=S.BORDER, button_hover_color=S.FAINT,
                                         text_color=S.INK)
        self.perturb.set("50 cm · 20°")
        self.perturb.pack(side="left")
        S.button(row, "Fausser la vraie pose", self.perturb_truth, "ghost").pack(side="left", padx=6)

        S.button(panel, "Localiser", self.run, "primary", height=44, font=S.font(15, "bold")).pack(fill="x", pady=(0, 6))
        S.button(panel, "Comparer toutes les méthodes", self.compare).pack(fill="x", pady=(0, 10))

        card = S.Card(panel, "Résultat")
        card.pack(fill="x", pady=(0, 10))
        self.out = S.KeyValues(card.body)
        self.out.pack(fill="x")
        self.apply_btn = S.button(card.body, "Appliquer la pose trouvée", self.apply_found, "ghost")
        self.apply_btn.pack(fill="x", pady=(6, 0))
        self.table = ctk.CTkFrame(panel, fg_color=S.SURFACE, corner_radius=S.RADIUS, border_width=1,
                                  border_color=S.BORDER)
        self._ref_changed()

    # ------------------------------------------------------------------ choix
    def _ref_changed(self) -> None:
        kind = REFS[self.ref.get()]
        for w in self.method_box.winfo_children():
            w.destroy()
        names = [m for m in METHODS if m.startswith(kind)]
        best = next((m for m in names if "+" in m), names[0])
        self.method_var.set(best)
        for m in names:
            ctk.CTkRadioButton(self.method_box, text=m.split(" · ")[1], value=m, variable=self.method_var,
                               font=S.font(12), text_color=S.INK, fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER,
                               border_color=S.FAINT, command=self._method_changed).pack(anchor="w", pady=3)
        self.ref_hint.configure(text={"grid": "Distance de chaque point au mur (case occupée) le plus proche.",
                                      "vecteur": "Distance de chaque point au segment de mur le plus proche.",
                                      "scans": "Distance au scan pris le plus près (ses points, ou ses droites)."}[kind])
        self._method_changed()
        self.app.render()

    def _method_changed(self) -> None:
        self.method_hint.configure(text=EXPLAIN[self.method_var.get().split(" · ")[1]])

    def _targets(self) -> dict[str, int | None]:
        out = {c.title: c.number for c in self.project.visible}
        if self.app.source is not None:
            out = {LIVE: None, **out}
        return out

    def on_project(self) -> None:
        names = list(self._targets()) or ["—"]
        self.target.configure(values=names)
        if self.target.get() not in names:
            self.target.set(names[0])
            self._target_changed()

    def _target_changed(self) -> None:
        self.result = None
        cap = self._target_cap()
        if cap is not None and self.start is None:
            self.perturb_truth()
        self.app.render()

    def _invalidate(self) -> None:
        self._refs_key = None
        self.result = None
        self.app.render()

    def _target_cap(self) -> Capture | None:
        num = self._targets().get(self.target.get())
        return self.project.find(num) if num is not None else None

    def _scan(self):
        """(points du scan dans le repère lidar, vraie pose ou None)."""
        cap = self._target_cap()
        if cap is not None:
            f = self.app.filtered(cap)
            return f.local_points(), cap.scan.pose
        frame = self.app.last_frame if self.target.get() == LIVE else None
        if frame is not None:
            return self.app.filters().apply(frame.scan).local_points(), None
        return None, None

    # ------------------------------------------------------------------ pose de départ
    def enter(self) -> None:
        self.on_project()
        self.app.set_tool(PoseTool(self.app.map, self.set_start, S.INITIAL))

    def set_start(self, pose: Pose) -> None:
        self.start = pose
        self.start_field.set(pose.x, pose.y, pose.theta_deg)
        self.result = None
        self.app.render()

    def perturb_truth(self) -> None:
        cap = self._target_cap()
        if cap is None:
            self.app.status("Choisis un scan du projet pour fausser sa pose connue.")
            return
        d, a = PERTURB[self.perturb.get()]
        phi = np.random.default_rng().uniform(0, 2 * np.pi)
        p = cap.scan.pose
        self.set_start(p.moved(d * np.cos(phi), d * np.sin(phi), a * np.random.default_rng().choice([-1, 1])))

    # ------------------------------------------------------------------ calcul
    def references(self) -> References:
        cap = self._target_cap()
        excl = cap.number if (cap is not None and self.exclude.get()) else None
        caps = [c for c in self.project.visible if c.number != excl]
        key = (tuple((c.number, id(c.scan.ranges_m), c.scan.pose) for c in caps), self.project.manual_segments,
               self.app.filters(), self.start)
        if key != self._refs_key:
            scans = [self.app.filtered(c) for c in caps]
            near = None
            if scans and self.start is not None:
                near = min(scans, key=lambda s: np.hypot(s.pose.x - self.start.x, s.pose.y - self.start.y))
            segs = list(self.project.manual_segments) if self.project.manual_segments is not None else None
            self._refs = References.build(scans, segments=segs,
                                          reference_points=near.global_points() if near is not None else None)
            self._refs_key = key
        return self._refs

    def _ready(self) -> tuple:
        try:
            self.start = Pose(*self.start_field.get())
        except ValueError:
            self.app.status("La pose de départ doit contenir des nombres.")
            return None, None, None
        pts, truth = self._scan()
        if pts is None or len(pts) < 20:
            self.app.status("Pas de scan à localiser : choisis un scan du projet ou connecte le robot.")
            return None, None, None
        if len(self.project.visible) - (1 if self.exclude.get() and self._target_cap() else 0) < 1:
            self.app.status("Il faut une carte : au moins un autre scan visible dans le projet.")
            return None, None, None
        return pts, truth, self.references()

    def run(self) -> None:
        pts, truth, refs = self._ready()
        if pts is None:
            return
        method = self.method_var.get()
        self.result = localize(method, pts, self.start, refs)
        r = self.result
        rows = [("Méthode", method.split(" · ")[1]),
                ("Pose trouvée", f"{r.pose.x:+.3f}  {r.pose.y:+.3f}  {r.pose.theta_deg:+.1f}°"),
                ("Score", f"{r.score:.2f}"), ("Temps", f"{r.time_s * 1000:.0f} ms"),
                ("Évaluations", str(r.evaluations))]
        if truth is not None:
            e, a = pose_error(r.pose, truth)
            ok = e < 0.05 and a < 2.0
            rows += [("Erreur", f"{e * 100:.1f} cm · {a:.2f}°"), ("Verdict", "réussi ✓" if ok else "échec ✗")]
        self.out.show(rows, highlight="Erreur" if truth is not None else "Pose trouvée")
        msg = f"{method.split(' · ')[1]} : pose ({r.pose.x:+.2f}, {r.pose.y:+.2f}, {r.pose.theta_deg:+.1f}°) en {r.time_s * 1000:.0f} ms"
        if truth is not None:
            msg += f" · erreur {e * 100:.1f} cm, {a:.2f}° · {'réussi ✓' if ok else 'échec ✗'}"
        self.app.status(msg)
        self.app.render()

    def compare(self) -> None:
        pts, truth, refs = self._ready()
        if pts is None:
            return
        self.app.status("Comparaison des 10 méthodes…")
        self.app.root.update_idletasks()
        rows = []
        for m in METHODS:
            try:
                r = localize(m, pts, self.start, refs)
            except ValueError:
                continue
            err = pose_error(r.pose, truth) if truth is not None else None
            rows.append((m, r, err))
        for w in self.table.winfo_children():
            w.destroy()
        self.table.pack(fill="x", pady=(0, 10))
        ctk.CTkLabel(self.table, text="Comparaison (même scan, même départ)", font=S.font(14, "bold"),
                     text_color=S.INK, anchor="w").grid(row=0, column=0, columnspan=3, sticky="w", padx=14, pady=(12, 6))
        for k, (m, r, err) in enumerate(rows, start=1):
            ok = err is not None and err[0] < 0.05 and err[1] < 2.0
            col = S.SUCCESS if ok else (S.DANGER if err is not None else S.INK)
            ctk.CTkLabel(self.table, text=m, font=S.font(11), text_color=S.INK, anchor="w").grid(
                row=k, column=0, sticky="w", padx=(14, 6))
            txt = f"{err[0] * 100:.1f} cm {err[1]:.1f}°" if err is not None else f"{r.pose.x:+.2f} {r.pose.y:+.2f}"
            ctk.CTkLabel(self.table, text=txt, font=S.font(11, mono=True), text_color=col).grid(row=k, column=1, sticky="e")
            ctk.CTkLabel(self.table, text=f"{r.time_s * 1000:.0f} ms", font=S.font(11, mono=True),
                         text_color=S.MUTED).grid(row=k, column=2, sticky="e", padx=(8, 14))
        ctk.CTkLabel(self.table, text="").grid(row=len(rows) + 1, column=0, pady=2)
        self.app.status("Comparaison terminée : vert = réussi (< 5 cm et < 2°).")

    def apply_found(self) -> None:
        if self.result is None:
            self.app.status("Lance d'abord une localisation.")
            return
        cap = self._target_cap()
        if cap is not None:
            self.app.apply(P.set_poses(self.project, {cap.number: self.result.pose}), f"Pose de {cap.title} mise à jour.")
        elif self.app.last_frame is not None:
            new, nums = P.add_scans(self.project, [("localisé", self.app.last_frame.scan.with_pose(self.result.pose))],
                                    source=self.app.last_frame.fmt)
            self.app.apply(new, f"Scan en direct ajouté au projet à la pose trouvée (scan {nums[0]}).")

    def reset(self) -> None:
        self.start, self.result, self._refs_key = None, None, None

    def on_live(self, frame) -> None:
        if self.target.get() == LIVE:
            self.app.render(100)

    # ------------------------------------------------------------------ carte
    def render(self) -> None:
        app = self.app
        if self.project.is_empty:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        refs = self.references() if self.start is not None else None
        kind = REFS[self.ref.get()]
        img = extent = None
        segs, layers = [], []
        if refs is not None and kind == "grid" and refs.field is not None:
            f = refs.field
            img = 1.0 - 0.9 * f.likelihood                     # champ de vraisemblance : sombre près des murs
            extent = (f.origin[0], f.origin[0] + f.dist.shape[1] * f.resolution,
                      f.origin[1], f.origin[1] + f.dist.shape[0] * f.resolution)
        elif refs is not None and kind == "vecteur" and refs.segmap is not None:
            segs = [Segment(float(a[0]), float(a[1]), float(b[0]), float(b[1]))
                    for a, b in zip(refs.segmap.a, refs.segmap.b)]
        elif refs is not None and refs.scan_ref is not None:
            layers.append((refs.scan_ref.points, S.FAINT, 5, "o"))
        app.map.draw_analysis(img, extent, segs, [], None)
        pts, truth = self._scan()
        poses = []
        if pts is not None and self.start is not None:
            layers.append((local_to_global(pts, self.start), S.INITIAL, 7, "o"))
            poses.append((self.start, S.INITIAL, False))
            if self.result is not None:
                layers.append((local_to_global(pts, self.result.pose), S.FOUND, 9, "o"))
                poses.append((self.result.pose, S.FOUND, True))
            if truth is not None:
                layers.append((np.array([[truth.x, truth.y]]), S.INK, 260, "+"))
        app.map.draw_overlay(layers, [], poses)
        self.legend("orange = départ · vert = trouvé · + = vraie pose · fond sombre = proche d'un mur (vraisemblance)")
