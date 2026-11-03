"""Étape 5 · Localiser : où est le robot sur la carte de référence ?

Deux usages :
- « Un scan du projet » : on choisit un scan, on lance. Sans pose de départ (par défaut), la
  localisation GLOBALE fouille toute la carte ; avec une pose approximative (clic sur la carte),
  la méthode choisie cherche autour. Sa pose actuelle sert de vérité pour afficher l'erreur.
- « Le robot en direct » : chaque scan reçu (robot réel ou démo qui roule dans ta carte) est localisé
  en temps réel (localization.tracking) : global au début, puis suivi autour de la pose précédente.
La carte utilisée est la carte de référence VALIDÉE à l'étape Cartes (sinon une carte provisoire).
"""

from __future__ import annotations

import customtkinter as ctk
import numpy as np

from ...geometry import Pose, local_to_global
from ...localization import METHODS, References, global_localize, localize, pose_error
from ...localization import grid as G
from ...localization import scans as SC
from ...localization import vector as V
from ...localization.globalloc import _on_map
from ...localization.tracking import Tracker
from ...refmap import build_reference
from ...segments import Segment
from .. import style as S
from ..tools import PoseTool
from .base import Step

MODES = ["Un scan du projet", "Le robot en direct"]
STARTS = ["Inconnue : chercher partout", "Approximative : clic"]
REFS = {"Grid map": "grid", "Carte vectorielle": "vecteur", "Scan voisin": "scans"}
PERTURB = {"10 cm · 5°": (0.10, 5), "25 cm · 10°": (0.25, 10), "50 cm · 20°": (0.50, 20), "1 m · 30°": (1.0, 30)}
EXPLAIN = {
    "exhaustive": "Teste toutes les poses autour du départ (pas de 5 cm et 1°) : sûr mais lent.",
    "multi-résolution": "Recherche complète sur une carte grossière, puis affinage sur des cartes plus fines.",
    "gradient": "Descend la pente de l'erreur depuis le départ (Gauss-Newton) : précis mais il faut partir près.",
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
    headline = "Vert = scan à la pose trouvée · + = vraie pose · orange = départ (si donné) · fond = carte de référence."
    course = ("Scan matching : on cherche la transformation (x, y, θ) qui pose au mieux les points du scan sur la "
              "carte. Sans pose de départ, on fouille toute la carte (localisation globale) ; avec une pose "
              "approximative, on cherche autour (exhaustive, multi-résolution, gradient). En direct, la pose "
              "précédente sert de point de départ au scan suivant.")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.start: Pose | None = None
        self.found = None           # (pose, candidats globaux)
        self.tracker: Tracker | None = None
        self.live_step = None
        self.live_points = None
        self._refs: dict = {}
        self._temp_ref = (None, None)

    # ================================================================== panneau
    def build(self, panel) -> None:
        card = S.Card(panel, "Carte de référence")
        card.pack(fill="x", pady=(0, 10))
        self.ref_info = S.hint(card.body, "")
        self.validate_btn = S.button(card.body, "Valider la carte actuelle (étape Cartes)", self._validate, "ghost")
        self.validate_btn.pack(fill="x")

        self.mode = S.segmented(panel, MODES, MODES[0], lambda v: self._mode_changed())
        self.mode.pack(fill="x", pady=(0, 10))

        # ---- un scan du projet
        self.project_box = ctk.CTkFrame(panel, fg_color="transparent")
        card = S.Card(self.project_box, "Scan à localiser")
        card.pack(fill="x", pady=(0, 10))
        self.target = ctk.CTkOptionMenu(card.body, values=["—"], height=30, font=S.font(12), fg_color=S.SURFACE_2,
                                        button_color=S.BORDER, button_hover_color=S.FAINT, text_color=S.INK,
                                        dropdown_font=S.font(12), command=lambda v: self._target_changed())
        self.target.pack(fill="x")
        card = S.Card(self.project_box, "Pose de départ")
        card.pack(fill="x", pady=(0, 10))
        self.start_mode = S.segmented(card.body, STARTS, STARTS[0], lambda v: self._start_mode_changed())
        self.start_mode.pack(fill="x")
        self.start_hint = S.hint(card.body, "")
        self.start_field = S.PoseField(card.body)
        S.button(self.project_box, "Localiser", self.run, "primary", height=44, font=S.font(15, "bold")).pack(
            fill="x", pady=(0, 10))

        # ---- robot en direct
        self.live_box = ctk.CTkFrame(panel, fg_color="transparent")
        card = S.Card(self.live_box, "Robot", "Le robot réel se connecte à l'étape 1 (câble USB). Sans robot, la démo "
                                              "fait rouler un robot simulé dans ta carte de référence.")
        card.pack(fill="x", pady=(0, 10))
        self.live_info = S.hint(card.body, "")
        row = ctk.CTkFrame(card.body, fg_color="transparent")
        row.pack(fill="x", pady=(4, 0))
        S.button(row, "Démo sur ma carte", self._demo, width=150).pack(side="left")
        S.button(row, "Connecter (étape 1)", lambda: self.app.go(0), "ghost", width=150).pack(side="left", padx=6)
        self.follow_btn = S.button(self.live_box, "▶  Suivre le robot en temps réel", self.toggle_follow, "primary",
                                   height=44, font=S.font(15, "bold"))
        self.follow_btn.pack(fill="x", pady=(0, 6))
        S.button(self.live_box, "Relocaliser dans toute la carte", self._relocalize, "ghost").pack(fill="x", pady=(0, 10))

        # ---- résultat (commun)
        card = S.Card(panel, "Résultat")
        card.pack(fill="x", pady=(0, 10))
        self.out = S.KeyValues(card.body)
        self.out.pack(fill="x")
        self.out.show([("—", "lance une localisation")])

        # ---- avancé
        adv = S.Collapsible(panel, "Avancé : référence, méthode, tests")
        adv.pack(fill="x", pady=(0, 10))
        b = adv.body
        ctk.CTkLabel(b, text="Référence", font=S.font(12, "bold"), text_color=S.INK, anchor="w").pack(fill="x")
        self.ref = S.segmented(b, list(REFS), "Grid map", lambda v: self._ref_changed())
        self.ref.pack(fill="x", pady=(2, 8))
        ctk.CTkLabel(b, text="Méthode (après la recherche globale, ou depuis la pose approximative)",
                     font=S.font(12, "bold"), text_color=S.INK, anchor="w", wraplength=330, justify="left").pack(fill="x")
        self.method_var = ctk.StringVar()
        self.method_box = ctk.CTkFrame(b, fg_color="transparent")
        self.method_box.pack(fill="x")
        self.method_hint = S.hint(b, "", S.INK)
        S.button(b, "Comparer toutes les méthodes sur ce scan", self.compare).pack(fill="x", pady=(4, 8))
        self.control = S.toggle(b, "Contrôle : carte construite sans ce scan", self._invalidate)
        self.control.pack(anchor="w", pady=(0, 4))
        row = ctk.CTkFrame(b, fg_color="transparent")
        row.pack(fill="x")
        self.perturb = ctk.CTkOptionMenu(row, values=list(PERTURB), width=120, height=30, font=S.font(12),
                                         fg_color=S.SURFACE_2, button_color=S.BORDER, text_color=S.INK)
        self.perturb.set("50 cm · 20°")
        self.perturb.pack(side="left")
        S.button(row, "Départ = vraie pose faussée", self.perturb_truth, "ghost").pack(side="left", padx=6)
        self.table = ctk.CTkFrame(b, fg_color="transparent")
        self.table.pack(fill="x", pady=(8, 0))
        self._ref_changed()
        self._mode_changed()

    # ================================================================== état
    def _mode_changed(self) -> None:
        live = self.mode.get() == MODES[1]
        (self.project_box.pack_forget if live else self.project_box.pack)(**({} if live else
                                                                            {"fill": "x", "after": self.mode}))
        (self.live_box.pack if live else self.live_box.pack_forget)(**({"fill": "x", "after": self.mode} if live else {}))
        if not live and self.tracker is not None:
            self.toggle_follow()
        self.enter_tool()
        self.app.render()

    def _start_mode_changed(self) -> None:
        approx = self.start_mode.get() == STARTS[1]
        self.start_hint.configure(text="Clique sur la carte à l'endroit supposé, glisse pour l'orientation."
                                  if approx else "La localisation globale fouille toute la carte : rien à donner.")
        if approx:
            self.start_field.pack(anchor="w", pady=(6, 0))
        else:
            self.start_field.pack_forget()
            self.start = None
        self.enter_tool()
        self.app.render()

    def enter_tool(self) -> None:
        if self.app.step is not self:
            return
        approx = self.mode.get() == MODES[0] and self.start_mode.get() == STARTS[1]
        self.app.set_tool(PoseTool(self.app.map, self.set_start, S.INITIAL) if approx else None)

    def _ref_changed(self) -> None:
        kind = REFS[self.ref.get()]
        for w in self.method_box.winfo_children():
            w.destroy()
        names = [m for m in METHODS if m.startswith(kind)]
        self.method_var.set(next((m for m in names if "+" in m), names[0]))
        for m in names:
            ctk.CTkRadioButton(self.method_box, text=m.split(" · ")[1], value=m, variable=self.method_var,
                               font=S.font(12), text_color=S.INK, fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER,
                               border_color=S.FAINT,
                               command=lambda: self.method_hint.configure(text=EXPLAIN[self.method_var.get().split(" · ")[1]])
                               ).pack(anchor="w", pady=2)
        self.method_hint.configure(text=EXPLAIN[self.method_var.get().split(" · ")[1]])
        self.app.render()

    def _targets(self) -> dict[str, int]:
        return {c.title: c.number for c in self.project.visible}

    def on_project(self) -> None:
        names = list(self._targets()) or ["—"]
        self.target.configure(values=names)
        if self.target.get() not in names:
            self.target.set(names[0])
            self.found = None
        ref = self.project.reference_map
        self.ref_info.configure(text=(f"Carte validée : {ref.describe()}" if ref else
                                      "Aucune carte validée : une carte provisoire (scans visibles + retouches) est "
                                      "utilisée. Valide-la à l'étape Cartes pour figer la référence."),
                                text_color=S.SUCCESS if ref else S.WARN)
        (self.validate_btn.pack_forget if ref else self.validate_btn.pack)(**({} if ref else {"fill": "x"}))

    def _target_changed(self) -> None:
        self.found = None
        self.app.render()

    def _invalidate(self) -> None:
        self.found = None
        self.app.render()

    def _validate(self) -> None:
        self.app.steps[3].validate()

    def enter(self) -> None:
        self.on_project()
        self.enter_tool()

    def leave(self) -> None:
        if self.tracker is not None:
            self.toggle_follow()

    def reset(self) -> None:
        self.start, self.found, self.tracker, self.live_step = None, None, None, None
        self._refs.clear()

    # ================================================================== références
    def reference_map(self, exclude: int | None = None):
        """Carte de référence validée, ou carte provisoire (scans visibles + retouches) mise en cache."""
        if exclude is None and self.project.reference_map is not None:
            return self.project.reference_map
        caps = [c for c in self.project.visible if c.number != exclude]
        if not caps:
            return None
        key = (tuple((c.number, id(c.scan.ranges_m), c.scan.pose) for c in caps), self.project.wall_edits,
               self.app.filters())
        if self._temp_ref[0] != key:
            self._temp_ref = (key, build_reference([self.app.filtered(c) for c in caps], [c.title for c in caps],
                                                   self.project.wall_edits))
        return self._temp_ref[1]

    def references(self, ref, near_points: np.ndarray | None) -> References:
        """Les trois références (grid, murs, scan voisin) tirées de la carte, préparées une fois par carte."""
        k = id(ref)
        if k not in self._refs:
            if len(self._refs) > 6:
                self._refs.clear()
            field = G.LikelihoodField.from_grid(ref.grid)
            segmap = V.SegmentMap.from_segments(list(ref.segments)) if ref.segments else None
            seg_field = G.LikelihoodField.from_points(segmap.sample()) if segmap else None
            self._refs[k] = References(field=field, pyramid=G.build_pyramid(field), segmap=segmap, seg_field=seg_field,
                                       seg_pyramid=G.build_pyramid(seg_field) if seg_field else None)
        base = self._refs[k]
        if near_points is None or not len(near_points):
            return base
        scan_ref = SC.ScanReference.from_points(near_points)
        return References(base.field, base.pyramid, base.segmap, base.seg_field, base.seg_pyramid, scan_ref,
                          G.LikelihoodField.from_points(scan_ref.points))

    def _near_points(self, pose: Pose, exclude: int | None) -> np.ndarray | None:
        caps = [c for c in self.project.visible if c.number != exclude]
        if not caps:
            return None
        near = min(caps, key=lambda c: np.hypot(c.scan.pose.x - pose.x, c.scan.pose.y - pose.y))
        return self.app.points(near)

    # ================================================================== scan du projet
    def _target_cap(self):
        return self.project.find(self._targets().get(self.target.get()))

    def set_start(self, pose: Pose) -> None:
        self.start = pose
        self.start_field.set(pose.x, pose.y, pose.theta_deg)
        self.found = None
        self.app.render()

    def perturb_truth(self) -> None:
        cap = self._target_cap()
        if cap is None:
            return
        d, a = PERTURB[self.perturb.get()]
        rng = np.random.default_rng()
        phi = rng.uniform(0, 2 * np.pi)
        self.start_mode.set(STARTS[1])
        self._start_mode_changed()
        p = cap.scan.pose
        self.set_start(p.moved(d * np.cos(phi), d * np.sin(phi), a * rng.choice([-1, 1])))

    def _prepare(self):
        cap = self._target_cap()
        if cap is None:
            self.app.status("Choisis un scan du projet (ou passe en mode « robot en direct »).")
            return None
        exclude = cap.number if self.control.get() else None
        ref = self.reference_map(exclude)
        if ref is None:
            self.app.status("Il faut une carte : au moins un autre scan visible.")
            return None
        pts = self.app.filtered(cap).local_points()
        if len(pts) < 20:
            self.app.status("Ce scan a trop peu de points pour être localisé.")
            return None
        if self.start_mode.get() == STARTS[1]:
            try:
                self.start = Pose(*self.start_field.get())
            except ValueError:
                self.app.status("La pose de départ doit contenir des nombres.")
                return None
        return cap, exclude, ref, pts

    def run(self) -> None:
        prep = self._prepare()
        if prep is None:
            return
        cap, exclude, ref, pts = prep
        self.app.status("Localisation en cours…")
        self.app.root.update_idletasks()
        rows, glob = [], None
        start, t_global = self.start, 0.0
        if start is None:
            glob = global_localize(pts, ref.grid, ref.segments, self.references(ref, None).field,
                                   self.references(ref, None).pyramid)
            start, t_global = glob.pose, glob.time_s
        refs = self.references(ref, self._near_points(start, exclude))
        method = self.method_var.get()
        try:
            r = localize(method, pts, start, refs)
        except ValueError as exc:
            self.app.status(f"Méthode impossible avec cette carte : {exc}")
            return
        inl = float(_on_map(refs.field, pts, r.pose).mean())
        self.found = (r.pose, glob.candidates[1:3] if glob else [])
        rows.append(("Pose trouvée", f"{r.pose.x:+.2f}  {r.pose.y:+.2f}  {r.pose.theta_deg:+.1f}°"))
        if glob:
            rows.append(("Confiance", "ambiguë ⚠ (salle symétrique vue d'ici)" if glob.ambiguous else "fiable ✓"))
        rows += [("Points sur les murs", f"{inl:.0%}"), ("Temps", f"{(t_global + r.time_s) * 1000:.0f} ms"),
                 ("Méthode", ("globale + " if glob else "") + method.split(" · ")[1])]
        truth = cap.scan.pose
        e, a = pose_error(r.pose, truth)
        ok = e < 0.05 and a < 2.0
        rows += [("Écart à la pose du scan", f"{e * 100:.1f} cm · {a:.2f}°"), ("Verdict", "retrouvé ✓" if ok else "différent ✗")]
        self.out.show(rows, highlight="Pose trouvée")
        self.app.status(f"{cap.title} : pose ({r.pose.x:+.2f}, {r.pose.y:+.2f}, {r.pose.theta_deg:+.1f}°), "
                        f"{inl:.0%} des points sur les murs, écart {e * 100:.1f} cm" +
                        (" · ⚠ ambiguë" if glob and glob.ambiguous else ""))
        self.app.render()

    def compare(self) -> None:
        prep = self._prepare()
        if prep is None:
            return
        cap, exclude, ref, pts = prep
        start = self.start
        if start is None:
            base = self.references(ref, None)
            start = global_localize(pts, ref.grid, ref.segments, base.field, base.pyramid).pose
        refs = self.references(ref, self._near_points(start, exclude))
        for w in self.table.winfo_children():
            w.destroy()
        ctk.CTkLabel(self.table, text="Même scan, même départ — vert : < 5 cm et < 2°", font=S.font(11),
                     text_color=S.MUTED, anchor="w").grid(row=0, column=0, columnspan=3, sticky="w")
        for k, m in enumerate(METHODS, start=1):
            try:
                r = localize(m, pts, start, refs)
            except ValueError:
                continue
            e, a = pose_error(r.pose, cap.scan.pose)
            col = S.SUCCESS if (e < 0.05 and a < 2.0) else S.DANGER
            ctk.CTkLabel(self.table, text=m, font=S.font(11), text_color=S.INK, anchor="w").grid(row=k, column=0, sticky="w")
            ctk.CTkLabel(self.table, text=f"{e * 100:.1f} cm {a:.1f}°", font=S.font(11, mono=True),
                         text_color=col).grid(row=k, column=1, sticky="e", padx=6)
            ctk.CTkLabel(self.table, text=f"{r.time_s * 1000:.0f} ms", font=S.font(11, mono=True),
                         text_color=S.MUTED).grid(row=k, column=2, sticky="e")
        self.app.status("Comparaison terminée (départ : " + ("pose approximative" if self.start else "localisation globale") + ").")

    # ================================================================== robot en direct
    def _demo(self) -> None:
        ref = self.reference_map()
        if ref is None or not ref.segments:
            self.app.status("Il faut une carte avec des murs (étape Cartes) pour la démo.")
            return
        walls = [s.as_tuple() for s in ref.segments]
        way = [(p.x, p.y) for _, p in ref.scans] or [(0.0, 0.0)]
        order = [way.pop(0)]
        while way:                                    # on visite les poses des scans de proche en proche
            k = min(range(len(way)), key=lambda i: np.hypot(way[i][0] - order[-1][0], way[i][1] - order[-1][1]))
            order.append(way.pop(k))
        self.app.map_demo(walls, order)
        self.app.status("Démo : un robot simulé roule dans ta carte. Clique sur « Suivre le robot ».")

    def toggle_follow(self) -> None:
        if self.tracker is not None:
            self.tracker = None
            self.follow_btn.configure(text="▶  Suivre le robot en temps réel", fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER)
            self.app.status("Suivi arrêté.")
            return
        if self.app.source is None:
            self.app.status("Aucun robot : lance la démo sur ta carte, ou connecte le robot à l'étape 1.")
            return
        ref = self.reference_map()
        if ref is None:
            self.app.status("Il faut une carte de référence (étape Cartes).")
            return
        self.tracker = Tracker(ref.grid, ref.segments)
        self.live_step = None
        self.follow_btn.configure(text="■  Arrêter le suivi", fg_color=S.WARN, hover_color="#E68600")
        self.app.status("Suivi : 1er scan localisé dans toute la carte, puis autour de la pose précédente.")

    def _relocalize(self) -> None:
        if self.tracker is not None:
            self.tracker.reset()
            self.app.status("Le prochain scan sera localisé dans toute la carte.")

    def on_live(self, frame) -> None:
        if self.mode.get() != MODES[1]:
            return
        src = "démo sur ta carte" if self.app.demo_truth is not None else "robot"
        self.live_info.configure(text=f"Connecté ({src}) · {int((frame.scan.ranges_m > 0).sum())} points par tour",
                                 text_color=S.SUCCESS)
        if self.tracker is None:
            return
        pts = self.app.filters().apply(frame.scan).local_points()
        try:
            st = self.tracker.update(pts)
        except ValueError as exc:
            self.app.status(str(exc))
            return
        self.live_step, self.live_points = st, pts
        rows = [("Pose du robot", f"{st.pose.x:+.2f}  {st.pose.y:+.2f}  {st.pose.theta_deg:+.1f}°"),
                ("Mode", "global" if st.mode == "global" else "suivi"),
                ("Points sur les murs", f"{st.inliers:.0%}" + ("  ⚠ ambiguë" if st.ambiguous else "")),
                ("Temps de calcul", f"{st.time_s * 1000:.0f} ms")]
        truth = self.app.demo_truth
        if truth is not None:
            e, a = pose_error(st.pose, truth)
            rows.append(("Erreur (démo)", f"{e * 100:.1f} cm · {a:.1f}°"))
        self.out.show(rows, highlight="Pose du robot")
        self.app.render()

    # ================================================================== carte
    def render(self) -> None:
        app = self.app
        live = self.mode.get() == MODES[1]
        cap = None if live else self._target_cap()
        exclude = cap.number if (cap is not None and self.control.get()) else None
        ref = self.reference_map(exclude) if not self.project.is_empty or self.project.reference_map else None
        if ref is None:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        img = 1.0 - ref.grid.probability()
        app.map.draw_analysis(img, ref.grid.extent(), list(ref.segments), [], None)
        layers, poses, trail = [], [], []
        if live:
            st = self.live_step
            if st is not None and self.live_points is not None:
                layers.append((local_to_global(self.live_points, st.pose), S.FOUND, 7, "o"))
                poses.append((st.pose, S.ACCENT, True))
            if self.tracker is not None and len(self.tracker.trail) > 1:
                tr = self.tracker.trail
                trail = [Segment(a.x, a.y, b.x, b.y) for a, b in zip(tr, tr[1:])]
            truth = app.demo_truth
            if truth is not None:
                layers.append((np.array([[truth.x, truth.y]]), S.INK, 260, "+"))
            self.legend("bleu = robot localisé · vert = son scan · trait = trajet · + = vraie pose (démo)")
        elif cap is not None:
            pts = app.filtered(cap).local_points()
            if self.start is not None:
                layers.append((local_to_global(pts, self.start), S.INITIAL, 6, "o"))
                poses.append((self.start, S.INITIAL, False))
            if self.found is not None:
                pose, others = self.found
                layers.append((local_to_global(pts, pose), S.FOUND, 9, "o"))
                poses.append((pose, S.FOUND, True))
                for p, _ in others:
                    poses.append((p, S.FAINT, False))
            layers.append((np.array([[cap.scan.pose.x, cap.scan.pose.y]]), S.INK, 260, "+"))
            if REFS[self.ref.get()] == "scans" and self.found is not None:
                near = self._near_points(self.found[0], exclude)
                if near is not None:
                    layers.insert(0, (near, S.FAINT, 4, "o"))
            self.legend("vert = trouvé · gris = autres candidats · orange = départ · + = pose connue")
        app.map.draw_overlay(layers, [(trail, S.ACCENT, 2.0)] if trail else [], poses)
