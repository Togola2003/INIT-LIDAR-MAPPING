"""Étape 5 · Localiser : replacer un scan sur la carte de référence, ou suivre le robot en direct.

Ce que demande le TP, dans l'ordre :
- « Scan de la liste » : on clique sur un scan dans la liste à gauche, il est aussitôt localisé sur la
  carte de référence, SANS pose de départ (localisation globale), puis affiné par la méthode choisie.
  Sa pose dans le projet est montrée en gris : on voit si l'algorithme le remet au même endroit.
- « Fichier importé » : un scan qui n'est pas dans le projet (autre séance, autre groupe…).
- « Robot en direct » : robot en Wi-Fi (piloté depuis cette page), au câble, ou démo dans la carte ;
  chaque nouveau tour est localisé (global au début, puis autour de la pose précédente).
"""

from __future__ import annotations

from pathlib import Path
from tkinter import filedialog

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
from ...scope.session import read_any
from ...segments import Segment
from .. import style as S
from ..components import PilotPad
from ..tools import PoseTool
from .base import Step

MODES = ["Scan de la liste", "Fichier importé", "Robot en direct"]
REFS = {"Grid map": "grid", "Carte vectorielle": "vecteur", "Scan voisin": "scans"}
LOST = 0.35            # moins de 35 % des points sur les murs : pose jugée fausse
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
    headline = "Gris = scan à sa pose connue · vert = scan replacé par l'algorithme · fond = carte de référence."
    course = ("Scan matching : on cherche la transformation (x, y, θ) qui pose au mieux les points du scan sur la "
              "carte de référence. Sans pose de départ, on fouille toute la carte (localisation globale), puis "
              "on affine (multi-résolution, gradient…). En direct, la pose précédente sert de point de départ "
              "au scan suivant : c'est rapide (≈ 15 ms).")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.start: Pose | None = None
        self.found = None               # dict : pose, candidates, points, truth
        self.imported: list = []        # [(label, Scan)] du fichier importé
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
        self.validate_btn = S.button(card.body, "Valider la carte actuelle", lambda: self.app.steps[3].validate(), "ghost")
        self.validate_btn.pack(fill="x")

        ctk.CTkLabel(panel, text="Que localiser ?", font=S.font(13, "bold"), text_color=S.INK, anchor="w").pack(fill="x")
        self.mode = S.segmented(panel, MODES, MODES[0], lambda v: self._mode_changed())
        self.mode.pack(fill="x", pady=(4, 10))

        # ---- scan de la liste
        self.list_box = S.Card(panel, "Scan de la liste", "Clique sur un scan dans la liste à gauche : il est "
                                                          "localisé tout de suite sur la carte de référence.")
        self.list_label = ctk.CTkLabel(self.list_box.body, text="—", font=S.font(13, "bold"), text_color=S.MUTED,
                                       anchor="w")
        self.list_label.pack(fill="x")
        S.button(self.list_box.body, "Localiser ce scan", self.run, "primary", height=42,
                 font=S.font(14, "bold")).pack(fill="x", pady=(8, 0))

        # ---- fichier importé
        self.file_box = S.Card(panel, "Fichier importé", "Un scan qui n'est pas dans le projet (CSV du TP, "
                                                         "RobotControl, MapTool, session…). Il n'est pas ajouté à la carte.")
        S.button(self.file_box.body, "Importer un scan…", self.import_scan).pack(fill="x")
        self.file_pick = ctk.CTkOptionMenu(self.file_box.body, values=["—"], height=30, font=S.font(12),
                                           fg_color=S.SURFACE_2, button_color=S.BORDER, text_color=S.INK,
                                           command=lambda v: self.run())
        self.file_pick.pack(fill="x", pady=(8, 0))
        S.button(self.file_box.body, "Localiser ce scan", self.run, "primary", height=42,
                 font=S.font(14, "bold")).pack(fill="x", pady=(8, 0))

        # ---- robot en direct
        self.live_box = ctk.CTkFrame(panel, fg_color="transparent")
        card = S.Card(self.live_box, "Robot", "Wi-Fi ou câble : connexion à l'étape 1. Sans robot : la démo fait "
                                              "rouler un robot simulé dans ta carte.")
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
        self.pilot_card = S.Card(self.live_box, "Piloter (Wi-Fi)", "Déplace le robot et regarde-le se replacer.")
        self.pilot_card.pack(fill="x", pady=(0, 10))
        self.pad = PilotPad(self.pilot_card.body, lambda: self.app.robot, self.app.status)
        self.pad.pack(fill="x")

        # ---- résultat
        self.result_card = S.Card(panel, "Résultat")
        self.result_card.pack(fill="x", pady=(0, 10))
        self.out = S.KeyValues(self.result_card.body)
        self.out.pack(fill="x")
        self.out.show([("—", "en attente")])

        # ---- avancé
        adv = S.Collapsible(panel, "Avancé : référence, méthode, tests")
        adv.pack(fill="x", pady=(0, 10))
        b = adv.body
        ctk.CTkLabel(b, text="Référence", font=S.font(12, "bold"), text_color=S.INK, anchor="w").pack(fill="x")
        self.ref = S.segmented(b, list(REFS), "Grid map", lambda v: self._ref_changed())
        self.ref.pack(fill="x", pady=(2, 8))
        ctk.CTkLabel(b, text="Méthode d'affinage (après la recherche globale)", font=S.font(12, "bold"),
                     text_color=S.INK, anchor="w").pack(fill="x")
        self.method_var = ctk.StringVar()
        self.method_box = ctk.CTkFrame(b, fg_color="transparent")
        self.method_box.pack(fill="x")
        self.method_hint = S.hint(b, "", S.INK)
        S.button(b, "Comparer toutes les méthodes sur ce scan", self.compare).pack(fill="x", pady=(4, 8))
        self.control = S.toggle(b, "Contrôle : carte construite sans ce scan", self._invalidate)
        self.control.pack(anchor="w", pady=(0, 6))
        ctk.CTkLabel(b, text="Pose de départ (facultative)", font=S.font(12, "bold"), text_color=S.INK,
                     anchor="w").pack(fill="x")
        S.hint(b, "Sans pose de départ, la recherche est globale. Avec une pose (à moins d'1 m et 30° de la vraie), "
                  "seule la méthode choisie cherche autour ; si elle échoue, on repasse en global.")
        row = ctk.CTkFrame(b, fg_color="transparent")
        row.pack(fill="x")
        S.button(row, "Cliquer sur la carte", self.pick_start, width=140).pack(side="left")
        S.button(row, "Aucune", self.clear_start, "ghost", width=80).pack(side="left", padx=6)
        row = ctk.CTkFrame(b, fg_color="transparent")
        row.pack(fill="x", pady=(6, 0))
        self.perturb = ctk.CTkOptionMenu(row, values=list(PERTURB), width=120, height=30, font=S.font(12),
                                         fg_color=S.SURFACE_2, button_color=S.BORDER, text_color=S.INK)
        self.perturb.set("50 cm · 20°")
        self.perturb.pack(side="left")
        S.button(row, "= pose connue faussée", self.perturb_truth, "ghost").pack(side="left", padx=6)
        self.table = ctk.CTkFrame(b, fg_color="transparent")
        self.table.pack(fill="x", pady=(8, 0))
        self._ref_changed()
        self._mode_changed()

    # ================================================================== modes et réglages
    def _mode_changed(self) -> None:
        mode = self.mode.get()
        boxes = {MODES[0]: self.list_box, MODES[1]: self.file_box, MODES[2]: self.live_box}
        for m, box in boxes.items():
            if m == mode:
                box.pack(fill="x", pady=(0, 10), before=self.result_card)
            else:
                box.pack_forget()
        if mode != MODES[2] and self.tracker is not None:
            self.toggle_follow()
        self.found = None
        self.out.show([("—", "en attente")])
        self.app.render()

    def _ref_changed(self) -> None:
        kind = REFS[self.ref.get()]
        for w in self.method_box.winfo_children():
            w.destroy()
        names = [m for m in METHODS if m.startswith(kind)]
        self.method_var.set(next((m for m in names if "+" in m), names[0]))
        for m in names:
            ctk.CTkRadioButton(self.method_box, text=m.split(" · ")[1], value=m, variable=self.method_var,
                               font=S.font(12), text_color=S.INK, fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER,
                               border_color=S.FAINT, command=self._method_hint).pack(anchor="w", pady=2)
        self._method_hint()

    def _method_hint(self) -> None:
        self.method_hint.configure(text=EXPLAIN[self.method_var.get().split(" · ")[1]])

    def _invalidate(self) -> None:
        self.found = None
        self.app.render()

    def on_project(self) -> None:
        ref = self.project.reference_map
        self.ref_info.configure(text=(f"Carte validée : {ref.describe()}" if ref else
                                      "Aucune carte validée : une carte provisoire (scans visibles + retouches) est "
                                      "utilisée. Valide-la à l'étape Cartes pour figer la référence."),
                                text_color=S.SUCCESS if ref else S.WARN)
        if ref:
            self.validate_btn.pack_forget()
        else:
            self.validate_btn.pack(fill="x")
        self._show_list_target()

    def _show_list_target(self) -> None:
        cap = self.project.find(self.app.primary)
        self.list_label.configure(text=f"● {cap.title}" if cap else "Aucun scan sélectionné",
                                  text_color=cap.color if cap else S.MUTED)

    def on_selection(self) -> None:
        """Clic sur un scan de la liste : on le localise aussitôt."""
        self._show_list_target()
        if self.mode.get() == MODES[0] and self.app.step is self and self.project.find(self.app.primary):
            self.app.root.after(10, self.run)

    def enter(self) -> None:
        self.on_project()

    def leave(self) -> None:
        if self.tracker is not None:
            self.toggle_follow()

    def reset(self) -> None:
        self.start, self.found, self.tracker, self.live_step = None, None, None, None
        self.imported = []
        self._refs.clear()

    def on_key(self, key: str) -> None:
        if self.mode.get() == MODES[2]:
            self.pad.key(key)

    # ================================================================== pose de départ (avancé)
    def pick_start(self) -> None:
        self.app.set_tool(PoseTool(self.app.map, self._start_picked, S.INITIAL))
        self.app.status("Clique sur la carte là où tu penses que le scan a été pris, glisse pour l'orientation.")

    def _start_picked(self, pose: Pose) -> None:
        self.app.set_tool(None)
        self.start = pose
        self.app.status(f"Pose de départ : ({pose.x:+.2f}, {pose.y:+.2f}, {pose.theta_deg:+.0f}°). Lance la localisation.")
        self.app.render()

    def clear_start(self) -> None:
        self.start = None
        self.app.status("Plus de pose de départ : recherche globale dans toute la carte.")
        self.app.render()

    def perturb_truth(self) -> None:
        target = self._target()
        if target is None or target[2] is None:
            self.app.status("Il faut un scan dont la pose est connue.")
            return
        d, a = PERTURB[self.perturb.get()]
        rng = np.random.default_rng()
        phi = rng.uniform(0, 2 * np.pi)
        t = target[2]
        self.start = t.moved(d * np.cos(phi), d * np.sin(phi), a * rng.choice([-1, 1]))
        self.run()

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

    def references(self, ref, near_points: np.ndarray | None = None) -> References:
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

    # ================================================================== localiser un scan
    def _target(self):
        """(titre, points du scan dans le repère lidar, pose connue ou None, numéro à exclure) selon le mode."""
        if self.mode.get() == MODES[0]:
            cap = self.project.find(self.app.primary)
            if cap is None:
                return None
            return cap.title, self.app.filtered(cap).local_points(), cap.scan.pose, cap.number
        if self.mode.get() == MODES[1] and self.imported:
            names = [lab for lab, _ in self.imported]
            k = names.index(self.file_pick.get()) if self.file_pick.get() in names else 0
            lab, scan = self.imported[k]
            known = scan.pose if scan.pose != Pose() else None
            return lab, self.app.filters().apply(scan).local_points(), known, None
        return None

    def import_scan(self) -> None:
        path = filedialog.askopenfilename(title="Scan à localiser", initialdir="data",
                                          filetypes=[("Scans lidar", "*.csv *.json"), ("Tous", "*.*")])
        if not path:
            return
        try:
            self.imported = read_any(path)
        except (ValueError, OSError, KeyError) as exc:
            self.app.status(f"Lecture impossible : {exc}")
            return
        names = [f"{lab}" for lab, _ in self.imported] or ["—"]
        self.file_pick.configure(values=names)
        self.file_pick.set(names[0])
        self.app.status(f"{Path(path).name} : {len(self.imported)} scan(s). Localisation du premier…")
        self.run()

    def _locate(self, pts: np.ndarray, exclude: int | None):
        """Localise des points (repère lidar). Rend un dict : pose, inliers, glob (ou None), temps, origine."""
        ref = self.reference_map(exclude)
        if ref is None:
            raise ValueError("il faut une carte : au moins un scan visible, ou une carte validée")
        base = self.references(ref)
        method = self.method_var.get()
        if self.start is not None:
            r = localize(method, pts, self.start, self.references(ref, self._near_points(self.start, exclude)))
            inl = float(_on_map(base.field, pts, r.pose).mean())
            if inl >= LOST:
                return {"pose": r.pose, "inliers": inl, "glob": None, "time": r.time_s,
                        "how": f"depuis la pose de départ · {method.split(' · ')[1]}"}
        glob = global_localize(pts, ref.grid, ref.segments, base.field, base.pyramid)
        pose, inl, t = glob.pose, glob.inliers, glob.time_s
        try:
            r = localize(method, pts, glob.pose, self.references(ref, self._near_points(glob.pose, exclude)))
            r_inl = float(_on_map(base.field, pts, r.pose).mean())
            t += r.time_s
            if r_inl >= inl:
                pose, inl = r.pose, r_inl
        except ValueError:
            pass
        return {"pose": pose, "inliers": inl, "glob": glob, "time": t,
                "how": f"globale + {method.split(' · ')[1]}"}

    def run(self) -> None:
        target = self._target()
        if target is None:
            self.app.status("Choisis un scan : clic dans la liste à gauche, ou « Importer un scan… ».")
            return
        title, pts, known, num = target
        if len(pts) < 20:
            self.app.status(f"{title} : trop peu de points pour se localiser.")
            return
        exclude = num if (num is not None and self.control.get()) else None
        self.app.status(f"Localisation de {title}…")
        self.app.root.update_idletasks()
        try:
            res = self._locate(pts, exclude)
        except ValueError as exc:
            self.app.status(str(exc))
            return
        glob = res["glob"]
        p = res["pose"]
        self.found = {"pose": p, "points": pts, "truth": known,
                      "others": [c for c, _ in glob.candidates[1:3]] if glob else []}
        sure = res["inliers"] >= LOST and not (glob and glob.ambiguous)
        rows = [("Scan", title if len(title) < 30 else title[:29] + "…"),
                ("Pose trouvée", f"{p.x:+.2f}  {p.y:+.2f}  {p.theta_deg:+.1f}°"),
                ("Points sur les murs", f"{res['inliers']:.0%}"),
                ("Confiance", "fiable ✓" if sure else ("ambiguë ⚠" if glob and glob.ambiguous else "faible ⚠")),
                ("Temps", f"{res['time'] * 1000:.0f} ms"), ("Méthode", res["how"])]
        msg = f"{title} : replacé en ({p.x:+.2f}, {p.y:+.2f}, {p.theta_deg:+.1f}°), {res['inliers']:.0%} des points sur les murs"
        if known is not None:
            e, a = pose_error(p, known)
            ok = e < 0.05 and a < 2.0
            rows += [("Écart à la pose connue", f"{e * 100:.1f} cm · {a:.2f}°"),
                     ("Verdict", "au même endroit ✓" if ok else "ailleurs ✗")]
            msg += f" · écart {e * 100:.1f} cm {'✓' if ok else '✗'}"
        if not sure:
            rows.append(("Pourquoi ?", "trop peu de murs vus (meubles proches, zone hors carte, salle symétrique)"))
        self.out.show(rows, highlight="Pose trouvée")
        self.app.status(msg)
        self.app.render()

    def compare(self) -> None:
        target = self._target()
        if target is None:
            return
        title, pts, known, num = target
        exclude = num if (num is not None and self.control.get()) else None
        ref = self.reference_map(exclude)
        if ref is None:
            return
        base = self.references(ref)
        start = self.start or global_localize(pts, ref.grid, ref.segments, base.field, base.pyramid).pose
        refs = self.references(ref, self._near_points(start, exclude))
        for w in self.table.winfo_children():
            w.destroy()
        what = "écart à la pose connue" if known is not None else "points sur les murs"
        ctk.CTkLabel(self.table, text=f"Même scan, même départ · {what}", font=S.font(11), text_color=S.MUTED,
                     anchor="w").grid(row=0, column=0, columnspan=3, sticky="w")
        for k, m in enumerate(METHODS, start=1):
            try:
                r = localize(m, pts, start, refs)
            except ValueError:
                continue
            if known is not None:
                e, a = pose_error(r.pose, known)
                txt, ok = f"{e * 100:.1f} cm {a:.1f}°", e < 0.05 and a < 2.0
            else:
                inl = float(_on_map(base.field, pts, r.pose).mean())
                txt, ok = f"{inl:.0%}", inl >= LOST
            ctk.CTkLabel(self.table, text=m, font=S.font(11), text_color=S.INK, anchor="w").grid(row=k, column=0, sticky="w")
            ctk.CTkLabel(self.table, text=txt, font=S.font(11, mono=True),
                         text_color=S.SUCCESS if ok else S.DANGER).grid(row=k, column=1, sticky="e", padx=6)
            ctk.CTkLabel(self.table, text=f"{r.time_s * 1000:.0f} ms", font=S.font(11, mono=True),
                         text_color=S.MUTED).grid(row=k, column=2, sticky="e")
        self.app.status(f"Comparaison terminée sur {title}.")

    # ================================================================== robot en direct
    def _demo(self) -> None:
        ref = self.reference_map()
        if ref is None or not ref.segments:
            self.app.status("Il faut une carte avec des murs (étape Cartes) pour la démo.")
            return
        way = [(p.x, p.y) for _, p in ref.scans] or [(0.0, 0.0)]
        order = [way.pop(0)]
        while way:
            k = min(range(len(way)), key=lambda i: np.hypot(way[i][0] - order[-1][0], way[i][1] - order[-1][1]))
            order.append(way.pop(k))
        self.app.map_demo([s.as_tuple() for s in ref.segments], order)
        self.app.status("Démo : un robot simulé roule dans ta carte. Clique sur « Suivre le robot ».")

    def toggle_follow(self) -> None:
        if self.tracker is not None:
            self.tracker = None
            self.follow_btn.configure(text="▶  Suivre le robot en temps réel", fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER)
            self.app.status("Suivi arrêté.")
            return
        if self.app.source is None:
            self.app.status("Aucun robot : « Démo sur ma carte », ou connecte le robot à l'étape 1 (Wi-Fi ou câble).")
            return
        ref = self.reference_map()
        if ref is None:
            self.app.status("Il faut une carte de référence (étape Cartes).")
            return
        self.tracker = Tracker(ref.grid, ref.segments)
        self.live_step = None
        self.follow_btn.configure(text="■  Arrêter le suivi", fg_color=S.WARN, hover_color="#E68600")
        self.app.status("Suivi : 1er tour localisé dans toute la carte, puis autour de la pose précédente.")

    def _relocalize(self) -> None:
        if self.tracker is not None:
            self.tracker.reset()
            self.app.status("Le prochain tour sera localisé dans toute la carte.")

    def on_live(self, frame) -> None:
        if self.mode.get() != MODES[2]:
            return
        kind = "démo dans ta carte" if self.app.demo_truth is not None else self.app.source_label
        self.live_info.configure(text=f"Connecté : {kind} · {int((frame.scan.ranges_m > 0).sum())} points par tour",
                                 text_color=S.SUCCESS)
        if self.tracker is None:
            self.live_points = self.app.filters().apply(frame.scan).local_points()
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
        mode = self.mode.get()
        target = self._target() if mode != MODES[2] else None
        exclude = target[3] if (target is not None and target[3] is not None and self.control.get()) else None
        ref = self.reference_map(exclude)
        if ref is None:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        app.map.draw_analysis(1.0 - ref.grid.probability(), ref.grid.extent(), list(ref.segments), [], None)
        layers, poses, trail = [], [], []
        if mode == MODES[2]:
            st = self.live_step
            if st is not None and self.live_points is not None:
                layers.append((local_to_global(self.live_points, st.pose), S.FOUND, 8, "o"))
                poses.append((st.pose, S.ACCENT, True))
            if self.tracker is not None and len(self.tracker.trail) > 1:
                tr = self.tracker.trail
                trail = [Segment(a.x, a.y, b.x, b.y) for a, b in zip(tr, tr[1:])]
            truth = app.demo_truth
            if truth is not None:
                poses.append((truth, S.FAINT, False))
            self.legend("bleu = robot localisé · vert = ses points · trait = trajet · gris = vraie pose (démo)")
        elif target is not None:
            _, pts, known, _ = target
            if known is not None:
                layers.append((local_to_global(pts, known), "#B8B8BE", 6, "o"))
                poses.append((known, "#8E8E93", False))
            if self.start is not None:
                poses.append((self.start, S.INITIAL, False))
            if self.found is not None and np.array_equal(self.found["points"], pts):
                for p in self.found["others"]:
                    poses.append((p, S.FAINT, False))
                layers.append((local_to_global(pts, self.found["pose"]), S.FOUND, 9, "o"))
                poses.append((self.found["pose"], S.FOUND, True))
            self.legend("gris = pose connue · vert = replacé par l'algorithme · orange = départ donné")
        app.map.draw_overlay(layers, [(trail, S.ACCENT, 2.0)] if trail else [], poses)
