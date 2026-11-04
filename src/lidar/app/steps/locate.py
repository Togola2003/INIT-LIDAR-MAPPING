"""Étape 5 · Localiser : la carte validée reste FIXE, un scan LIBRE vient s'y poser.

- « Scan de la liste » : un clic sur un scan de la liste le sort du projet comme un scan libre
  (le projet n'est pas modifié). On le glisse, on le tourne (clic droit glissé), on le « perd » au
  hasard… puis « Localiser » : l'algorithme le ramène à sa place sur la carte figée, avec une
  animation et une surbrillance. Sa pose connue (dans le projet) sert à afficher l'écart.
- « Fichier importé » : pareil avec un scan qui n'est pas dans le projet (posé à la pose notée dans
  le fichier, avec son robot).
- « Robot en direct » : chaque tour du lidar (Wi-Fi, câble ou démo) est replacé sur la carte,
  en continu : global au premier tour, puis autour de la pose précédente.
Calcul : depuis la pose où on a laissé le scan, la méthode choisie cherche autour ; une recherche
GLOBALE dans toute la carte est faite aussi, et on garde la pose qui pose le plus de points sur les murs.
"""

from __future__ import annotations

from pathlib import Path
from tkinter import filedialog

import customtkinter as ctk
import numpy as np

from ...geometry import Pose, local_to_global, move_rigid
from ...localization import METHODS, References, global_localize, localize, pose_error
from ...localization import grid as G
from ...localization import scans as SC
from ...localization import vector as V
from ...localization.globalloc import _on_map

from ...refmap import build_reference
from ...scope.mapview import Editor
from ...scope.session import read_any
from .. import style as S
from ..components import PilotPad
from ..live import trail_segments
from .base import Step

MODES = ["Scan de la liste", "Fichier importé", "Robot en direct"]
REFS = {"Grid map": "grid", "Carte vectorielle": "vecteur", "Scan voisin": "scans"}
LOST = 0.35            # moins de 35 % des points sur les murs : pose jugée fausse
ANIM_FRAMES, ANIM_MS = 16, 28
MODE_TEXT = {"suivi": "suivi", "global": "relocalisé (toute la carte)", "incertain": "incertain : pose prévue gardée",
             "perdu": "perdu : recherche en cours"}
PROBE = "probe"        # clé du scan libre pour les gestes de la carte
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
    headline = "La carte validée reste fixe. Glisse / tourne le scan libre n'importe où, puis « Localiser » : il revient se poser."
    course = ("Scan matching : on cherche la transformation (x, y, θ) qui pose au mieux les points du scan sur la "
              "carte de référence. Depuis une pose de départ, on cherche autour (exhaustive, multi-résolution, "
              "gradient) ; sans bonne pose de départ, on fouille toute la carte (localisation globale). En direct, "
              "la pose précédente sert de départ au tour suivant.")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.probe: dict | None = None      # scan libre : title, points (repère lidar), pose, truth, source
        self.flash = 0                      # surbrillance après un placement (nombre de rendus restants)
        self.anim = None
        self.imported: list = []
        self.following = False              # suivi en direct lancé (calcul en arrière-plan : app.live)
        self.live_step = None
        self.live_points = None
        self.trail: tuple = ()
        self._refs: dict = {}
        self._temp_ref = (None, None)

    # ================================================================== panneau
    def build(self, panel) -> None:
        card = S.Card(panel, "Carte de référence (fixe)")
        card.pack(fill="x", pady=(0, 10))
        self.ref_info = S.hint(card.body, "")
        self.validate_btn = S.button(card.body, "Valider la carte actuelle", lambda: self.app.steps[3].validate(), "ghost")
        self.validate_btn.pack(fill="x")

        ctk.CTkLabel(panel, text="Que localiser ?", font=S.font(13, "bold"), text_color=S.INK, anchor="w").pack(fill="x")
        self.mode = S.segmented(panel, MODES, MODES[0], lambda v: self._mode_changed())
        self.mode.pack(fill="x", pady=(4, 10))

        # ---- scan libre (liste ou fichier)
        self.probe_box = S.Card(panel, "Scan à replacer")
        self.probe_hint = S.hint(self.probe_box.body, "")
        self.probe_label = ctk.CTkLabel(self.probe_box.body, text="—", font=S.font(13, "bold"), text_color=S.MUTED,
                                        anchor="w")
        self.probe_label.pack(fill="x")
        self.file_row = ctk.CTkFrame(self.probe_box.body, fg_color="transparent")
        S.button(self.file_row, "Importer un scan…", self.import_scan, width=150).pack(side="left")
        self.file_pick = ctk.CTkOptionMenu(self.file_row, values=["—"], width=170, height=30, font=S.font(12),
                                           fg_color=S.SURFACE_2, button_color=S.BORDER, text_color=S.INK,
                                           command=lambda v: self._probe_from_file())
        self.file_pick.pack(side="left", padx=6)
        S.hint(self.probe_box.body, "Souris : glisser = déplacer · clic droit glissé = tourner · Ctrl + molette = ±1°.")
        row = ctk.CTkFrame(self.probe_box.body, fg_color="transparent")
        row.pack(fill="x", pady=(4, 0))
        S.button(row, "🎲  Le perdre au hasard", self.lose, width=170).pack(side="left")
        S.button(row, "↺  Pose connue", self.back_to_known, "ghost", width=120).pack(side="left", padx=6)
        self.locate_btn = S.button(self.probe_box.body, "◎  Localiser", self.run, "primary", height=46,
                                   font=S.font(15, "bold"))
        self.locate_btn.pack(fill="x", pady=(10, 0))

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
        self.pad = PilotPad(self.pilot_card.body, lambda: self.app.robot, self.app.status, self.app.on_command)
        self.pad.pack(fill="x")

        # ---- résultat
        self.result_card = S.Card(panel, "Résultat")
        self.result_card.pack(fill="x", pady=(0, 10))
        self.out = S.KeyValues(self.result_card.body)
        self.out.pack(fill="x")
        self.out.show([("—", "en attente")])

        # ---- avancé
        adv = S.Collapsible(panel, "Avancé : référence, méthode, comparaison")
        adv.pack(fill="x", pady=(0, 10))
        b = adv.body
        ctk.CTkLabel(b, text="Référence", font=S.font(12, "bold"), text_color=S.INK, anchor="w").pack(fill="x")
        self.ref = S.segmented(b, list(REFS), "Grid map", lambda v: self._ref_changed())
        self.ref.pack(fill="x", pady=(2, 8))
        ctk.CTkLabel(b, text="Méthode (depuis la pose où tu as laissé le scan)", font=S.font(12, "bold"),
                     text_color=S.INK, anchor="w").pack(fill="x")
        self.method_var = ctk.StringVar()
        self.method_box = ctk.CTkFrame(b, fg_color="transparent")
        self.method_box.pack(fill="x")
        self.method_hint = S.hint(b, "", S.INK)
        self.with_global = S.toggle(b, "Vérifier aussi par une recherche globale (conseillé)", lambda: None, on=True)
        self.with_global.pack(anchor="w", pady=(2, 4))
        self.control = S.toggle(b, "Contrôle : carte construite sans ce scan", self._invalidate)
        self.control.pack(anchor="w", pady=(0, 6))
        S.button(b, "Comparer toutes les méthodes (même départ)", self.compare).pack(fill="x", pady=(2, 6))
        self.table = ctk.CTkFrame(b, fg_color="transparent")
        self.table.pack(fill="x")
        self._ref_changed()
        self._mode_changed()

    # ================================================================== modes, réglages
    def _mode_changed(self) -> None:
        mode = self.mode.get()
        live = mode == MODES[2]
        if live:
            self.probe_box.pack_forget()
            self.live_box.pack(fill="x", before=self.result_card)
        else:
            self.live_box.pack_forget()
            self.probe_box.pack(fill="x", pady=(0, 10), before=self.result_card)
            if self.following:
                self.toggle_follow()
        if mode == MODES[1]:
            self.file_row.pack(fill="x", pady=(6, 0), after=self.probe_label)
            self.probe_hint.configure(text="Importe un scan (CSV du TP, RobotControl, MapTool, session…). Il n'est "
                                           "pas ajouté à la carte.")
            self._probe_from_file()
        else:
            self.file_row.pack_forget()
            self.probe_hint.configure(text="Clique un scan dans la liste à gauche : il devient un scan libre "
                                           "(le projet n'est pas modifié).")
            self._probe_from_list()
        self._set_editor()
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
        self.app.render()

    def on_project(self) -> None:
        ref = self.project.reference_map
        self.ref_info.configure(text=(f"Carte validée : {ref.describe()}" if ref else
                                      "Aucune carte validée : une carte provisoire (scans visibles + retouches) est "
                                      "utilisée. Valide-la à l'étape Cartes pour la figer."),
                                text_color=S.SUCCESS if ref else S.WARN)
        if ref:
            self.validate_btn.pack_forget()
        else:
            self.validate_btn.pack(fill="x")

    def on_selection(self) -> None:
        if self.mode.get() == MODES[0]:
            self._probe_from_list()
            self.app.render()

    def enter(self) -> None:
        self.on_project()
        if self.mode.get() == MODES[0] and (self.probe is None or self.probe["source"] != self.app.primary):
            self._probe_from_list()
        self._set_editor()

    def leave(self) -> None:
        if self.following:
            self.toggle_follow()
        self._stop_anim()

    def reset(self) -> None:
        if self.following:
            self.app.live.stop()
        self.probe, self.following, self.live_step, self.trail = None, False, None, ()
        self.imported = []
        self._refs.clear()

    def on_key(self, key: str) -> None:
        if self.mode.get() == MODES[2]:
            self.pad.key(key)
        elif key == "Return":
            self.run()

    # ================================================================== le scan libre
    def _probe_from_list(self) -> None:
        cap = self.project.find(self.app.primary)
        if cap is None:
            self.probe = None
            self.probe_label.configure(text="Aucun scan : clique un scan dans la liste à gauche", text_color=S.MUTED)
            return
        self.probe = {"title": cap.title, "points": self.app.filtered(cap).local_points(), "pose": cap.scan.pose,
                      "truth": cap.scan.pose, "source": cap.number, "color": cap.color}
        self.probe_label.configure(text=f"● {cap.title}", text_color=cap.color)
        self.out.show([("Scan", self._short(cap.title)), ("À faire", "glisse-le ailleurs, puis ◎ Localiser")])

    def _probe_from_file(self) -> None:
        names = [lab for lab, _ in self.imported]
        if not names:
            self.probe = None
            self.probe_label.configure(text="Aucun scan importé", text_color=S.MUTED)
            return
        k = names.index(self.file_pick.get()) if self.file_pick.get() in names else 0
        lab, scan = self.imported[k]
        known = scan.pose if scan.pose != Pose() else None
        pose = known or self._map_center()
        self.probe = {"title": lab, "points": self.app.filters().apply(scan).local_points(), "pose": pose,
                      "truth": known, "source": f"fichier:{k}", "color": S.ACCENT}
        self.probe_label.configure(text=f"● {lab}", text_color=S.ACCENT)
        self.out.show([("Scan", self._short(lab)), ("À faire", "◎ Localiser (ou glisse-le d'abord)")])
        self.app.render()

    def _map_center(self) -> Pose:
        ref = self.reference_map()
        if ref is None:
            return Pose()
        x0, x1, y0, y1 = ref.grid.extent()
        return Pose((x0 + x1) / 2, (y0 + y1) / 2, 0.0)

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
        names = [lab for lab, _ in self.imported] or ["—"]
        self.file_pick.configure(values=names)
        self.file_pick.set(names[0])
        self.app.status(f"{Path(path).name} : {len(self.imported)} scan(s). Glisse-le où tu veux, puis ◎ Localiser.")
        self._probe_from_file()

    def _set_editor(self) -> None:
        if self.app.step is not self:
            return
        if self.mode.get() == MODES[2]:
            self.app.map.set_editor(None)
            return
        self.app.map.set_editor(Editor(pick=self._pick, selection=lambda: {PROBE} if self.probe else set(),
                                       select=lambda key, additive: None, pivot=lambda keys: self._pivot(),
                                       commit=self._moved))

    def _pick(self, x: float, y: float, tol: float):
        if self.probe is None:
            return None
        p = self.probe["pose"]
        pts = np.vstack([local_to_global(self.probe["points"], p), [[p.x, p.y]]])
        return PROBE if float(np.min(np.hypot(*(pts - np.array([x, y])).T))) < max(tol, 0.15) else None

    def _pivot(self) -> tuple[float, float]:
        p = self.probe["pose"]
        return (p.x, p.y)

    def _moved(self, keys, dx, dy, dth, pivot) -> None:
        self._stop_anim()
        self.probe = {**self.probe, "pose": move_rigid(self.probe["pose"], dx, dy, dth, pivot), "placed": False}
        self.flash = 0
        self.app.render()

    def lose(self) -> None:
        """Pose au hasard dans la carte (le « robot kidnappé ») : pour tester la localisation."""
        if self.probe is None:
            return
        ref = self.reference_map()
        x0, x1, y0, y1 = ref.grid.extent()
        rng = np.random.default_rng()
        self._stop_anim()
        self.probe = {**self.probe, "pose": Pose(float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1)),
                                                 float(rng.uniform(-180, 180))), "placed": False}
        self.flash = 0
        self.app.status("Scan perdu au hasard. ◎ Localiser pour le ramener à sa place.")
        self.app.render()

    def back_to_known(self) -> None:
        if self.probe is not None and self.probe["truth"] is not None:
            self._stop_anim()
            self.probe = {**self.probe, "pose": self.probe["truth"], "placed": False}
            self.app.render()

    @staticmethod
    def _short(text: str) -> str:
        return text if len(text) < 30 else text[:29] + "…"

    # ================================================================== références
    def reference_map(self, exclude: int | None = None):
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

    def _near_points(self, pose: Pose, exclude) -> np.ndarray | None:
        caps = [c for c in self.project.visible if c.number != exclude]
        if not caps:
            return None
        near = min(caps, key=lambda c: np.hypot(c.scan.pose.x - pose.x, c.scan.pose.y - pose.y))
        return self.app.points(near)

    def _exclude(self):
        src = self.probe["source"] if self.probe else None
        return src if (isinstance(src, int) and self.control.get()) else None

    # ================================================================== localiser
    def _locate(self, pts: np.ndarray, start: Pose, exclude):
        """Depuis `start` avec la méthode choisie, + recherche globale ; on garde la pose qui pose le plus
        de points sur les murs. Rend un dict : pose, inliers, ambiguous, time, how."""
        ref = self.reference_map(exclude)
        if ref is None:
            raise ValueError("il faut une carte : valide-la à l'étape Cartes")
        base = self.references(ref)
        method = self.method_var.get()
        cands = []
        try:
            r = localize(method, pts, start, self.references(ref, self._near_points(start, exclude)))
            cands.append((float(_on_map(base.field, pts, r.pose).mean()), r.pose, r.time_s,
                          f"{method.split(' · ')[1]} depuis la pose laissée", False))
        except ValueError:
            pass
        if self.with_global.get() or not cands or cands[0][0] < LOST:
            g = global_localize(pts, ref.grid, ref.segments, base.field, base.pyramid)
            cands.append((g.inliers, g.pose, g.time_s, "recherche globale", g.ambiguous))
        best = max(cands, key=lambda c: c[0])
        return {"pose": best[1], "inliers": best[0], "ambiguous": best[4] and best is cands[-1],
                "time": sum(c[2] for c in cands), "how": best[3]}

    def run(self) -> None:
        if self.probe is None:
            self.app.status("Choisis d'abord un scan (liste à gauche) ou importe-en un.")
            return
        if len(self.probe["points"]) < 20:
            self.app.status("Ce scan a trop peu de points pour être localisé.")
            return
        self.locate_btn.configure(text="Localisation…", state="disabled")
        self.app.root.update_idletasks()
        try:
            res = self._locate(self.probe["points"], self.probe["pose"], self._exclude())
        except ValueError as exc:
            self.app.status(str(exc))
            return
        finally:
            self.locate_btn.configure(text="◎  Localiser", state="normal")
        p, title, known = res["pose"], self.probe["title"], self.probe["truth"]
        sure = res["inliers"] >= LOST and not res["ambiguous"]
        rows = [("Scan", self._short(title)),
                ("Pose trouvée", f"{p.x:+.2f}  {p.y:+.2f}  {p.theta_deg:+.1f}°"),
                ("Points sur les murs", f"{res['inliers']:.0%}"),
                ("Confiance", "fiable ✓" if sure else ("ambiguë ⚠" if res["ambiguous"] else "faible ⚠")),
                ("Temps", f"{res['time'] * 1000:.0f} ms"), ("Trouvée par", res["how"])]
        msg = f"{title} replacé en ({p.x:+.2f}, {p.y:+.2f}, {p.theta_deg:+.1f}°) · {res['inliers']:.0%} des points sur les murs"
        if known is not None:
            e, a = pose_error(p, known)
            rows.append(("Écart à la pose connue", f"{e * 100:.1f} cm · {a:.2f}°"))
            msg += f" · écart à la pose connue {e * 100:.1f} cm"
        if not sure:
            rows.append(("Pourquoi ?", "peu de murs vus (meubles proches, zone hors carte, salle symétrique)"))
        self.out.show(rows, highlight="Pose trouvée")
        self.app.status(msg)
        self._animate(self.probe["pose"], p)

    # ------------------------------------------------------------------ animation
    def _animate(self, a: Pose, b: Pose) -> None:
        self._stop_anim()
        dth = (b.theta_deg - a.theta_deg + 180) % 360 - 180

        def frame(k: int) -> None:
            t = k / ANIM_FRAMES
            t = 1 - (1 - t) ** 3                          # ralenti à l'arrivée
            self.probe = {**self.probe, "pose": Pose(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                                                     a.theta_deg + t * dth)}
            self.flash = 1
            self.render()
            if k < ANIM_FRAMES:
                self.anim = self.app.root.after(ANIM_MS, lambda: frame(k + 1))
            else:
                self.probe = {**self.probe, "pose": b, "placed": True}
                self.anim = None
                self.flash = 40                           # surbrillance quelques instants
                self.render()

        frame(1)

    def _stop_anim(self) -> None:
        if self.anim is not None:
            self.app.root.after_cancel(self.anim)
            self.anim = None

    def compare(self) -> None:
        if self.probe is None:
            return
        pts, start, known = self.probe["points"], self.probe["pose"], self.probe["truth"]
        exclude = self._exclude()
        ref = self.reference_map(exclude)
        if ref is None:
            return
        base = self.references(ref)
        refs = self.references(ref, self._near_points(start, exclude))
        for w in self.table.winfo_children():
            w.destroy()
        what = "écart à la pose connue" if known is not None else "points sur les murs"
        ctk.CTkLabel(self.table, text=f"Depuis la pose où tu as laissé le scan · {what}", font=S.font(11),
                     text_color=S.MUTED, anchor="w").grid(row=0, column=0, columnspan=3, sticky="w")
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
        self.app.status("Comparaison terminée : chaque méthode part de la pose où tu as laissé le scan.")

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
        if self.following:
            self.following = False
            self.app.live.stop()
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
        self.app.live.start(ref.grid, ref.segments)       # pas de pose de départ : recherche globale
        self.following, self.live_step, self.trail = True, None, ()
        self.follow_btn.configure(text="■  Arrêter le suivi", fg_color=S.WARN, hover_color="#E68600")
        self.app.status("Suivi : chaque tour du lidar est replacé sur la carte (global au début, puis autour de la "
                        "pose prévue par les commandes). Le calcul tourne en arrière-plan.")

    def _relocalize(self) -> None:
        if self.following:
            self.app.live.relocalize()
            self.trail = ()
            self.app.status("Le prochain tour sera localisé dans toute la carte.")

    def on_live(self, frame) -> None:
        if self.mode.get() != MODES[2]:
            return
        kind = "démo dans ta carte" if self.app.demo_truth is not None else self.app.source_label
        self.live_info.configure(text=f"Connecté : {kind} · {int((frame.scan.ranges_m > 0).sum())} points par tour",
                                 text_color=S.SUCCESS)
        if not self.following:
            self.live_points = self.app.filters().apply(frame.scan).local_points()
            return
        pts = self.app.filters().apply(frame.scan).local_points()
        if len(pts) >= 20:
            self.app.live.submit(pts)       # le résultat arrive plus tard dans on_track

    def on_track(self, result) -> None:
        if not self.following:
            return
        st = result.step
        self.live_step, self.live_points, self.trail = st, result.points, result.trail
        rows = [("Pose du robot", f"{st.pose.x:+.2f}  {st.pose.y:+.2f}  {st.pose.theta_deg:+.1f}°"),
                ("Mode", MODE_TEXT.get(st.mode, st.mode)),
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
        ref = self.reference_map(self._exclude() if self.mode.get() != MODES[2] else None)
        if ref is None:
            self.clear_map()
            app.map.draw_overlay([], [], [])
            return
        app.map.draw_analysis(1.0 - ref.grid.probability(), ref.grid.extent(), list(ref.segments), [], None)
        layers, poses, trail = [], [], []
        if self.mode.get() == MODES[2]:
            st = self.live_step
            if st is not None and self.live_points is not None:
                g = local_to_global(self.live_points, st.pose)
                layers += [(g, "#34C75955", 60, "o"), (g, S.FOUND, 9, "o")]
                poses.append((st.pose, S.ACCENT, True))
            trail = trail_segments(self.trail)
            if app.demo_truth is not None:
                poses.append((app.demo_truth, S.FAINT, False))
            self.legend("bleu = robot replacé · vert = son tour de lidar · trait = trajet · gris = vraie pose (démo)")
        elif self.probe is not None:
            p = self.probe["pose"]
            g = local_to_global(self.probe["points"], p)
            if self.probe["truth"] is not None:
                poses.append((self.probe["truth"], "#C7C7CC", False))
            if self.flash > 0 or self.probe.get("placed"):
                if self.flash > 0:
                    layers.append((g, "#34C75944", 90, "o", PROBE))          # halo lumineux juste après la pose
                    self.flash -= 1
                layers.append((g, S.FOUND, 12, "o", PROBE))
                poses.append((p, S.FOUND, True, PROBE))
            else:
                layers.append((g, S.INITIAL, 10, "o", PROBE))
                poses.append((p, S.INITIAL, True, PROBE))
            self.legend("orange = scan libre (glisser / tourner) · vert brillant = posé par ◎ Localiser · "
                        "gris = pose connue")
        app.map.draw_overlay(layers, [(trail, S.ACCENT, 2.0)] if trail else [], poses)
