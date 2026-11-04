"""Étape 1 · Récolter : relier le robot, faire le SLAM pas à pas (ou de simples captures), importer.

SLAM pas à pas, consigne du Jour 3 (le calcul est dans lidar/slam.py, ici il n'y a que l'interface) :
  1. 1er scan, n'importe où : il devient la PREMIÈRE RÉFÉRENCE (origine 0, 0, 0 de la carte) ;
  2. on déplace un peu le robot (pavé de pilotage Wi-Fi, ou à la main). Entre deux scans, il est suivi en
     direct sur la carte déjà construite (app/live.py, en arrière-plan) ;
  3. on scanne à l'arrêt (N tours) : le scan est LOCALISÉ sur la référence grâce à ses points communs
     (orange), et ses autres points sont la PARTIE NOUVELLE (vert) ;
  4. Garder : référence + nouveau scan = nouvelle référence. On recommence par petits pas ;
  5. quand un scan retrouve un scan ancien (boucle), « Optimiser toute la carte » recale toutes les poses.
"""

from __future__ import annotations

import customtkinter as ctk

from ...geometry import Pose, local_to_global
from ...robot import DEFAULT_HOST
from ...scope.collector import COMBINE_MODES, Collection, finish
from ...scope.sources import FORMAT_TELEPLOT
from ...slam import SlamMap
from .. import project as P
from .. import style as S
from ..components import PilotPad
from ..live import trail_segments
from ..tools import PoseTool
from .base import Step

BAUDS = ["115200", "230400", "460800", "921600"]
SOURCES = ["Câble USB", "Wi-Fi", "Démo"]
MIN_POINTS = 20


class CollectStep(Step):
    title = "Récolter"
    subtitle = "robot, SLAM pas à pas"
    headline = ("SLAM pas à pas : scanne, garde, déplace un peu, rescanne. Chaque scan se pose sur les précédents "
                "(orange = commun, vert = partie nouvelle).")
    course = ("SLAM = localisation et cartographie simultanées. Le 1er scan est la référence. Chaque nouveau scan "
              "a des points communs avec elle : on cherche la pose (x, y, θ) qui les superpose (scan matching), "
              "puis on ajoute ses autres points à la carte. Petits pas = beaucoup de points communs = pose sûre. "
              "Le lidar mesure en mètres : l'échelle de la carte ne change jamais.")

    def __init__(self, app) -> None:
        super().__init__(app)
        self.robot = Pose()
        self.collection: Collection | None = None
        self.preview = None             # (scan, info, raw, fmt, SlamStep | None)
        self.trail: tuple = ()          # trajet suivi en direct (Pose ou None = coupure)
        self._map_key = None
        self._map: SlamMap | None = None
        self._announced: set[int] = set()   # scans anciens déjà signalés comme « boucle refermée »

    # ================================================================== panneau
    def build(self, panel) -> None:
        self._build_robot(panel)
        self._build_slam(panel)
        self.pilot_card = S.Card(panel, "Piloter le robot (Wi-Fi)", "Petits déplacements : un appui = un court "
                                                                     "mouvement, puis arrêt automatique.")
        self.pad = PilotPad(self.pilot_card.body, lambda: self.app.robot, self.app.status, self.app.on_command)
        self.pad.pack(fill="x")
        self._build_settings(panel)
        self._build_pose(panel)
        card = S.Card(panel, "Importer", "Session .lidar.json, CSV du TP, RobotControl, MapTool…")
        card.pack(fill="x", pady=(0, 10))
        S.button(card.body, "Ouvrir des scans…", self.app.open_files).pack(fill="x")
        self.refresh_ports()
        self._source_changed()
        self._refresh_status()

    def _build_robot(self, panel) -> None:
        card = S.Card(panel, "Robot", "Câble USB, Wi-Fi (réseau du robot, firmware 02_LidarRobot_WiFi), ou démo.")
        card.pack(fill="x", pady=(0, 10))
        self.robot_card = card
        b = card.body
        self.source = S.segmented(b, SOURCES, SOURCES[0], lambda v: self._source_changed())
        self.source.pack(fill="x", pady=(0, 8))
        row = ctk.CTkFrame(b, fg_color="transparent")
        row.pack(fill="x")
        menu = dict(height=30, font=S.font(12), fg_color=S.SURFACE_2, button_color=S.BORDER,
                    button_hover_color=S.FAINT, text_color=S.INK)
        self.port = ctk.CTkOptionMenu(row, values=["—"], width=110, dropdown_font=S.font(12), **menu)
        self.port.pack(side="left")
        S.button(row, "↻", self.refresh_ports, width=34).pack(side="left", padx=6)
        self.baud = ctk.CTkOptionMenu(row, values=BAUDS, width=100, **menu)
        self.baud.pack(side="left")
        self.wifi_row = ctk.CTkFrame(b, fg_color="transparent")
        ctk.CTkLabel(self.wifi_row, text="Adresse", font=S.font(12), text_color=S.MUTED).pack(side="left")
        self.host = ctk.StringVar(value=DEFAULT_HOST)
        ctk.CTkEntry(self.wifi_row, textvariable=self.host, width=160, height=30, font=S.font(12, mono=True),
                     fg_color=S.SURFACE_2, border_color=S.BORDER, text_color=S.INK, corner_radius=8).pack(side="left", padx=8)
        self.port_row = row
        self.connect_btn = S.button(b, "Connecter", self.toggle_connect, "primary")
        self.connect_btn.pack(fill="x", pady=(10, 2))
        self.source_hint = S.hint(b, "")
        self.live_info = S.KeyValues(b)
        self.live_info.pack(fill="x", pady=(4, 0))

    def _build_slam(self, panel) -> None:
        card = S.Card(panel, "SLAM pas à pas", "① scanne → Garder (1er scan = référence)   ② déplace un peu   "
                                               "③ rescanne → Garder   … jusqu'à toute la salle")
        card.pack(fill="x", pady=(0, 10))
        self.slam_card = card
        b = card.body
        self.map_status = ctk.CTkLabel(b, text="", font=S.font(14, "bold"), text_color=S.INK, anchor="w")
        self.map_status.pack(fill="x")
        self.next_hint = S.hint(b, "")
        self.next_hint.pack(fill="x", pady=(0, 4))
        self.capture_btn = S.button(b, "Scanner", self.capture, "primary", height=42, font=S.font(14, "bold"))
        self.capture_btn.pack(fill="x", pady=(4, 4))
        self.bar = ctk.CTkProgressBar(b, height=6, progress_color=S.ACCENT, fg_color=S.SURFACE_2, corner_radius=3)
        self.bar.set(0)
        self.bar.pack(fill="x", pady=(2, 4))
        self.preview_box = ctk.CTkFrame(b, fg_color="#FFF6E5", corner_radius=12)
        self.preview_title = ctk.CTkLabel(self.preview_box, text="", font=S.font(12, "bold"), text_color=S.WARN,
                                          anchor="w", justify="left", wraplength=300)
        self.preview_title.pack(fill="x", padx=12, pady=(10, 0))
        self.preview_info = S.KeyValues(self.preview_box)
        self.preview_info.pack(fill="x", padx=12, pady=4)
        row = ctk.CTkFrame(self.preview_box, fg_color="transparent")
        row.pack(fill="x", padx=8, pady=(0, 10))
        S.button(row, "Garder", self.keep, "primary", width=90).pack(side="left", padx=3)
        S.button(row, "Refaire", self.redo, width=80).pack(side="left", padx=3)
        S.button(row, "Jeter", self.drop, "danger", width=70).pack(side="left", padx=3)
        self.track_info = S.hint(b, "")
        self.track_info.pack(fill="x", pady=(4, 0))
        self.loop_info = S.hint(b, "", color=S.ACCENT)
        self.loop_info.pack(fill="x")
        S.button(b, "Optimiser toute la carte (fermer les boucles)", self.optimize, "ghost").pack(fill="x", pady=(6, 0))
        S.hint(b, "C = scanner · Entrée = garder · Échap = jeter").pack(fill="x", pady=(4, 0))

    def _build_settings(self, panel) -> None:
        card = S.Card(panel, "Réglages du scan")
        card.pack(fill="x", pady=(0, 10))
        b = card.body
        self.tours = S.Slider(b, "Tours par scan (robot immobile)", 1, 30, 10, "{:.0f}", lambda v: None, steps=29)
        self.tours.pack(fill="x")
        self.combine = S.segmented(b, list(COMBINE_MODES), COMBINE_MODES[0], lambda v: None)
        self.combine.pack(fill="x", pady=(2, 6))
        S.hint(b, "Médiane : un point par angle, bruit réduit. Tous : nuage plus dense.")
        self.auto = S.toggle(b, "Poser chaque scan sur la carte déjà construite", self.render, on=True)
        self.auto.pack(anchor="w", pady=(6, 2))
        self.follow = S.toggle(b, "Suivre le robot en direct entre deux scans", self._follow_changed, on=True)
        self.follow.pack(anchor="w", pady=2)

    def _build_pose(self, panel) -> None:
        card = S.Card(panel, "Pose du robot", "Mise à jour toute seule par le SLAM. À la main : mesurée au mètre, ou "
                                             "placée à la souris (clic = position, glisser = orientation).")
        card.pack(fill="x", pady=(0, 10))
        self.pose_field = S.PoseField(card.body)
        self.pose_field.pack(anchor="w")
        row = ctk.CTkFrame(card.body, fg_color="transparent")
        row.pack(fill="x", pady=(6, 0))
        S.button(row, "Placer sur la carte", self.place_on_map, width=150).pack(side="left")
        S.button(row, "Appliquer", self.apply_pose_field, "ghost", width=90).pack(side="right")

    # ================================================================== connexion
    def _source_changed(self) -> None:
        src = self.source.get()
        self.source_hint.configure(text={
            "Wi-Fi": "Connecte d'abord le PC au Wi-Fi du robot (mot de passe 12345678), puis Connecter.",
            "Démo": "Démo : une salle simulée, sans robot.",
            "Câble USB": "Ferme le moniteur série d'Arduino avant de connecter."}[src], text_color=S.MUTED)
        if src == "Wi-Fi":
            self.port_row.pack_forget()
            self.wifi_row.pack(fill="x", before=self.connect_btn)
            self.pilot_card.pack(fill="x", pady=(0, 10), after=self.slam_card)
        else:
            self.wifi_row.pack_forget()
            self.port_row.pack(fill="x", before=self.connect_btn)
            self.pilot_card.pack_forget()

    def refresh_ports(self) -> None:
        ports = self.app.ports() or ["—"]
        self.port.configure(values=ports)
        if self.port.get() not in ports:
            self.port.set(ports[0])

    def toggle_connect(self) -> None:
        if self.app.source is not None:
            self.app.disconnect()
            self.connect_btn.configure(text="Connecter")
            return
        if self.source.get() == "Démo":
            self.app.demo()
        elif self.source.get() == "Wi-Fi":
            self.app.connect_wifi(self.host.get().strip() or DEFAULT_HOST)
        elif self.port.get() == "—":
            self.app.status("Aucun port série : branche l'ESP32 puis ↻.")
            return
        else:
            self.app.connect(self.port.get(), int(self.baud.get()))
        self.connect_btn.configure(text="Déconnecter")

    # ================================================================== pose donnée à la main
    def place_on_map(self) -> None:
        """Un seul placement : le clic suivant pose le robot, puis la souris redevient normale."""
        self.app.set_tool(PoseTool(self.app.map, self._placed, S.ROBOT))
        self.app.status("Clique sur la carte à la position du robot, glisse pour son orientation.")

    def _placed(self, pose: Pose) -> None:
        self.app.set_tool(None)
        self.set_robot(pose)
        self._start_follow()             # le suivi repart de la pose donnée à la main

    def set_robot(self, pose: Pose) -> None:
        self.robot = pose
        self.pose_field.set(pose.x, pose.y, pose.theta_deg)
        self.render()

    def apply_pose_field(self) -> None:
        try:
            self.set_robot(Pose(*self.pose_field.get()))
        except ValueError:
            self.app.status("x, y et θ doivent être des nombres.")
            return
        self._start_follow()

    # ================================================================== scan
    def capture(self) -> None:
        if self.collection is not None:
            self._finish()
            return
        if self.app.source is None:
            self.app.status("Connecte le robot (ou la démo) pour scanner, ou importe des scans.")
            return
        self.drop(quiet=True)
        self.collection = Collection(int(round(self.tours.get())), self.robot)
        self.capture_btn.configure(text="Arrêter le scan", fg_color=S.WARN, hover_color="#E68600")

    def on_frame(self, frame) -> None:
        if self.collection is not None:
            self.collection = self.collection.add(frame)
            self.bar.set(self.collection.progress)
            self.app.status(f"Scan : {self.collection.count} / {self.collection.target} tours… ne bouge pas le robot.")
            if self.collection.done:
                self._finish()

    def _finish(self) -> None:
        col, self.collection = self.collection, None
        self.capture_btn.configure(fg_color=S.ACCENT, hover_color=S.ACCENT_HOVER)
        self.bar.set(0)
        self._refresh_status()
        if col is None or not col.count:
            self.app.status("Aucun tour reçu : scan annulé.")
            return
        scan, info = finish(col, self.combine.get())
        scan, step, title, rows = self._locate(scan)
        self.preview = (scan, info, tuple(f.raw for f in col.frames), col.frames[-1].fmt, step)
        noise = info["bruit_m"]
        quality = "—" if noise is None else ("bon" if noise < 0.02 else "moyen")
        self.preview_title.configure(text=title, text_color=S.SUCCESS if step is None or step.ok else S.WARN)
        self.preview_info.show(rows + [("Tours", f"{info['tours']} / {info['voulus']}"),
                                       ("Points", f"{info['points']}"),
                                       ("Bruit", "—" if noise is None else f"{noise * 100:.1f} cm ({quality})")],
                               highlight="Points communs")
        self.preview_box.pack(fill="x", pady=(6, 0), after=self.bar)
        self.app.status("Aperçu prêt : Garder (Entrée), Refaire, ou Jeter (Échap).")
        self.render()

    def _locate(self, scan):
        """Localisation du nouveau scan sur la référence (SLAM). Rendu : scan posé, SlamStep, titre, lignes."""
        if not self.auto.get():
            return scan, None, "Posé à la pose du robot (SLAM désactivé)", []
        slam = self.current_map()
        if slam is None:
            first = scan.with_pose(self.robot if self.project.captures else Pose())
            return first, None, "1er scan : il devient la référence (origine de la carte)", []
        pts = self.app.filters().apply(scan).local_points()
        if len(pts) < MIN_POINTS:
            return scan, None, "⚠ Trop peu de points après les filtres : posé à la pose du robot", []
        self.app.status("Localisation du scan sur la carte…")
        self.app.root.update_idletasks()
        try:
            st = slam.locate(pts, self.robot)
        except Exception as exc:                       # jamais de plantage : on garde la pose du robot
            return scan, None, f"⚠ Localisation impossible ({exc}) : posé à la pose du robot", []
        if st.ok:
            title = "✓ Localisé sur la carte : Garder ajoute la partie nouvelle (vert)"
        else:
            title = "⚠ Peu de points communs : rapproche-toi d'une zone déjà scannée, puis Refaire"
        p = st.pose
        rows = [("Points communs", f"{st.overlap:.0%}"), ("Partie nouvelle", f"{st.new_share:.0%}"),
                ("Pose", f"{p.x:+.2f}  {p.y:+.2f}  {p.theta_deg:+.0f}°"), ("Méthode", st.how.split(" (")[0]),
                ("Calcul", f"{st.time_s * 1000:.0f} ms")]
        return scan.with_pose(p), st, title, rows

    def keep(self) -> None:
        if self.preview is None:
            return
        scan, info, raw, fmt, step = self.preview
        caps = self.project.visible
        self.drop(quiet=True)
        new, nums = P.add_scans(self.project, [("", scan)], source=fmt, raw=raw)
        self.app.sel, self.app.primary = {nums[0]}, nums[0]
        msg = (f"Scan {nums[0]} gardé : la carte a maintenant {len(caps) + 1} scan(s). "
               "Déplace un peu le robot, puis scanne à nouveau.")
        self.app.apply(new, msg)
        loops = step.loops([c.scan.pose for c in caps]) if step is not None else []
        self._announced |= {caps[k].number for k in loops}
        if self._announced:                 # un seul message, gardé jusqu'à la prochaine optimisation
            seen = sorted(self._announced)
            names = ", ".join(map(str, seen[:4])) + ("…" if len(seen) > 4 else "")
            self.loop_info.configure(text=f"↻ Boucle refermée : le robot retrouve des scans anciens ({names}). "
                                          "« Optimiser toute la carte » corrige l'erreur accumulée (ou à la fin).")
        self.set_robot(scan.pose)
        self._start_follow()

    def redo(self) -> None:
        self.drop(quiet=True)
        self.capture()

    def drop(self, quiet: bool = False) -> None:
        if self.preview is not None:
            self.preview = None
            self.preview_box.pack_forget()
            if not quiet:
                self.app.status("Scan jeté.")
            self.render()

    def on_key(self, key: str) -> None:
        k = key.lower()
        if self.source.get() == "Wi-Fi" and self.pad.key(key):
            return
        if k == "c":
            self.capture()
        elif k == "return":
            self.keep()
        elif k == "escape":
            self.drop()

    def reset(self) -> None:
        self.drop(quiet=True)
        self.collection = None
        self.robot, self.trail, self._map_key, self._map = Pose(), (), None, None
        self._announced = set()
        self.app.live.stop()

    # ================================================================== référence en construction, suivi
    def current_map(self) -> SlamMap | None:
        """La référence = tous les scans gardés et visibles. En cache tant que scans, poses et filtres ne changent pas."""
        caps = self.project.visible
        if not caps:
            return None
        key = (tuple((c.number, id(c.scan.ranges_m), c.scan.pose) for c in caps), self.project.wall_edits,
               self.app.filters())
        if key != self._map_key:
            self._map = SlamMap([self.app.filtered(c) for c in caps], [c.title for c in caps], self.project.wall_edits)
            self._map_key = key
        return self._map

    def _start_follow(self) -> None:
        """(Re)lance le suivi en direct sur la référence actuelle, depuis la pose du robot."""
        if not (self.follow.get() and self.app.source is not None and self.app.step is self):
            self.app.live.stop()
            return
        slam = self.current_map()
        if slam is None:
            return
        self.app.live.start(slam.grid, slam.segments, self.robot, slam.field, slam.pyramid)
        self.trail = (self.robot,)

    def _follow_changed(self) -> None:
        self._start_follow()
        if not self.follow.get():
            self.trail = ()
            self.track_info.configure(text="")
        self.render()

    def on_track(self, result) -> None:
        st = result.step
        if st.mode in ("suivi", "global"):
            self.robot = st.pose
            self.pose_field.set(st.pose.x, st.pose.y, st.pose.theta_deg)
        self.trail = result.trail
        text = {"suivi": "● Robot suivi sur la carte", "global": "● Robot retrouvé sur la carte",
                "incertain": "● Suivi incertain (tour flou) : pose prévue gardée",
                "perdu": "● Robot perdu : rapproche-le d'une zone déjà scannée"}[st.mode]
        self.track_info.configure(text=f"{text} · {st.inliers:.0%} des points sur les murs",
                                  text_color=S.SUCCESS if st.mode in ("suivi", "global") else S.WARN)
        self.render()

    def optimize(self) -> None:
        if len(self.project.visible) < 2:
            self.app.status("Il faut au moins deux scans pour optimiser la carte.")
            return
        self.app.steps[2].align_full()
        self.loop_info.configure(text="")
        self._announced = set()
        if self.app.live.running:
            self._start_follow()

    def _refresh_status(self) -> None:
        n = len(self.project.visible)
        self.map_status.configure(text="Carte vide" if n == 0 else f"Carte : {n} scan{'s' if n > 1 else ''}")
        self.next_hint.configure(text="Pose le robot n'importe où (un coin, par exemple), puis Scanner." if n == 0 else
                                 "Déplace un peu le robot (≤ 1 m, ≤ 45°), puis Scanner : le scan se pose sur la carte.")
        if self.collection is None:
            self.capture_btn.configure(text="Scanner  ·  1er scan = référence" if n == 0 else f"Scanner  ·  scan {n + 1}")

    def on_project(self) -> None:
        self._refresh_status()

    def enter(self) -> None:
        self._refresh_status()
        self._start_follow()

    # ================================================================== carte
    def on_live(self, frame) -> None:
        f = self.app.last_frame
        if f is None:
            return
        fmt = "intensité ✓" if f.fmt != FORMAT_TELEPLOT else "sans intensité (Teleplot)"
        self.live_info.show([("Points du tour", str(int((f.scan.ranges_m > 0).sum()))), ("Format", fmt)])
        if self.follow.get() and self.project.visible and self.collection is None and self.preview is None:
            if not self.app.live.running:
                self._start_follow()
            pts = self.app.filters().apply(f.scan).local_points()
            if len(pts) >= MIN_POINTS:
                self.app.live.submit(pts)
        self.render()

    def _preview_layers(self) -> list:
        """Aperçu du scan : points communs avec la carte (orange) et partie nouvelle (vert, plus gros)."""
        scan, step = self.preview[0], self.preview[4]
        g = local_to_global(self.app.filters().apply(scan).local_points(), scan.pose)
        if step is None or len(step.common) != len(g):
            return [(g, S.INITIAL, 10, "o")]
        return [(g[step.common], S.INITIAL, 9, "o"), (g[~step.common], S.FOUND, 16, "o")]

    def render(self) -> None:
        app = self.app
        pts, poses = app.scan_layers(big=False)
        if self.preview is not None:
            pts = pts + self._preview_layers()
            poses = list(poses) + [(self.preview[0].pose, S.INITIAL, True)]
        app.map.draw_overlay(pts, [(trail_segments(self.trail), S.ACCENT, 2.0)], poses)
        app.map.draw_analysis(None, None, [], [], None)
        app.map.draw_robot(self.robot)
        frame = app.last_frame
        live = None
        if frame is not None and self.preview is None:
            live = app.filters().apply(frame.scan).with_pose(self.robot)
        app.map.draw_live(live, False, [])
        app.map.draw_capture(0, None)
        self.legend("couleurs = scans gardés · noir = tour en direct · aperçu : orange = commun, vert = nouveau · "
                    "trait bleu = trajet")

    def leave(self) -> None:
        self.app.live.stop()
        self.app.map.draw_live(None, False, [])
        self.app.map.draw_capture(0, None)
        self.app.map.robot_dot.set_data([], [])
        self.app.map.robot_line.set_data([], [])
