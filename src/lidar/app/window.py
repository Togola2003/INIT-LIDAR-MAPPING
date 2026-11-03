"""Fenêtre principale de LidarLab : en-tête, étapes à gauche, carte au centre, réglages à droite.

    .venv/Scripts/python -m lidar.app [fichiers…]        (ou double-clic sur LidarLab.bat)

La fenêtre tient l'état partagé : le projet (Store), la sélection, la connexion au robot, et des
caches (scans filtrés, cartes) pour que tout reste rapide. Chaque étape (steps/) ne fait que
construire son panneau et dessiner sa carte.
"""

from __future__ import annotations

import collections
import queue
import sys
from dataclasses import replace
from tkinter import filedialog, messagebox

import customtkinter as ctk
import numpy as np

from ..mapedit import apply_to_grid, apply_to_segments
from ..scope.mapping import MapParams, build_maps
from ..scope.mapview import Editor, MapView
from ..scope.processing import Filters
from ..scope.sources import DemoSource, MapDemoSource, SerialSource, list_ports
from . import project as P
from . import style as S
from .components import ScanList
from .steps import STEPS
from .tools import RectTool, WallTool

TICK_MS = 80
# outils toujours visibles au-dessus de la carte : gomme à points + retouches des murs
TOOLS = {"gomme": "⌫  Gomme", "tracer": "✎  Tracer un mur", "prolonger": "↔  Prolonger", "supprimer": "✕  Effacer un mur"}
WALL_MODES = {"tracer", "prolonger", "supprimer"}
MAPS_STEP = 3


class LidarLab:
    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.store = P.Store()
        self.sel: set[int] = set()
        self.primary: int | None = None
        self.originals: dict[int, object] = {}          # pose de chaque scan à son arrivée (bouton « revenir »)
        self.events: queue.Queue = queue.Queue()
        self.source = None
        self.recent: collections.deque = collections.deque(maxlen=30)
        self._filtered: dict = {}
        self._maps: dict = {}
        self.tool = None
        self.tool_mode: str | None = None
        self.step_index = 0
        self._render_job = None

        root.title("LidarLab")
        root.geometry("1620x960")
        root.minsize(1300, 800)
        root.configure(fg_color=S.BG)
        self._build_header()
        body = ctk.CTkFrame(root, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)
        self._build_sidebar(body)
        self._build_map(body)
        self._build_inspector(body)
        self.store.subscribe(self._on_project)
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self._tick_job = root.after(TICK_MS, self._tick)
        self.go(0)
        self._on_project(self.project)

    # ================================================================== construction
    def _build_header(self) -> None:
        bar = ctk.CTkFrame(self.root, fg_color="transparent", height=64)
        bar.pack(fill="x", padx=20, pady=(14, 10))
        logo = ctk.CTkFrame(bar, fg_color=S.ACCENT, width=36, height=36, corner_radius=10)
        logo.pack(side="left")
        logo.pack_propagate(False)
        ctk.CTkLabel(logo, text="◎", font=S.font(20, "bold"), text_color="#FFFFFF").pack(expand=True)
        box = ctk.CTkFrame(bar, fg_color="transparent")
        box.pack(side="left", padx=12)
        ctk.CTkLabel(box, text="LidarLab", font=S.font(20, "bold", display=True), text_color=S.INK).pack(anchor="w")
        self.project_label = ctk.CTkLabel(box, text="", font=S.font(12), text_color=S.MUTED)
        self.project_label.pack(anchor="w")
        S.button(bar, "Enregistrer", self.save_project, "primary", width=120).pack(side="right")
        S.button(bar, "Ouvrir…", self.open_files, width=100).pack(side="right", padx=8)
        S.button(bar, "Nouveau projet", self.new_project, width=130).pack(side="right")
        self.conn_pill = ctk.CTkLabel(bar, text="  ●  Robot non connecté  ", font=S.font(12), text_color=S.MUTED,
                                      fg_color=S.SURFACE, corner_radius=12, height=30)
        self.conn_pill.pack(side="right", padx=16)

    def _build_sidebar(self, parent) -> None:
        side = ctk.CTkFrame(parent, fg_color=S.SURFACE, corner_radius=S.RADIUS, border_width=1,
                            border_color=S.BORDER, width=250)
        side.grid(row=0, column=0, sticky="ns", padx=(0, 14))
        side.grid_propagate(False)
        side.pack_propagate(False)
        ctk.CTkLabel(side, text="Étapes", font=S.font(12, "bold"), text_color=S.MUTED).pack(anchor="w", padx=18, pady=(16, 6))
        self.nav = []
        for k, cls in enumerate(STEPS):
            item = ctk.CTkFrame(side, fg_color="transparent", corner_radius=10, height=50)
            item.pack(fill="x", padx=10, pady=1)
            badge = ctk.CTkLabel(item, text=str(k + 1), width=26, height=26, corner_radius=13, fg_color=S.SURFACE_2,
                                 font=S.font(12, "bold"), text_color=S.MUTED)
            badge.pack(side="left", padx=(10, 10), pady=10)
            txt = ctk.CTkFrame(item, fg_color="transparent")
            txt.pack(side="left", fill="x")
            t1 = ctk.CTkLabel(txt, text=cls.title, font=S.font(13, "bold"), text_color=S.INK, anchor="w")
            t1.pack(anchor="w")
            t2 = ctk.CTkLabel(txt, text=cls.subtitle, font=S.font(11), text_color=S.MUTED, anchor="w")
            t2.pack(anchor="w")
            for w in (item, badge, txt, t1, t2):
                w.bind("<Button-1>", lambda e, k=k: self.go(k))
            self.nav.append((item, badge))
        ctk.CTkFrame(side, fg_color=S.BORDER, height=1).pack(fill="x", padx=16, pady=(12, 8))
        head = ctk.CTkFrame(side, fg_color="transparent")
        head.pack(fill="x", padx=18)
        ctk.CTkLabel(head, text="Scans du projet", font=S.font(12, "bold"), text_color=S.MUTED).pack(side="left")
        S.button(head, "Tout", self.select_all, "ghost", width=46, height=24).pack(side="right")
        self.scan_list = ScanList(side, on_select=self.select, on_toggle=self.toggle_visible)
        self.scan_list.pack(fill="both", expand=True, padx=8, pady=(4, 10))

    def _build_map(self, parent) -> None:
        card = ctk.CTkFrame(parent, fg_color=S.SURFACE, corner_radius=S.RADIUS, border_width=1, border_color=S.BORDER)
        card.grid(row=0, column=1, sticky="nsew", padx=(0, 14))
        top = ctk.CTkFrame(card, fg_color="transparent")
        top.pack(fill="x", padx=18, pady=(14, 6))
        left = ctk.CTkFrame(top, fg_color="transparent")
        left.pack(side="left")
        self.view_title = ctk.CTkLabel(left, text="", font=S.font(22, "bold", display=True), text_color=S.INK)
        self.view_title.pack(anchor="w")
        self.view_subtitle = ctk.CTkLabel(left, text="", font=S.font(12), text_color=S.MUTED)
        self.view_subtitle.pack(anchor="w")
        bar = ctk.CTkFrame(card, fg_color="transparent")
        bar.pack(fill="x", padx=18, pady=(2, 6))
        strip = ctk.CTkFrame(bar, fg_color=S.SURFACE_2, corner_radius=12)
        strip.pack(side="left")
        self.tool_buttons = {}
        for key, text in TOOLS.items():
            b = S.button(strip, text, lambda k=key: self.toggle_tool(k), width=10, height=32, fg_color="transparent",
                         hover_color=S.BORDER, font=S.font(12))
            b.pack(side="left", padx=3, pady=3)
            self.tool_buttons[key] = b
        for text, cmd in (("↷", self.redo), ("↶", self.undo)):
            S.button(bar, text, cmd, width=38, font=S.font(15)).pack(side="right", padx=(6, 0))
        self.ruler_btn = S.button(bar, "Règle", self.toggle_ruler, width=70)
        self.ruler_btn.pack(side="right", padx=(6, 0))
        S.button(bar, "Tout voir", self.fit_view, width=86).pack(side="right")
        holder = ctk.CTkFrame(card, fg_color=S.MAP_BG, corner_radius=10)
        holder.pack(fill="both", expand=True, padx=14)
        self.cursor_var = ctk.StringVar(value="")
        self.map = MapView(holder, on_cursor=self.cursor_var.set, theme=S)
        self.map.fig.set_facecolor(S.MAP_BG)
        self.map.ax.set_position([0.05, 0.06, 0.93, 0.92])
        self.map.widget.configure(bg=S.MAP_BG)
        self.map.widget.pack(fill="both", expand=True, padx=4, pady=4)
        foot = ctk.CTkFrame(card, fg_color="transparent")
        foot.pack(fill="x", padx=18, pady=(4, 12))
        self.legend = ctk.CTkLabel(foot, text="", font=S.font(11), text_color=S.MUTED, anchor="w")
        self.legend.pack(side="left")
        ctk.CTkLabel(foot, textvariable=self.cursor_var, font=S.font(11, mono=True), text_color=S.FAINT).pack(side="right")
        self.status_var = ctk.StringVar(value="Bienvenue. Ouvre un projet, importe des scans ou connecte le robot.")
        ctk.CTkLabel(card, textvariable=self.status_var, font=S.font(12), text_color=S.INK, anchor="w",
                     fg_color=S.SURFACE_2, corner_radius=8, height=30).pack(fill="x", padx=14, pady=(0, 14))

    def _build_inspector(self, parent) -> None:
        self.inspector = ctk.CTkFrame(parent, fg_color="transparent", width=400)
        self.inspector.grid(row=0, column=2, sticky="ns")
        self.inspector.grid_propagate(False)
        self.inspector.pack_propagate(False)
        self.steps, self.panels = [], []
        for cls in STEPS:
            panel = ctk.CTkScrollableFrame(self.inspector, fg_color="transparent", scrollbar_button_color=S.BORDER)
            step = cls(self)
            course = ctk.CTkFrame(panel, fg_color=S.ACCENT_SOFT, corner_radius=S.RADIUS)
            course.pack(fill="x", pady=(0, 10))
            ctk.CTkLabel(course, text="Ce que dit le cours", font=S.font(12, "bold"), text_color=S.ACCENT,
                         anchor="w").pack(fill="x", padx=16, pady=(12, 0))
            ctk.CTkLabel(course, text=cls.course, font=S.font(12), text_color=S.INK, anchor="w", justify="left",
                         wraplength=340).pack(fill="x", padx=16, pady=(2, 12))
            step.build(panel)
            self.steps.append(step)
            self.panels.append(panel)

    def _bind_keys(self) -> None:
        r = self.root
        r.bind("<Control-z>", lambda e: self.undo())
        r.bind("<Control-y>", lambda e: self.redo())
        r.bind("<Control-s>", lambda e: self.save_project())
        r.bind("<Control-o>", lambda e: self.open_files())
        r.bind("<Control-n>", lambda e: self.new_project())
        r.bind("<Control-a>", lambda e: None if self.typing(e) else self.select_all())
        r.bind("<Key>", self._on_key)
        r.bind("<Escape>", lambda e: self.toggle_tool(None), add="+")

    @staticmethod
    def typing(event) -> bool:
        return "entry" in str(event.widget).lower() or "text" in str(event.widget).lower()

    def _on_key(self, event) -> None:
        if not self.typing(event):
            self.step.on_key(event.keysym)

    # ================================================================== navigation
    @property
    def step(self):
        return self.steps[self.step_index]

    @property
    def project(self) -> P.Project:
        return self.store.project

    def go(self, k: int) -> None:
        if hasattr(self, "steps") and self.steps:
            self.step.leave()
        self.step_index = max(0, min(len(self.steps) - 1, k))
        for i, (item, badge) in enumerate(self.nav):
            on = i == self.step_index
            item.configure(fg_color=S.ACCENT_SOFT if on else "transparent")
            badge.configure(fg_color=S.ACCENT if on else S.SURFACE_2, text_color="#FFFFFF" if on else S.MUTED)
        for i, panel in enumerate(self.panels):
            if i == self.step_index:
                panel.pack(fill="both", expand=True)
            else:
                panel.pack_forget()
        self._tool_off()
        st = self.step
        self.view_title.configure(text=st.title)
        self.view_subtitle.configure(text=st.headline)
        self.set_tool(None)
        self.map.set_editor(None)
        st.enter()
        self.render()

    def render(self, delay_ms: int = 0) -> None:
        if self._render_job:
            self.root.after_cancel(self._render_job)
        self._render_job = self.root.after(delay_ms, self._render_now)

    def _render_now(self) -> None:
        self._render_job = None
        self.step.render()

    # ------------------------------------------------------------------ outils de la barre
    def toggle_tool(self, mode: str | None) -> None:
        """Gomme ou retouche des murs. Re-cliquer (ou Échap) désactive l'outil."""
        if mode is None or mode == self.tool_mode:
            if self.tool_mode is not None:
                self._tool_off()
                self.step.enter()
            return
        if mode in WALL_MODES and self.step_index != MAPS_STEP:
            self.go(MAPS_STEP)                    # les murs se voient et se retouchent sur l'étape Cartes
        if mode == "gomme":
            self.set_tool(RectTool(self.map, self.erase_box))
            self.status("Gomme : glisse un rectangle sur les points à effacer (tous les scans visibles). Échap = arrêter.")
        else:
            self.set_tool(WallTool(self.map, mode, self.final_segments, self.add_edits))
            self.status({"tracer": "Tracer : glisse d'un point à un autre pour ajouter un mur (extrémités aimantées).",
                         "prolonger": "Prolonger : attrape l'extrémité d'un mur et déplace-la.",
                         "supprimer": "Effacer : clique sur un mur pour le supprimer."}[mode] + " Échap = arrêter.")
        self.tool_mode = mode
        for k, b in self.tool_buttons.items():
            on = k == mode
            b.configure(fg_color=S.ACCENT if on else "transparent", text_color="#FFFFFF" if on else S.INK,
                        hover_color=S.ACCENT_HOVER if on else S.BORDER)

    def _tool_off(self) -> None:
        self.tool_mode = None
        self.set_tool(None)
        for b in self.tool_buttons.values():
            b.configure(fg_color="transparent", text_color=S.INK, hover_color=S.BORDER)

    def erase_box(self, box) -> None:
        nums = {c.number for c in self.project.visible}
        new, n = P.erase_points(self.project, nums, box)
        if n:
            self.apply(new, f"{n} point(s) effacé(s) : la grid map, les murs et la localisation en tiennent compte. "
                            "Ctrl+Z pour annuler.")
        else:
            self.status("Aucun point dans ce rectangle.")

    def add_edits(self, edits) -> None:
        self.apply(P.add_edits(self.project, edits),
                   f"Retouche enregistrée ({len(self.project.wall_edits) + len(edits)} au total) : elle compte pour la "
                   "grid map, les murs et la localisation. Ctrl+Z pour annuler.")

    def set_tool(self, tool) -> None:
        if self.tool is not None:
            self.tool.close()
        self.tool = tool
        self.map.tool = tool

    def status(self, text: str) -> None:
        self.status_var.set(text)

    # ================================================================== projet
    def _on_project(self, proj: P.Project) -> None:
        for c in proj.captures:
            self.originals.setdefault(c.number, c.scan.pose)
        alive = {c.number for c in proj.captures}
        self.sel &= alive
        if self.primary not in alive:
            self.primary = min(self.sel) if self.sel else None
        dirty = " · modifié" if self.store.dirty else ""
        self.project_label.configure(text=f"{proj.name} · {len(proj.captures)} scan(s){dirty}")
        self.root.title(f"LidarLab — {proj.name}{' •' if self.store.dirty else ''}")
        self.scan_list.show(proj.captures, self.sel, self.primary)
        self.step.on_project()
        self.render(30)

    def apply(self, new: P.Project, message: str = "") -> None:
        self.store.apply(new)
        if message:
            self.status(message)

    def new_project(self) -> None:
        if self.store.dirty and not messagebox.askyesno(
                "Nouveau projet", "Le projet actuel a des modifications non enregistrées. Tout vider quand même ?"):
            return
        self.originals.clear()
        self.sel, self.primary = set(), None
        self._filtered.clear()
        self._maps.clear()
        self.store.reset()
        for st in self.steps:
            st.reset()
        self.status("Nouveau projet vide : importe des scans ou connecte le robot (étape 1).")
        self.map.fit(np.array([[-3.0, -3.0], [3.0, 3.0]]))

    def open_files(self, paths=None) -> None:
        paths = paths or filedialog.askopenfilenames(title="Ouvrir des scans ou un projet", initialdir="data",
                                                     filetypes=[("Scans lidar", "*.json *.csv"), ("Tous", "*.*")])
        if not paths:
            return
        new, added, errors = P.open_files(self.project, list(paths))
        if errors:
            messagebox.showerror("Ouvrir", "\n".join(errors))
        if added:
            if self.project.is_empty and len(paths) == 1 and str(paths[0]).endswith(".json"):
                new = P.replace_name(new, str(paths[0]))
            self.sel, self.primary = {added[0]}, added[0]
            self.apply(new, f"{len(added)} scan(s) ajouté(s).")
            self.fit_view()

    def save_project(self) -> None:
        if self.project.is_empty:
            self.status("Rien à enregistrer : le projet est vide.")
            return
        path = self.project.path or filedialog.asksaveasfilename(
            title="Enregistrer le projet", defaultextension=".json", initialdir="data",
            initialfile=f"{self.project.name.replace(' ', '_')}.lidar.json", filetypes=[("Projet LidarLab", "*.json")])
        if path:
            self.store.apply(P.save(self.project, path, {"map_params": self.map_params().as_dict()}), remember=False)
            self.store.mark_saved()
            self._on_project(self.project)
            self.status(f"Projet enregistré : {path}")

    def undo(self) -> None:
        self.status("Annulé." if self.store.undo() else "Rien à annuler.")

    def redo(self) -> None:
        self.status("Rétabli." if self.store.redo() else "Rien à rétablir.")

    # ================================================================== sélection
    def select(self, number: int, additive: bool = False) -> None:
        if additive and number in self.sel and len(self.sel) > 1:
            self.sel = self.sel - {number}
            self.primary = self.primary if self.primary in self.sel else min(self.sel)
        else:
            self.sel = (self.sel | {number}) if additive else {number}
            self.primary = number
        self.scan_list.show(self.project.captures, self.sel, self.primary)
        self.step.on_selection()
        self.render()

    def select_all(self) -> None:
        vis = [c.number for c in self.project.visible]
        if vis:
            self.sel = set(vis)
            self.primary = self.primary if self.primary in self.sel else vis[0]
            self.scan_list.show(self.project.captures, self.sel, self.primary)
            self.step.on_selection()
            self.render()

    def toggle_visible(self, number: int) -> None:
        cap = self.project.find(number)
        if cap:
            self.apply(P.set_visible(self.project, {number}, not cap.visible))

    def editor(self) -> Editor:
        """Branchement de la carte pour déplacer / tourner les scans à la souris."""
        return Editor(pick=self.pick, selection=lambda: self.sel, select=self.select,
                      pivot=lambda keys: P.pivot_of(self.project, keys),
                      commit=lambda keys, dx, dy, dth, piv: self.apply(P.move(self.project, keys, dx, dy, dth, piv)))

    def pick(self, x: float, y: float, tol: float) -> int | None:
        best, best_d, click = None, tol, np.array([x, y])
        for c in self.project.visible:
            pts = np.vstack([self.points(c), [[c.scan.pose.x, c.scan.pose.y]]])
            d = float(np.min(np.hypot(*(pts - click).T)))
            if d < best_d:
                best, best_d = c.number, d
        return best

    # ================================================================== calculs partagés (avec caches)
    def filters(self) -> Filters:
        return self.steps[1].filters()

    def map_params(self) -> MapParams:
        return self.steps[3].params()

    def filtered(self, cap):
        """Scan filtré (repère lidar), mis en cache : le filtre ne dépend pas de la pose."""
        f = self.filters()
        key = (cap.number, id(cap.scan.ranges_m), f)
        if key not in self._filtered:
            if len(self._filtered) > 400:
                self._filtered.clear()
            self._filtered[key] = f.apply(cap.scan)
        return self._filtered[key].with_pose(cap.scan.pose)

    def points(self, cap) -> np.ndarray:
        return self.filtered(cap).global_points()

    def final_maps(self, params: MapParams | None = None):
        """Cartes des scans visibles AVEC les retouches à la main (murs ajoutés / supprimés) : ce que
        l'utilisateur voit, exporte et valide comme référence."""
        res = self.maps(params=params)
        edits = self.project.wall_edits
        if not edits:
            return res
        grid = apply_to_grid(res.grid, edits) if res.grid is not None else None
        segs = apply_to_segments(res.segments, edits) if (params or self.map_params()).vector.enabled else res.segments
        return replace(res, grid=grid, segments=segs)

    def final_segments(self) -> list:
        if not self.project.visible:
            return []
        p = self.map_params()
        return list(self.final_maps(replace(p, grid=replace(p.grid, enabled=False))).segments)

    def maps(self, caps=None, params: MapParams | None = None):
        caps = self.project.visible if caps is None else caps
        params = params or self.map_params()
        key = (tuple((c.number, id(c.scan.ranges_m), c.scan.pose) for c in caps), params)
        if key not in self._maps:
            if len(self._maps) > 12:
                self._maps.clear()
            self._maps[key] = build_maps([c.scan for c in caps], params)
        return self._maps[key]

    def scan_layers(self, caps=None, big: bool = True, faint: str | None = None) -> tuple[list, list]:
        """Couches « points » et « poses » de la carte pour des captures (déplaçables à la souris)."""
        caps = self.project.visible if caps is None else caps
        pts = [(self.points(c), faint or c.color, (9 if c.number in self.sel and big else 4), "o", c.number)
               for c in caps]
        poses = [(c.scan.pose, c.color, c.number in self.sel, c.number) for c in caps]
        return pts, poses

    def fit_view(self) -> None:
        caps = self.project.visible
        if caps:
            self.map.fit(np.vstack([self.points(c) for c in caps] + [[[c.scan.pose.x, c.scan.pose.y]] for c in caps]))

    def toggle_ruler(self) -> None:
        on = not self.map.ruler_mode
        self.map.set_ruler(on)
        self.ruler_btn.configure(fg_color=S.WARN if on else S.SURFACE_2, text_color="#FFFFFF" if on else S.INK)

    # ================================================================== robot
    def ports(self) -> list[str]:
        return list_ports()

    def connect(self, port: str, baud: int) -> None:
        self.disconnect()
        self._start(SerialSource(port, baud, self.events), f"Port {port}")

    def demo(self) -> None:
        self.disconnect()
        self._start(DemoSource(self.events), "Démo (robot simulé)")

    def map_demo(self, walls, waypoints) -> None:
        """Robot simulé qui roule dans la carte de référence : pour essayer la localisation en direct."""
        self.disconnect()
        self._start(MapDemoSource(self.events, walls, waypoints), "Démo sur ta carte")

    @property
    def demo_truth(self):
        """Vraie pose du robot de démo (None avec le vrai robot)."""
        return getattr(self.source, "truth", None)

    def _start(self, source, label: str) -> None:
        self.source = source
        self.source_label = label
        source.start()
        self.conn_pill.configure(text=f"  ●  {label}  ", text_color=S.WARN)

    def disconnect(self) -> None:
        if self.source:
            self.source.stop()
            self.source = None
        self.recent.clear()
        self.conn_pill.configure(text="  ●  Robot non connecté  ", text_color=S.MUTED)

    @property
    def last_frame(self):
        return self.recent[-1] if self.recent else None

    def _tick(self) -> None:
        got = None
        while True:
            try:
                ev = self.events.get_nowait()
            except queue.Empty:
                break
            if ev.kind == "scan":
                self.recent.append(ev.payload)
                got = ev.payload
                self.step.on_frame(ev.payload)
            elif ev.kind == "error":
                self.disconnect()
                messagebox.showerror("Connexion", f"{ev.payload}\n\nLe moniteur série Arduino est-il encore ouvert ?")
            elif ev.kind == "status":
                self.status(ev.payload)
        if got is not None and self.source is not None:
            self.conn_pill.configure(text=f"  ●  {self.source_label}  ", text_color=S.SUCCESS)
            self.step.on_live(got)
        self._tick_job = self.root.after(TICK_MS, self._tick)

    def quit(self) -> None:
        if self.store.dirty and not messagebox.askyesno("Quitter", "Des modifications ne sont pas enregistrées. "
                                                        "Quitter quand même ?"):
            return
        self.disconnect()
        for job in (self._tick_job, self._render_job):
            if job:
                self.root.after_cancel(job)
        self.root.quit()
        self.root.destroy()


def main() -> None:
    ctk.set_appearance_mode("light")
    root = ctk.CTk()
    app = LidarLab(root)
    if len(sys.argv) > 1:
        root.after(300, lambda: app.open_files(sys.argv[1:]))
    root.mainloop()
