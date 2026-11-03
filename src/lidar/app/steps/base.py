"""Une étape de LidarLab = un panneau de réglages à droite + une façon de dessiner la carte."""

from __future__ import annotations


class Step:
    title = ""          # nom court (barre latérale, titre de la carte)
    subtitle = ""       # quelques mots sous le nom, dans la barre latérale
    headline = ""       # phrase sous le titre de la carte
    course = ""         # « Ce que dit le cours », en tête du panneau

    def __init__(self, app) -> None:
        self.app = app

    # appelés par la fenêtre ------------------------------------------------
    def build(self, panel) -> None: ...

    def enter(self) -> None: ...

    def leave(self) -> None: ...

    def render(self) -> None: ...

    def reset(self) -> None:
        """« Nouveau projet » : oublier ce qui dépend de l'ancien projet."""

    def on_project(self) -> None: ...

    def on_selection(self) -> None: ...

    def on_frame(self, frame) -> None:
        """Chaque tour complet reçu du robot (même si l'étape n'est pas affichée… seulement l'étape active)."""

    def on_live(self, frame) -> None:
        """Dernier tour reçu, au plus une fois par rafraîchissement : pour l'affichage en direct."""

    def on_key(self, key: str) -> None: ...

    # outils ----------------------------------------------------------------
    @property
    def project(self):
        return self.app.project

    def legend(self, text: str) -> None:
        self.app.legend.configure(text=text)

    def clear_map(self) -> None:
        self.app.map.draw_live(None, False, [])
        self.app.map.draw_capture(0, None)
        self.app.map.draw_analysis(None, None, [], [], None)
