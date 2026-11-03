"""Les 6 étapes de LidarLab, dans l'ordre du TP."""

from .align import AlignStep
from .clean import CleanStep
from .collect import CollectStep
from .export import ExportStep
from .locate import LocateStep
from .maps import MapsStep

STEPS = [CollectStep, CleanStep, AlignStep, MapsStep, LocateStep, ExportStep]
