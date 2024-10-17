from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union

from tpc.config.visualiser.base import AbstractVisualiser

@dataclass
class GymnasiumPendulumVisualizerConfig(AbstractVisualiser):

    name: str
    plot_theta: bool
