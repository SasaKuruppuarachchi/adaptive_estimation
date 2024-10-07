from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from tpc.config.simulator.base import Simulator

@dataclass
class PendulumSimulator(Simulator):

    name: str = "PendulumSimulator"
    env_name: str = "CartPole-v1"

    dt: float = 0.01
    seed: int = 666

