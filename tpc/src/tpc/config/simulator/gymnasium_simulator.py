from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from tpc.config.simulator.base import Simulator

@dataclass
class GymnasiumSimulator(Simulator):

    name: str
    env_name: str
    # dt: Optional[float]
    seed: int
    observation_noise_std: float

