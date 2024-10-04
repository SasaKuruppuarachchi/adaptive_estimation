from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from agent import PendulumAgent
from simulator import PendulumSimulator

@dataclass
class Config:
    name: str
    simulator: PendulumSimulator
    agent: PendulumAgent
