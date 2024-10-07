from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from tpc.config.agent.base import Agent
from tpc.config.simulator.base import Simulator

@dataclass
class Config:
    name: str
    simulator: Simulator
    agent: Agent
