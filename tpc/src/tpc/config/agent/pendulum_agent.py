from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from tpc.config.agent.base import Agent

@dataclass
class PendulumAgent(Agent):

    name: str = "PendulumAgent"

@dataclass
class KFPendulumAgent(Agent):

    name: str = "PendulumAgent"

@dataclass
class tPCPendulumAgent(Agent):

    name: str = "PendulumAgent"
    inference_duration: int = 500
