from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

from tpc.config.agent.base import Agent

@dataclass
class PendulumAgent(Agent):

    name: str = "PendulumAgent"
