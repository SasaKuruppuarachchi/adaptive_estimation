from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

@dataclass
class Agent:

    name: str

@dataclass
class PendulumAgent(Agent):

    name: str = "PendulumAgent"
