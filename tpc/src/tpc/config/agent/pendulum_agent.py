from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union

from tpc.config.agent.base import Agent

from tpc.utils.types import ControlTypes

@dataclass
class PendulumAgent(Agent):

    name: str

@dataclass
class KFPendulumAgent(Agent):

    name: str

@dataclass
class tPCPendulumAgent(Agent):

    inference_duration: int
    control_type: ControlTypes
    controller_args: Dict[str, Union[float, int]]
    name: str
    dt: float
