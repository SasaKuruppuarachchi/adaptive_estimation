from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union, TypedDict, Any

import numpy as np

from tpc.config.agent.base import Agent, AgentClassArgs

from tpc.utils.types import ControlTypes, AgentType

@dataclass
class KalmanFilterPendulumAgentArgs(AgentClassArgs):
    name: str
    control_type: ControlTypes
    controller_args: Dict[str, Union[float, int]]
    dt: float
    state_noise_std: float
    observation_noise_std: float

@dataclass
class KalmanFilterPendulumAgentConfig(Agent):

    type: AgentType
    args: KalmanFilterPendulumAgentArgs

@dataclass
class tPCPendulumAgentArgs(AgentClassArgs):
    name: str
    inference_duration: int
    control_type: ControlTypes
    controller_args: Dict[str, Union[float, int]]
    dt: Optional[float]

@dataclass
class tPCPendulumAgentConfig(Agent):

    type: AgentType
    args: tPCPendulumAgentArgs
