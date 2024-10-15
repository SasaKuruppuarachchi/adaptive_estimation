from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union, TypedDict, Any

from tpc.config.agent.base import Agent, AgentClassArgs

from tpc.utils.types import ControlTypes, AgentType

@dataclass
class PendulumAgentConfig(Agent):

    type: AgentType
    args: Dict[str, Any]

@dataclass
class KFPendulumAgentConfig(Agent):

    name: str
    type: AgentType
    control_type: ControlTypes
    controller_args: Dict[str, Union[float, int]]

@dataclass
class tPCPendulumAgentArgs(AgentClassArgs):
    name: str
    inference_duration: int
    control_type: ControlTypes
    controller_args: Dict[str, Union[float, int]]
    dt: float

@dataclass
class tPCPendulumAgentConfig(Agent):

    type: AgentType
    args: tPCPendulumAgentArgs
    # name: str
    # inference_duration: int
    # control_type: ControlTypes
    # controller_args: Dict[str, Union[float, int]]
    # dt: float
