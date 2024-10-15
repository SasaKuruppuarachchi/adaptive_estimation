from typing import List, Tuple, Dict, Any, Union, TYPE_CHECKING
from enum import Enum, auto

if TYPE_CHECKING:
    from tpc.simulator import Simulator, PendulumSimulator
    from tpc.agent import Agent, PendulumAgent

    SimulatorType = Union[Simulator, PendulumSimulator]
    AgentType = Union[Agent, PendulumAgent]

else:
    SimulatorType = Any
    AgentType = Any

class ControlTypes(Enum):
    PID = auto()
    LQR = auto()
    RANDOM = auto()
    # MPC = auto()
    # AIF = auto()
