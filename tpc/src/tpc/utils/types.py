from typing import List, Tuple, Dict, Any, Union, TYPE_CHECKING
from enum import Enum, auto

if TYPE_CHECKING:
    from tpc.simulator import Simulator
    from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
    from tpc.agent import Agent
    from tpc.agent.pendulum_agent import tPCPendulumAgent, KalmanFilterPendulumAgent
    from simple_pid.PID import PID

    SimulatorType = Union[Simulator, GymnasiumSimulator]
    AgentType = Union[Agent, tPCPendulumAgent, KalmanFilterPendulumAgent]
    ControlType = Union[PID, Any]

else:
    SimulatorType = Any
    AgentType = Any
    ControlType = Any

class ControlTypes(Enum):
    """
    #TODO is this naming convention consistent with the rest of the codebase?
    """
    PID = auto()
    LQR = auto()
    RANDOM = auto()
    # MPC = auto()
    # AIF = auto()
    NONE = auto()

class CommunicationTypes(Enum):
    LOCAL = auto()
    ROS = auto()
