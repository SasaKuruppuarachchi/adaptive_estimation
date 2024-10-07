from typing import List, Tuple, Dict, Any, Union, TYPE_CHECKING

if TYPE_CHECKING:
    from tpc.simulator import Simulator, PendulumSimulator, SimulatorHandle
    from tpc.agent import Agent, PendulumAgent

    SimulatorType = Union[Simulator, PendulumSimulator]
    SimulatorHandleType = SimulatorHandle
    AgentType = Union[Agent, PendulumAgent]

else:
    SimulatorType = Any
    SimulatorHandleType = Any
    AgentType = Any
