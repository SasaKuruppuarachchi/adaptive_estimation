from typing import List, Tuple, Dict, Any, Union, TYPE_CHECKING

if TYPE_CHECKING:
    from tpc.simulator import Simulator, PendulumSimulator, SimulatorHandle
    from tpc.agent import Agent, PendulumAgent

    Simulator = Union[Simulator, PendulumSimulator]
    SimulatorHandle = SimulatorHandle
    Agent = Union[Agent, PendulumAgent]

else:
    Simulator = Any
    SimulatorHandle = Any
    Agent = Any
