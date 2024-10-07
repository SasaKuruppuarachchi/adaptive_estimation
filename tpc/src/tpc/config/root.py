from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union

from tpc.config.agent.base import Agent
from tpc.config.simulator.base import Simulator

# from tpc.config.agent.pendulum_agent import PendulumAgent
# from tpc.config.simulator.pendulum_simulator import PendulumSimulator

@dataclass
class Config:
    name: str
    simulator: Simulator
    agent: Agent
    # simulator: Union[PendulumSimulator, Simulator]
    # agent: Union[Agent, PendulumAgent]
