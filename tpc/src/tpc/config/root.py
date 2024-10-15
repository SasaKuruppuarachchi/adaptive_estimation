from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union

from tpc.config.agent.base import Agent
from tpc.config.simulator.base import Simulator

@dataclass
class Config:
    name: str
    simulator: Simulator
    agents: Dict[str, Agent]
    seed: int = 42
    # simulator: Union[PendulumSimulator, Simulator]
    # agent: Union[Agent, PendulumAgent]

    def __post_init__(self):
        # Ensure agent names are unique
        self.assert_unique_agent_names()

### Functionality that asserts that each agent.name is unique

    def assert_unique_agent_names(self) -> None:
        names = [agent.args.name for agent in self.agents.values()]
        assert len(names) == len(set(names)), "Agent names are not unique"
    

