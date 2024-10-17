from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, Union

from tpc.config.agent.base import Agent
from tpc.config.simulator.base import Simulator
from tpc.config.visualiser.base import AbstractVisualiser

from tpc.utils.types import ControlTypes

@dataclass
class Config:
    name: str
    simulator: Simulator
    agents: Dict[str, Agent]
    visualiser: Optional[AbstractVisualiser]
    seed: int = 42
    # simulator: Union[PendulumSimulator, Simulator]
    # agent: Union[Agent, PendulumAgent]

    def __post_init__(self):
        # Ensure agent names are unique
        self.assert_unique_agent_names()
        self.assert_only_one_controlling()

### Functionality that asserts that each agent.name is unique

    def assert_unique_agent_names(self) -> None:
        names = [agent.args.name for agent in self.agents.values()]
        assert len(names) == len(set(names)), "Agent names are not unique"

    def assert_only_one_controlling(self) -> None:
        not_none_controls = []
        for agent in self.agents.values():
            if agent.args.control_type is not ControlTypes.NONE:
                not_none_controls.append(agent)

        assert len(not_none_controls) == 1, "Only one agent can control the environment"



    

