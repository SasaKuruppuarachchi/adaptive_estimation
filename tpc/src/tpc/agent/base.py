from abc import ABC, abstractmethod
from typing import Tuple, List, Dict

import numpy as np

from tpc.utils.types import SimulatorType, ControlTypes

class Agent(ABC):

    def __init__(self):
        self.name: str
        self.control_type: ControlTypes

    @abstractmethod
    def attach(self, simulator):
        """
        Attach the agent to a simulator.
        Access all configured readable states from the simulator.
        """
        pass

    @abstractmethod
    def step(self):
        pass

    def is_controlling(self):
        return True if self.control_type is not ControlTypes.NONE else False
