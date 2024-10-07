from abc import ABC, abstractmethod
from typing import Tuple, List, Dict

import numpy as np

from tpc.utils.types import SimulatorType

class Agent(ABC):

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
