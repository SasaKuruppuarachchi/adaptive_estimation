from typing import Dict, List, Tuple, Any
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType

class Simulator(ABC):

    def __init__(self):

        self.agents: Dict[str, Agent] = {}
        self.state_shape: np.ndarray
        self.action_shape: np.ndarray
        pass

    @abstractmethod
    def start(self) -> Any:
        pass
    #
    # @abstractmethod
    # def stop(self):
    #     pass

    @abstractmethod
    def step(self):
        pass

    # @abstractmethod
    # def reset(self):
    #     pass
    # 
    # @abstractmethod
    # def run(self):
    #     pass
