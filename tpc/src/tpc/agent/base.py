from abc import ABC, abstractmethod
from typing import Tuple, List, Dict

import numpy as np

from tpc.utils.types import SimulatorType, ControlTypes
from tpc.communication.base import ServerCommunicationHandler as Server

class Agent(ABC):

    @abstractmethod
    def __init__(self,
                 name: str, 
                 control_type: ControlTypes,
                 ):
        self.name: str = name
        self.control_type: ControlTypes = control_type
        self.server: Server


    def init_communication_handler(self, server: Server):
        self.server: Server = server

    @abstractmethod
    def attach(self, simulator):
        """
        Attach the agent to a simulator.
        Access all configured readable states from the simulator.
        """
        pass

    @abstractmethod
    def step(self, observation: np.ndarray) -> bool:
        pass

    @abstractmethod
    def get_action(self) -> np.ndarray:
        pass

    def is_controlling(self):
        return True if self.control_type is not ControlTypes.NONE else False
