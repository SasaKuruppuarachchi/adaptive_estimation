from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType

class SimulatorHandle:
    
        def __init__(self, simulator: SimulatorType):
            self.simulator = simulator
    
        def get_state(self):
            pass
    
        def get_observation(self):
            pass
    
        def get_reward(self):
            pass
    
        def get_done(self):
            pass
    
        def get_info(self):
            pass

        def set_action(self, action):
            pass

class Simulator(ABC):

    def __init__(self):

        self.agents: Dict[str, Agent] = {}
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def reset(self):
        pass
    
    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def set_handle(
        self, agent: AgentType, 
        # states: PendulumState,
        # observations: PendulumObservations
    ) -> SimulatorHandle:
        """
        Create a handle for the agent to interact with the simulator.
        """
        pass
