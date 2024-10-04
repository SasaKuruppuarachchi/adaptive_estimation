from abc import ABC, abstractmethod
from typing import Tuple, List, Dict

import numpy as np

from utils import PendulumState, PendulumObservations
from simulator import Simulator, PendulumSimulator, SimulatorHandle

class Agent(ABC):

    @abstractmethod
    def attach(self, simulator):
        """
        Attach the agent to a simulator.
        Access all configured readable states from the simulator.
        """
        pass

    @abstractmethod
    def detach(self, simulator):
        pass

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def run(self):
        pass

class PendulumAgent(Agent):
    

    def __init__(self,
        state_dim: Tuple[int],
        action_dim: Tuple[int],
        observation_dim: Tuple[int],
        name: str
    ):
        self.state: np.ndarray = np.zeros(state_dim)
        self.observation: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)
        self.simulator_handle: SimulatorHandle = None

    def attach(self, simulator):
        self.simulator_handle = simulator.set_listener(
            self, 
            states=[PendulumState.x_0, PendulumState.x_1],
            observations=[PendulumObservations.y_0, PendulumObservations.y_1]
        )

    def detach(self, simulator):
        simulator.remove_listener(self)

    def compute_action(self):
        pass

    def step(self):
        # Read observations from simulator
        success_observation = self.observation = self.simulator.get_observations()

        # Compute action
        success_action = self.action = self.compute_action()

        # Apply action to simulator
        success_set_action = self.simulator_handle.set_action(self.action)

        return success_observation, success_action, success_set_action

