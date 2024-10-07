from abc import ABC, abstractmethod
from typing import Tuple, List, Dict

import numpy as np

from tpc.agent import Agent
# from tpc.utils.utils import PendulumState, PendulumObservations

from tpc.utils.types import SimulatorType, SimulatorHandleType

class PendulumAgent(Agent):
    

    def __init__(self,
        state_dim: Tuple[int],
        action_dim: Tuple[int],
        observation_dim: Tuple[int],
        name: str,
    ):
        self.state: np.ndarray = np.zeros(state_dim)
        self.observation: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)
        self.simulator_handle: SimulatorHandleType = None

    def attach(self, simulator):
        self.simulator_handle = simulator.set_handle(
            self, 
            # states=[PendulumState.x_0, PendulumState.x_1],
            # observations=[PendulumObservations.y_0, PendulumObservations.y_1]
        )

    def detach(self, simulator):
        raise NotImplementedError("Not implemented on simulator")
        simulator.remove_handle(self)

    def compute_action(self):

        self.action = np.random.normal(0, 1, self.action.shape)
        return True


    def compute_state(self):
        self.state = np.random.normal(0, 1, self.state.shape)
        return True

    def get_action(self):
        return self.action

    def step(self):

        # Compute state
        success_state: bool = self.simulator_handle.update_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action

