from abc import ABC, abstractmethod
from typing import Tuple, List, Dict, Union

import numpy as np

from tpc.agent import Agent
# from tpc.utils.utils import PendulumState, PendulumObservations

from tpc.utils.types import SimulatorType

class PendulumAgent(Agent):
    

    def __init__(self,
        state_dim: Union[Tuple[int], np.ndarray],
        action_dim: Union[Tuple[int], np.ndarray],
        observation_dim: Union[Tuple[int], np.ndarray],
        name: str,
    ):
        self.state: np.ndarray = np.zeros(state_dim)
        self.observation: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)

        self.name = name

    def attach(self, simulator):
        simulator.agents[self.name] = self

    def compute_action(self):

        # self.action[:] = (np.random.normal(0, 1, self.action.shape) > 0).astype(int)
        self.action[:] = np.random.normal(0, 1, self.action.shape)
        return True


    def compute_state(self):
        self.state[:] = np.random.normal(0, 1, self.state.shape)
        return True

    def get_action(self):
        return int(self.action.item())

    def step(self):

        # Compute state
        success_state: bool = self.compute_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action

