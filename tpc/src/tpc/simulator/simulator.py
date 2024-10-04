from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

import numpy as np

from utils import PendulumState, PendulumObservations
from agent import Agent, PendulumAgent


class SimulatorHandle:
    
        def __init__(self, simulator: Simulator):
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
    def set_listener(
        self, agent: Agent, 
        states: PendulumState,
            observations: PendulumObservations) -> SimulatorHandle:
        pass


class PendulumSimulator(Simulator):

    def __init__(self,
        state_dim: int,
        action_dim: int,
        dt: float,
    ):
        self.state: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)
        self.handles: Dict[str, SimulatorHandle] = {}

    def start(self):
        pass

    def stop(self):
        pass

    def step(self):
        pass

    def reset(self):
        pass

    def run(self):
        pass

    def set_listener(
        self, agent: Agent, 
        states: PendulumState,
        observations: PendulumObservations) -> SimulatorHandle:

        self.handles[agent.name] = SimulatorHandle(self)

        # Somehow integrate this agent with the states it's requiring
        # Such that when agent calls get_state, get_observation, etc.
        # it gets the correct available state or observation 

        for state in states:
            pass
        for observation in observations:
            pass

        return self.handles[agent.name]

