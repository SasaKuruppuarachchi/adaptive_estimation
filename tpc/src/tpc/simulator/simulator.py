from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym

from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import Agent, Simulator

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
        self, agent: Agent, states: PendulumState,
        observations: PendulumObservations) -> SimulatorHandle:
        """
        Create a handle for the agent to interact with the simulator.
        """
        pass


class PendulumSimulator(Simulator):

    def __init__(self,
        state_dim: int,
        action_dim: int,
        dt: float,
        seed: int,
        gymnasium_env: str = 'CartPole-v1'
    ):
        # self.state: np.ndarray = np.zeros(state_dim)
        # self.action: np.ndarray = np.zeros(action_dim)
        self.actions:  Dict[str, np.ndarray] = {}
        self.handles: Dict[str, SimulatorHandle] = {}

        self.rng = np.random.default_rng(seed)

        self.env: gym.Env  = gym.make(gymnasium_env)
        self.state, self.info = self.env.reset()

    def start(self):
        pass

    def stop(self):
        pass

    def step(self):

        self.observation: np.ndarray = self.rng.normal( 0, 1, self.state.shape)
        self.state: np.ndarray = self.rng.normal( 0, 1, self.state.shape)

        for agent in self.agents:

            # Send updated state and observation to the agent
            agent.observation = self.observation
            agent.state = self.state

            # Update agent state and get action
            agent.step()
            self.actions[agent.name] = agent.get_action()

            # Send action to the simulator environment

    def reset(self):
        pass

    def run(self):
        pass

    def set_handle(
        self, agent: Agent, 
        states: PendulumState,
        observations: PendulumObservations) -> SimulatorHandle:

        self.handles[agent.name] = SimulatorHandle(self)

        return self.handles[agent.name]

