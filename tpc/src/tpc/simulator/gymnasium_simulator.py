from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType

from tpc.simulator import SimulatorHandle, Simulator

class GymnasiumSimulator(Simulator):

    def __init__(self,
        # state_dim: int,
        # action_dim: int,
        dt: float,
        seed: int,
        name: str,
        env_name: str = 'CartPole-v1'
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
        self, agent: AgentType, 
        # states: PendulumState,
        # observations: PendulumObservations
        ) -> SimulatorHandle:

        self.handles[agent.name] = SimulatorHandle(self)

        return self.handles[agent.name]

