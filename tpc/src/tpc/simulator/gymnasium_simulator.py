# import logging
# logger = logging.getLogger(__name__)
from loguru import logger
logger.add("logs/main.log", rotation="500 MB")

from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym
from gymnasium.wrappers import ClipAction

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType

from tpc.simulator import Simulator

class GymnasiumSimulator(Simulator):

    def __init__(self,
        # state_dim: int,
        # action_dim: int,
        dt: float,
        seed: int,
        name: str,
        env_name: str = 'CartPole-v1',
        render_mode: str = 'human',

    ):
        self.agents: Dict[str, AgentType] = {}

        self.rng = np.random.default_rng(seed)

        self.env: gym.Env  = gym.make(env_name, render_mode=render_mode)
        # self.env = ClipAction(self.env)

        self.action_shape: np.ndarray = np.array([1])
        if self.env.action_space.shape:
            self.action_shape = self.env.action_space.shape

        self.state_shape: np.ndarray = np.array([1])
        if self.env.observation_space.shape:
            self.state_shape = self.env.observation_space.shape


    def start(self):
        self.state, self.info = self.env.reset()
        self.observation = self.state + self.rng.normal(0, 1, self.state.shape)
        self.env.render()
        logger.info(f"Initial state: {self.state}")

        return self.info

    def stop(self):
        pass

    def step(self):

        # self.observation: np.ndarray = self.rng.normal( 0, 1, self.state.shape)
        # self.state: np.ndarray = self.rng.normal( 0, 1, self.state.shape)

        for agent_name, agent in self.agents.items():

            # Send updated state and observation to the agent
            agent.observation[:] = self.observation
            # agent.state = self.state

            # Update agent state and get action
            agent.step()
            action = agent.get_action()
            # action = self.env.action_space.sample()

            # Send action to the simulator environment
            # self.env.step(action)
            self.state[:], self.reward, terminated, truncated, info = self.env.step(action)
            # self.observation[:] = self.state + self.rng.normal(0, 0.05, self.state.shape)
            self.observation[:] = self.state

            # logger.info(f"{agent.name} action: {action}")


    def reset(self):
        pass

    def run(self):
        pass
