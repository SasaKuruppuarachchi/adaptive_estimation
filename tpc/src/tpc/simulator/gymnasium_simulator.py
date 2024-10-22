# import logging
# logger = logging.getLogger(__name__)
from loguru import logger
logger.add("logs/main.log", rotation="500 MB")

from typing import Dict, List, Tuple, Union, Callable
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym
from gymnasium.wrappers import ClipAction

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType

from tpc.simulator import Simulator
from tpc.communication.base import ClientCommunicationHandler

def pendulum_state_post_process_(state: np.ndarray) -> np.ndarray:
    """
    Compute angle based on x-y position

    """
    # theta: float = np.arctan2(state[1], state[0]) + np.pi
    theta: float = arctan2_2pi(state[1], state[0])
    return np.array([theta, state[2]])

def arctan2_2pi(y, x):
    return -1 * (
            np.arctan2(
            y, (-1) * x
        ) - np.pi
    )

class GymnasiumSimulator(Simulator):

    def __init__(self,
        # state_dim: int,
        # action_dim: int,
        # dt: float,
        seed: int,
        name: str,
        observation_noise_std: float,
        # communication_handler: ClientCommunicationHandler,
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
        if self.env.observation_space.shape and env_name != 'Pendulum-v1':
            self.state_shape = self.env.observation_space.shape
        elif env_name == 'Pendulum-v1':
            # We'll post-process the state to get the angle
            self.state_shape = np.array([2])

        if env_name == 'Pendulum-v1':
            state, _ = self.env.reset()
            self.state: np.ndarray = pendulum_state_post_process_(state)
        else:
            raise NotImplementedError(
                "Only Pendulum-v1 is supported."
                "We're always post processing the state to be theta and theta dot"
            )
            self.state, _ = self.env.reset()

        self.observation_noise: Callable = lambda: self.rng.normal(
                                    0, observation_noise_std, self.state_shape
        )
        self.observation = self.state + self.observation_noise()

        self.dt: float = self.env.dt


    def init_communication_handler(self, clients: List[ClientCommunicationHandler]):
        self.clients : List[ClientCommunicationHandler] = clients

    def start(self):
        if not self.clients:
            raise ValueError("No clients attached to the simulator")

        self.env.render()
        logger.info(f"Initial state: {self.state}")

        return True

    def stop(self):
        pass

    def step(self):

        # action = self.env.action_space.sample()
        # for client in self.clients:
        #     action = client.send_action_request(observation=self.observation)
        action = self.clients[0].send_action_request(
                                observation=self.observation)

        # Send action to the simulator environment
        state, self.reward, terminated, truncated, info = self.env.step(action)
        # self.state[:] = state[:2]
        self.state[:] = pendulum_state_post_process_(state)

        self.observation[:] = self.state + self.observation_noise()

    def reset(self):
        pass

    def run(self):
        pass
