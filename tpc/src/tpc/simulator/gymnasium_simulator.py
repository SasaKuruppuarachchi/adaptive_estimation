import time
from asyncio import Future
from typing import Dict, List, Tuple, Union, Callable
from abc import ABC, abstractmethod


from loguru import logger
logger.add("logs/main.log", rotation="500 MB")

import numpy as np
import gymnasium as gym
from gymnasium.wrappers.clip_action import ClipAction


# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType
from tpc_ros.srv import ActionRequest

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
        self.action: np.ndarray = np.zeros(self.action_shape)

        self.dt: float = self.env.unwrapped.dt


        response = ActionRequest.Response()
        response.action = self.action.flatten().tolist()
        self.future_: Future = Future()
        self.future_.set_result(response)

    def init_communication_handler(self, clients: List[ClientCommunicationHandler]):
        self.clients : List[ClientCommunicationHandler] = clients

    def start(self):
        if not self.clients:
            raise ValueError("No clients attached to the simulator")

        for client in self.clients:
            while not client.client.wait_for_service(timeout_sec=1.0):
                logger.info(f"Waiting for service {client.service_name}...")

        self.env.render()
        logger.info(f"Initial state: {self.state}")

        return True

    def stop(self):
        pass

    def step(self):

        if self.future_.done():
            self.action[:] = self.future_.result().action
            logger.info(f"received action: {self.action}")
            self.future_ = self.clients[0].send_action_request(
                                    observation=self.observation)


        # Send action to the simulator environment
        state, self.reward, terminated, truncated, info = self.env.step(self.action)
        # self.state[:] = state[:2] # Only pos_x and pos_y
        self.state[:] = pendulum_state_post_process_(state)

        self.observation[:] = self.state + self.observation_noise()

        time.sleep(self.dt)

    def reset(self):
        pass

    def run(self):
        pass
