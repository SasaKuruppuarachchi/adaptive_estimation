from abc import ABC, abstractmethod
from typing import Tuple, List, Dict, Union, Optional
from loguru import logger
from simple_pid import PID

import numpy as np

from tpc.agent import Agent
# from tpc.utils.utils import PendulumState, PendulumObservations

from tpc.utils.types import SimulatorType, ControlTypes

class PendulumAgent(Agent):
    

    def __init__(self,
        state_dim: Union[Tuple[int], np.ndarray],
        action_dim: Union[Tuple[int], np.ndarray],
        observation_dim: Union[Tuple[int], np.ndarray],
        A: np.ndarray, C: np.ndarray, B: np.ndarray,
        name: str,
        control_type: ControlTypes,
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


def linear(x):
    return x


def linear_deriv(x):
    return np.array([np.sum(np.eye(x.shape[0]), axis=0)]).reshape(2, )


def tanh(x):
    return np.tanh(x)


def tanh_deriv(x):
    return 1 - tanh(x) ** 2

class tPCPendulumAgent(Agent):
    

    def __init__(self, name: str,
        state_dim: Union[Tuple[int], np.ndarray],
        action_dim: Union[Tuple[int], np.ndarray],
        observation_dim: Union[Tuple[int], np.ndarray],
        A: np.ndarray, C: np.ndarray, B: np.ndarray,
        control_type: ControlTypes,
        controller_args: Dict,
        dt: float,
        inference_duration: int,
        learning_duration: int = 1,
        k1: float = 0.001, k2: float = 0.008,
        activation: str = 'nonlinear',
    ):
        """
        dt: float
            Time step/step size for state update

        """
        self.state: np.ndarray = np.zeros(state_dim)
        self.observation: np.ndarray = np.zeros(state_dim)
        self.predicted_observation: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)
        self.A: np.ndarray = A
        self.B: np.ndarray = B
        self.C: np.ndarray = C
        self.dt: float = dt
        self.k1, self.k2 = k1, k2
        self.error: np.ndarray = np.zeros(state_dim)
        self.inference_duration: int = inference_duration
        self.learning_duration: int = learning_duration

        
        if control_type == ControlTypes.PID:
            self.controller = PID(
                **controller_args
            )
        elif control_type == ControlTypes.RANDOM:
            self.controller = lambda x: np.random.normal(0, 1, 1)
        elif control_type == ControlTypes.LQR:
            raise NotImplementedError("LQR not implemented yet")
        else:
            logger.error(f'Invalid control type: {control_type}. Only "PID", "Random" or "LQR" allowed.')
            raise KeyError()

        self.name = name

        if activation == 'linear':
            self.f = linear
            self.df = linear_deriv
        elif activation == 'nonlinear':
            self.f = tanh
            self.df = tanh_deriv
        else:
            logger.error(f'Invalid activation: {activation}. Only "linear" or "nonlinear" allowed.')
            raise KeyError()
        logger.info(f'Temporal Predictive Coding using a {activation} function')

    def attach(self, simulator):
        simulator.agents[self.name] = self

    def compute_action(self):

        # self.action[:] = (np.random.normal(0, 1, self.action.shape) > 0).astype(int)
        # self.action[:] = np.random.normal(0, 1, self.action.shape)
        return True

    def compute_state(self, C_decay: Optional[int] = None, A_decay: Optional[int] = None):
        # self.state[:] = np.random.normal(0, 1, self.state.shape)

        if C_decay:
            C_decay_counter = 1
        if A_decay:
            A_decay_counter = 1

        # TODO Preallocate these in constructor?
        prev_state: np.ndarray = self.state.copy()
        error_observation: np.ndarray = np.zeros(self.observation.shape)
        error_state: np.ndarray = np.zeros(self.state.shape)

        for t in range(self.inference_duration):
            prev_state[:] = self.state.copy()
            self.state[:] = self.state + self.dt * (self.A @ self.f(self.state) + self.B @ self.action)
            self.predicted_observation[:] = self.C @ self.f(self.state)
            error_observation[:] = self.observation - self.predicted_observation
            error_state[:] = self.C.T @ self.df(self.state) * error_observation
            self.state[:] = self.state + self.dt * (self.C.T @ (self.df(self.state) * error_observation))
            self.C += self.dt * (self.k1 * error_observation[..., np.newaxis] @ self.f(self.state)[..., np.newaxis].T)
            self.A += self.dt * (self.k2 * error_state[..., np.newaxis] @ self.f(prev_state)[..., np.newaxis].T)

            if A_decay:
                if A_decay_counter == A_decay:
                    self.k2 /= 1.015
                    C_decay_counter = 1
            if C_decay:
                if C_decay_counter == C_decay:
                    self.k1 /= 1.015
                    C_decay_counter = 1
            # self.error[:, t] = np.linalg.norm(
            #                 self.observation - self.predicted_observation) ** 2
            self.error[:] = np.linalg.norm(error_observation) ** 2
            if A_decay:
                A_decay_counter += 1
            if C_decay:
                C_decay_counter += 1

        return True

    def get_action(self):

        self.theta = np.arctan2(self.state[1], self.state[0])
        self.theta_dot = self.state[2]

        self.action[:] = self.controller(self.theta)

        # return int(self.action.item())
        return self.action

    def step(self) -> bool:

        # Compute state
        success_state: bool = self.compute_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action

