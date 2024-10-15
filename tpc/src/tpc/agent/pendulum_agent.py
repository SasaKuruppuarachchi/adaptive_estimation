from abc import ABC, abstractmethod
from typing import Tuple, List, Dict, Union, Optional
from loguru import logger

import numpy as np

from tpc.agent import Agent
# from tpc.utils.utils import PendulumState, PendulumObservations

from tpc.utils.types import SimulatorType, ControlTypes, ControlType
from tpc.utils.control import get_controller

def linear(x):
    return x

def linear_deriv(x):
    return np.array([np.sum(np.eye(x.shape[0]), axis=0)]).reshape(2, )

def tanh(x):
    return np.tanh(x)

def tanh_deriv(x):
    return 1 - tanh(x) ** 2

def get_activation(activation: str):
    if activation == 'linear':
        return linear, linear_deriv
    elif activation == 'nonlinear':
        return tanh, tanh_deriv
    else:
        logger.error(f'Invalid activation: {activation}. Only "linear" or "nonlinear" allowed.')
        raise KeyError()

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
        self.name = name
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

        self.controller: ControlType = get_controller(control_type, controller_args)
        self.f, self.df = get_activation(activation)
        logger.info(f'Temporal Predictive Coding using a {activation} function')

    def attach(self, simulator):
        simulator.agents[self.name] = self

    def compute_action(self):

        self.theta = np.arctan2(self.state[1], self.state[0])
        self.theta_dot = self.state[2]

        self.action[:] = self.controller(self.theta)
        return True

    def compute_state(self, C_decay: Optional[int] = None, A_decay: Optional[int] = None):

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
        # return int(self.action.item())
        return self.action

    def step(self) -> bool:

        # Compute state
        success_state: bool = self.compute_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action

class KalmanFilterPendulumAgent(Agent):
    """
    Kalman filter
    """

    def __init__(self, name:str,
                A: np.ndarray, B: np.ndarray, C: np.ndarray,
                Q: np.ndarray, R: np.ndarray, latent_size: int,
                state_dim: Union[Tuple[int], np.ndarray],
                action_dim: Union[Tuple[int], np.ndarray],
                observation_dim: Union[Tuple[int], np.ndarray],
                control_type: ControlTypes, controller_args: Dict,
                 ) -> None:


        self.name: str = name
        self.state: np.ndarray = np.zeros(state_dim)
        self.observation: np.ndarray = np.zeros(state_dim)
        self.predicted_observation: np.ndarray = np.zeros(state_dim)
        self.action: np.ndarray = np.zeros(action_dim)

        super().__init__()
        self.A: np.ndarray = A
        self.B: np.ndarray = B
        self.C: np.ndarray = C

        # control input, a list/1d array
        self.latent_size: int = latent_size

        # covariance matrix of noise
        self.Q = Q
        self.R = R

        # initialize covariance estimate of the latent state
        self.P: np.ndarray = np.eye(state_dim[0])

        self.controller: ControlType = get_controller(control_type, controller_args)

    def attach(self, simulator):
        simulator.agents[self.name] = self


    def compute_action(self):

        self.theta = np.arctan2(self.state[1], self.state[0])
        self.theta_dot = self.state[2]

        self.action[:] = self.controller(self.theta)
        return True

    def projection(self):
        state_proj = np.matmul(self.A, self.state) + np.matmul(self.B, self.action)
        P_proj = np.matmul(self.A, np.matmul(self.P, self.A.T)) + self.Q
        return state_proj, P_proj

    def correction(self, state_proj, P_proj):
        """Correction step in KF

        K: Kalman gain
        """
        K = np.matmul(np.matmul(P_proj, self.C.T),
                         np.linalg.inv(np.matmul(np.matmul(self.C, P_proj), self.C.T) + self.R))
        self.state = state_proj + np.matmul(K, self.observation - np.matmul(self.C, state_proj))
        self.P = P_proj - np.matmul(K, np.matmul(self.C, P_proj))

    def compute_state(self):

        # self.x = self.observation
        # self.u = self.action

        state_proj, P_proj = self.projection()
        self.correction(state_proj, P_proj)
        self.pred_observation = np.matmul(self.C, state_proj)

    def get_action(self):
        return self.action

    def step(self) -> bool:

        # Compute state
        success_state: bool = self.compute_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action
