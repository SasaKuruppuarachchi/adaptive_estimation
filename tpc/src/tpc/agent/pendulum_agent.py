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
        # state_dim: Union[Tuple[int], np.ndarray],
        action_dim: Union[Tuple[int], np.ndarray],
        # observation_dim: Union[Tuple[int], np.ndarray],
        A: np.ndarray, C: np.ndarray, B: np.ndarray,
        control_type: ControlTypes,
        controller_args: Dict,
        dt: float,
        state: np.ndarray,
        observation: np.ndarray,
        observation_estimate: np.ndarray,
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
        self.state: np.ndarray = state
        self.observation: np.ndarray = observation
        self.observation_estimate: np.ndarray = observation_estimate
        self.action: np.ndarray = np.zeros(action_dim)
        self.A: np.ndarray = A
        self.B: np.ndarray = B
        self.C: np.ndarray = C
        self.dt: float = dt
        self.k1, self.k2 = k1, k2
        # self.error: np.ndarray = np.zeros(state_dim)
        self.error: np.ndarray = np.zeros_like(observation)
        self.inference_duration: int = inference_duration
        self.learning_duration: int = learning_duration

        self.controller: ControlType = get_controller(control_type, controller_args)
        self.f, self.df = get_activation(activation)
        logger.info(f'Temporal Predictive Coding using a {activation} function')

    def attach(self, simulator):
        simulator.agents[self.name] = self

    def compute_action(self):

        self.action[:] = self.controller(self.state[0])
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
            self.observation_estimate[:] = self.C @ self.f(self.state)
            error_observation[:] = self.observation - self.observation_estimate
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
            #                 self.observation - self.observation_estimate) ** 2
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

class LinearKalmanFilterPendulumAgent(Agent):
    """
    Kalman filter
    """

    def __init__(self, name:str,
                # A: np.ndarray, B: np.ndarray, C: np.ndarray,
                # Q: np.ndarray, R: np.ndarray, 
                state: np.ndarray,
                observation: np.ndarray,
                observation_estimate: np.ndarray,
                action_dim: Union[Tuple[int], np.ndarray],
                control_type: ControlTypes, controller_args: Dict,
                 ) -> None:


        self.name: str = name
        self.state: np.ndarray = state
        self.observation: np.ndarray = observation
        self.observation_estimate: np.ndarray = observation_estimate
        self.action: np.ndarray = np.zeros(action_dim)

        # super().__init__() # TODO Do we want to call the super constructor?
        A = np.array([[1, 1], [0, 1]])
        B = np.array([[0], [1]])
        C = np.array([[1, 0], [0, 1]])
        Q = np.eye(state.shape[0])
        R = np.eye(state.shape[0])

        self.A: np.ndarray = A
        self.B: np.ndarray = B
        self.C: np.ndarray = C

        # covariance matrix of noise
        self.Q = Q
        self.R = R

        # initialize covariance estimate of the latent state
        self.P: np.ndarray = np.eye(state.shape[0])

        self.controller: ControlType = get_controller(control_type, controller_args)

    def attach(self, simulator):
        simulator.agents[self.name] = self


    def compute_action(self):
        self.action[:] = self.controller(self.state[0])
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

        state_proj, P_proj = self.projection()
        self.correction(state_proj, P_proj)
        self.observation_estimate = np.matmul(self.C, state_proj)

        return True

    def get_action(self):
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
                # A: np.ndarray, B: np.ndarray, C: np.ndarray,
                # Q: np.ndarray, R: np.ndarray, 
                state: np.ndarray,
                observation: np.ndarray,
                observation_estimate: np.ndarray,
                action_dim: Union[Tuple[int], np.ndarray],
                control_type: ControlTypes, controller_args: Dict,
                dt: float, 
                state_noise_std: float, observation_noise_std: float,
                pendulum_length: float = 1, gravity: float = 9.81,
                ) -> None:


        self.name: str = name
        self.state: np.ndarray = state
        self.observation: np.ndarray = observation
        self.observation_estimate: np.ndarray = observation_estimate
        self.action: np.ndarray = np.zeros(action_dim)
        self.dt: float = dt
        self.g: float = gravity
        self.l: float = pendulum_length

        # super().__init__() # TODO Do we want to call the super constructor?
        A = np.array([[1.0, self.dt], [-self.dt*(self.g/self.l)*np.cos(self.state[0]), 1]])
        B = np.array([[0], [1]])
        C = np.array([[1, 0], [0,0]])
        Q = np.eye(state.shape[0])*state_noise_std
        R = np.eye(state.shape[0])*observation_noise_std

        self.A: np.ndarray = A
        self.B: np.ndarray = B
        self.C: np.ndarray = C

        # covariance matrix of noise
        self.Q = Q
        self.R = R

        # initialize covariance estimate of the latent state
        self.P: np.ndarray = np.eye(state.shape[0])

        self.controller: ControlType = get_controller(control_type, controller_args)

    def get_nonlinear_dynamics(self, x):
        return np.array([x[1], -self.g/self.l*np.cos(x[0])]) + self.B @ self.action

    def update_A(self, x):
        """
        A is the jacobian of the state equation linearized around x
        """
        # return self.A[:] = [[1.0, self.dt], [-self.dT*(self.g/self.l)*np.cos(x[0,0]), 1])
        self.A[1,0] = -self.dt*(self.g/self.l)*np.cos(x[0])

    def update_C(self):
        """
        C is the jacobian of the output equation
        """
        pass

    def attach(self, simulator):
        simulator.agents[self.name] = self


    def compute_action(self):
        self.action[:] = self.controller(self.state[0])
        return True

    def projection(self):
        # state_proj = np.matmul(self.A, self.state) + np.matmul(self.B, self.action)
        # P_proj = np.matmul(self.A, np.matmul(self.P, self.A.T)) + self.Q

        state_proj = self.state + self.dt * self.get_nonlinear_dynamics(self.state)
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

        self.update_A(self.state)
        state_proj, P_proj = self.projection()

        # self.update_C()
        self.correction(state_proj, P_proj)
        self.observation_estimate[:] = np.matmul(self.C, state_proj)
        # import pdb; pdb.set_trace()

        return True

    def get_action(self):
        return self.action

    def step(self) -> bool:

        # Compute state
        success_state: bool = self.compute_state()

        # Compute action
        success_action: bool =  self.compute_action()

        return success_state and success_action
