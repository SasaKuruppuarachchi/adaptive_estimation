from typing import Dict, List, Union
from enum import Enum, auto

import numpy as np
from loguru import logger
from omegaconf import DictConfig, ListConfig

from tpc.simulator import Simulator
from tpc.agent import Agent
from tpc.visualiser.visualiser import GymnasiumPendulumVisualizer as Visualizer
from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
from tpc.agent.pendulum_agent import (
    tPCPendulumAgent,
    KalmanFilterPendulumAgent
)

from tpc.utils.types import ControlTypes, ControlType
from simple_pid.PID import PID

class PendulumState(Enum):
    x_0 = auto()
    x_1 = auto()

class PendulumObservations(Enum):
    y_0 = auto()
    y_1 = auto()

def init_sim(config: Union[DictConfig, ListConfig], rng: np.random.Generator) -> Dict:
    sim: Simulator = GymnasiumSimulator(**config.simulator)

    state_shape = sim.state_shape
    action_shape = sim.action_shape

    agents: List[Agent] = []
    for agent_key, agent_config in config.agents.items():

        if agent_config.type == 'tPCPendulumAgent':
            agent: Agent = tPCPendulumAgent(
                action_dim=action_shape,
                state=np.zeros_like(sim.state),
                observation=sim.observation,
                observation_estimate=np.zeros_like(sim.observation),
                **agent_config.args,
                **init_tPC_matrices(rng=rng, state_dim=state_shape[0], 
                    action_dim=action_shape[0], observation_dim=state_shape[0]),
            )
        elif agent_config.type == 'KalmanFilterPendulumAgent':
            agent: Agent = KalmanFilterPendulumAgent(
                action_dim=action_shape,
                state=np.zeros_like(sim.state),
                observation=sim.observation,
                observation_estimate=np.zeros_like(sim.observation),
                **agent_config.args,
            )


        agents.append(agent)
        agent.attach(simulator=sim)

    viz: Visualizer = Visualizer(
            # state: np.ndarray,
            # state_estimate: np.ndarray,
            observation = sim.observation,
            # observation_estimates = [agent.observation_estimate for agent in agents],
            observation_estimate = np.concatenate([agent.observation_estimate[None,...] for agent in agents], axis=0),
            # state_error: np.ndarray,
            # observation_error: np.ndarray,
            # positions_x=positions_x, positions_y=positions_y
            agent_names = [agent.name for agent in agents],
            **config.visualiser
                                 )

    return { 'simulator': sim, 'agents': agents, 'visualiser': viz}

def init_tPC_matrices(rng: np.random.Generator, state_dim: int, action_dim: int, observation_dim: int) -> Dict:
    # A = rng.normal(0, 1, (state_dim, state_dim))
    # C = rng.normal(0, 1, (observation_dim, state_dim))
    A = np.zeros((state_dim, state_dim))
    C = np.eye(state_dim)

    B = np.zeros((state_dim, action_dim))
    B[-1, -1] = 1 

    return { 'A': A, 'B': B, 'C': C }
