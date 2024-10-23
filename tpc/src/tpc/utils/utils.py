from typing import Dict, List, Union, Tuple, Optional
from dataclasses import dataclass
from enum import Enum, auto
from threading import Thread

import numpy as np
from loguru import logger
from omegaconf import DictConfig, ListConfig
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from tpc.simulator import Simulator
from tpc.agent import Agent
from tpc.visualiser.visualiser import GymnasiumPendulumVisualizer as Visualizer
from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
from tpc.agent.pendulum_agent import (
    tPCPendulumAgent,
    KalmanFilterPendulumAgent
)
from tpc.communication.base import (
    LocalClientCommunicationHandler as LocalClient,
    LocalServerCommunicationHandler as LocalServer,
    ROSClientCommunicationHandler as ROSClient,
    ROSServerCommunicationHandler as ROSServer,
    ClientCommunicationHandler as Client,
    ServerCommunicationHandler as Server,
)

from tpc.utils.types import ControlTypes, ControlType, CommunicationTypes
from simple_pid.PID import PID

class PendulumState(Enum):
    x_0 = auto()
    x_1 = auto()

class PendulumObservations(Enum):
    y_0 = auto()
    y_1 = auto()

class States:
    def __init__(self,
                 num_time_steps: int,
                 states_shape: Tuple[int, ...],
                 observations_shape: Tuple[int, ...],
                 agent_names: List[str],
                 action_shape: Tuple[int, ...] = (1,),
                 fill_value: float = np.nan
                 ):
        self.num_time_steps: int = num_time_steps
        self.states: np.ndarray = np.full((num_time_steps, *states_shape), fill_value)
        self.observations: np.ndarray = np.full((num_time_steps, *observations_shape), fill_value)
        self.sim_dt: np.ndarray = np.full((num_time_steps, 1), fill_value)
        self.actions: np.ndarray = np.full((num_time_steps, *action_shape), fill_value)

        self.agents_state_estimates: Dict[str, np.ndarray] = {}
        self.agents_observation_estimates: Dict[str, np.ndarray] = {}
        self.agents_actions: Dict[str, np.ndarray] = {}
        for name in agent_names:
            self.agents_state_estimates[name] = np.full(
                            (num_time_steps, *states_shape), fill_value)

            self.agents_observation_estimates[name] = np.full(   
                             (num_time_steps, *observations_shape), fill_value)

            self.agents_actions[name] = np.full(
                        (num_time_steps, *action_shape), fill_value)

    def update(self, agent: Agent, simulator: Simulator, step:int ):

            self.states[step] = simulator.state
            self.observations[step] = simulator.observation
            self.actions[step] = simulator.action

            self.agents_state_estimates[agent.name][step] = agent.state


def init_sim(config: Union[DictConfig, ListConfig], rng: np.random.Generator) -> Dict:

    if config.communication_type == CommunicationTypes.LOCAL:
        ServerClass = LocalServer
        ClientClass = LocalClient
    elif config.communication_type == CommunicationTypes.ROS:
        ServerClass = ROSServer
        ClientClass = ROSClient
        import rclpy
        rclpy.init(args=None)
    else:

        msg: str = str(f"Communication type {config.communication_type} not implemented.\n"
                    f"Expected one of {CommunicationTypes}. Got {config.communication_type}")
        logger.error(msg)
        raise ValueError(
            msg
        )

    sim: Simulator = GymnasiumSimulator(**config.simulator)
    config.dt = sim.dt

    state_shape = sim.state_shape
    action_shape = sim.action_shape

    agents: List[Agent] = []
    
    if config.communication_type == CommunicationTypes.ROS:
        executor: Union[SingleThreadedExecutor, MultiThreadedExecutor] = MultiThreadedExecutor()
        # executor: SingleThreadedExecutor = SingleThreadedExecutor()
    else:
        executor = None
    for agent_key, agent_config in config.agents.items():


        if agent_config.type == 'tPCPendulumAgent':
            agent: Agent = tPCPendulumAgent(
                action_dim=action_shape,
                state=np.zeros_like(sim.state),
                observation=sim.observation,
                observation_estimate=np.zeros_like(sim.observation),
                **agent_config.args,
                **init_tPC_matrices(rng=rng, 
                    state_dim=state_shape[0], 
                    action_dim=action_shape[0],
                    observation_dim=state_shape[0]),
            )
        elif agent_config.type == 'KalmanFilterPendulumAgent':
            agent: Agent = KalmanFilterPendulumAgent(
                action_dim=action_shape,
                state=np.zeros_like(sim.state),
                observation=sim.observation,
                observation_estimate=np.zeros_like(sim.observation),
                **agent_config.args,
            )
        else:
            raise ValueError(
                f"Agent type {agent_config.type} not implemented")

        server: Server = ServerClass(
            agent=agent
        )
        agent.init_communication_handler(server=server)

        # Create a thread for the agent and the server
        if config.communication_type == CommunicationTypes.ROS:

            executor.add_node(agent.server)

        agents.append(agent)
        agent.attach(simulator=sim)


    num_time_steps: int = int(config.duration/sim.dt)
    states: States = States(
        num_time_steps=num_time_steps,
        states_shape=sim.state.shape,
        observations_shape=sim.observation.shape,
        agent_names=[agent.name for agent in agents]
    )

    clients: List[LocalClient] = []
    for agent in agents:
        client = ClientClass(agent=agent)
        clients.append(client)
        if config.communication_type == CommunicationTypes.ROS:
            executor.add_node(client)

    sim.init_communication_handler(clients=clients)

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
            **config.visualiser,
            n_frames=num_time_steps
                                 )

    return { 
        'simulator': sim, 'agents': agents, 
        'visualiser': viz, 'states': states,
        'executor': executor
    }

def init_tPC_matrices(rng: np.random.Generator, state_dim: int, action_dim: int, observation_dim: int) -> Dict:
    # A = rng.normal(0, 1, (state_dim, state_dim))
    # C = rng.normal(0, 1, (observation_dim, state_dim))
    A = np.zeros((state_dim, state_dim))
    C = np.eye(state_dim)

    B = np.zeros((state_dim, action_dim))
    B[-1, -1] = 1 

    return { 'A': A, 'B': B, 'C': C }
