from typing import List, Dict, Union, Optional, Any
from threading import Thread
from asyncio import Future
import time


from loguru import logger

import numpy as np
from numpy.testing import assert_almost_equal
from omegaconf import OmegaConf
import pytest
import rclpy


from tpc.agent import Agent
from tpc.simulator import Simulator
from tpc.communication.base import (
    LocalClientCommunicationHandler as LocalClient,
    LocalServerCommunicationHandler as LocalServer,
    ROSClientCommunicationHandler as ROSClient,
    ROSServerCommunicationHandler as ROSServer,
    ClientCommunicationHandler as Client,
    ServerCommunicationHandler as Server,
)
from tpc.utils.types import ControlTypes
from tpc.utils.utils import run_agent_with_server, States

from tpc_ros.srv import ActionRequest

"""
TODO: True negative tests e.g.:

with pytest.raises(NotImplementedError) as e:
    val = self.some_method(args=args)
"""

class TestAgent(Agent):

    def __init__(self,
                 name: str,
                 control_type: ControlTypes,
                 state_size: int,
                 ):
        super().__init__(name=name, control_type=control_type)

        self.state: np.ndarray = np.zeros(state_size)
        self.observation: np.ndarray = np.zeros_like(self.state)
        self.action: np.ndarray = np.array([0.0])

        self.control_type: ControlTypes = control_type

    def init_communication_handler(self, server: Server):
        self.server: Server = server

    def step(self, observation: np.ndarray) -> bool:

        self.observation[:] = observation
        self.state[:] = observation
        self.action[:] = self.action + 1.0

        return True

    def get_action(self) -> np.ndarray:
        return self.action

    def is_controlling(self):
        return True if self.control_type is not ControlTypes.NONE else False

    def attach(self, simulator: Simulator):
        pass

class TestSimulator(Simulator):

    def __init__(self, 
        seed: int,
        name: str,
        observation_noise_std: float,
        # communication_handler: ClientCommunicationHandler,
        env_name: str = 'CartPole-v1',
        render_mode: str = 'human',
        state_size: int = 3,):

        self.state: np.ndarray = np.zeros(state_size)
        self.observation: np.ndarray = np.zeros_like(self.state)
        self.action: np.ndarray = np.array([0.0])

        response = ActionRequest.Response()
        response.action = self.action.flatten().tolist()
        self.future_: Future = Future()
        self.future_.set_result(response)

        self.clients: List[Client] = []

    def init_communication_handler(self, clients: List[Client]):
        self.clients : List[Client] = clients

    def start(self):
        super().start()
        logger.info(f"Initial state: {self.state}")

        return True

    def step(self):

        # while not self.future_.done():
        #     logger.info(f"Waiting for action request from {self.clients[0].service_name}...")
        #     time.sleep(0.1)

        if self.future_.done():
            # import pdb; pdb.set_trace()
            self.action[:] = self.future_.result().action
            logger.info(f"received action: {self.action}")
            self.future_ = self.clients[0].send_action_request(
                                    observation=self.observation)

        self.state[:] = self.state + 1.0
        self.observation[:] = self.state


class TestROSCommunicationHandlers:
    def __init__(self):
        self.spin_timeout_sec: float = 2.5
        self.num_time_steps: int = 5

        rclpy.init(args=None)
        self.agent: Agent = self.init_agent()
        self.agents: List[TestAgent] = [self.agent]
        self.simulator: Simulator = self.init_simulator()
        self.server = ROSServer(self.agent)
        self.client = ROSClient(self.agent)

        self.agent.init_communication_handler(server=self.server)
        self.simulator.init_communication_handler(clients=[self.client])

        self.states: States = States(
            num_time_steps=self.num_time_steps,
            states_shape=self.simulator.state.shape,
            observations_shape=self.simulator.observation.shape,
            agent_names=[agent.name for agent in self.agents]
        )




    def init_agent(self):
        agent = TestAgent(name='test_agent', control_type=ControlTypes.NONE, state_size=3)
        return agent

    def init_simulator(self):
        simulator = TestSimulator(seed=0, name='test_simulator', observation_noise_std=0.1, state_size=3)
        return simulator

    def test_ros_server_client_naming(self):
        assert self.server.service.srv_name == 'test_agent/action_request'
        assert self.client.client.srv_name == 'test_agent/action_request'

    def test_ros_server_client_communication_executor(self):

        # executor = rclpy.executors.MultiThreadedExecutor()
        executor = rclpy.executors.SingleThreadedExecutor()
        # executor = rclpy.executors.StaticSingleThreadedExecutor()
        executor.add_node(self.agent.server)
        executor.add_node(self.simulator.clients[0])

        self.simulator.start()

        for i in range(self.num_time_steps):
            print("#"*20 + f" Step {i} " + "#"*20)
            self.simulator.step()

            executor.spin_once(timeout_sec=self.spin_timeout_sec)

            time.sleep(0.05)

            self.states.update(agent=self.agent, simulator=self.simulator, step=i)

        print(self.states.states)
        print(self.states.agents_state_estimates[self.agent.name])

        executor.shutdown()
        rclpy.shutdown()

    def test_ros_server_client_communication_threading(self):

        agent_thread = Thread(
            target=rclpy.spin,
            args=(self.agent.server,))
        agent_thread.start()
         
        simulator_thread = Thread(
            target=rclpy.spin,
            args=(self.simulator.clients[0],))
        simulator_thread.start()

        self.simulator.start()

        for i in range(self.num_time_steps):
            print("#"*20 + f" Step {i} " + "#"*20)
            self.simulator.step()

            time.sleep(0.05)

            self.states.update(agent=self.agent, simulator=self.simulator, step=i)

        print(self.states.states)
        print(self.states.agents_state_estimates[self.agent.name])

        rclpy.shutdown()

    def test_ros_server_client_communication_spin_once(self):

        self.simulator.start()

        for i in range(self.num_time_steps):
            print("#"*20 + f" Step {i} " + "#"*20)
            self.simulator.step()

            rclpy.spin_once(
                self.simulator.clients[0],
                timeout_sec=self.spin_timeout_sec)
            rclpy.spin_once(
                self.agent.server,
                timeout_sec=self.spin_timeout_sec)

            time.sleep(0.05)
            self.states.update(agent=self.agent, simulator=self.simulator, step=i)

        print(self.states.states)
        print(self.states.actions)
        # print(self.states.agents_state_estimates[self.agent.name])

        rclpy.shutdown()
