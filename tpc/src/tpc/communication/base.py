import time
from enum import Enum, auto
from typing import List, Dict, Union, Optional, Any
from abc import ABC, abstractmethod
from asyncio import Future

from loguru import logger

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.service import Service
from std_msgs.msg import String
from tpc_ros.srv import ActionRequest

from tpc.utils.types import AgentType as Agent


# Abstract base class for CommunicationHandler
class ClientCommunicationHandler(ABC):

    def __init__(self, service_name: str):
        self.service_name: str = service_name
        self.client: Any

    @abstractmethod
    def send_action_request(self, observation: np.ndarray) -> Any:
        pass

class ServerCommunicationHandler(ABC):

    def __init__(self, service_name: str):
        self.service_name: str = service_name

    @abstractmethod
    def receive_state_update(self, request: Any, response: Any) -> Any:
        pass

class LocalServerCommunicationHandler(ServerCommunicationHandler):

    def __init__(self, agent: Agent):
        service_name = f'{agent.name}/action_request'
        super().__init__(service_name=service_name)

        self.agent: Agent = agent

    def receive_state_update(self, request: np.ndarray, response: np.ndarray) -> Any:
        """
        Here the client takes care of calling 
            `agent.step(observation=observation)` and `agent.get_action()`
            so no need to do anything here.
        """
        return None

class LocalClientCommunicationHandler(ClientCommunicationHandler):

    def __init__(self, agent: Agent):
        super().__init__(
            service_name=f'{agent.name}/action_request'
        )

        self.agent: Agent = agent

    def send_action_request(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        """
        Send action request to the agent
        """

        # actions: Dict[str, np.ndarray] = {agent.name: agent.get_action() for agent in self.agents}
        success: bool = self.agent.step(observation=observation)
        action: np.ndarray = self.agent.get_action()
        return action


class ROSServerCommunicationHandler(ServerCommunicationHandler, Node):
    """
    Manages ROS Service for the simulator
    """

    def __init__(self, agent: Agent):
        service_name = f'{agent.name}/action_request'
        self.agent: Agent = agent
        ServerCommunicationHandler.__init__(
            self, service_name=service_name)
        Node.__init__(self, node_name=agent.name)

        self.service: Service = self.create_service(
            ActionRequest, service_name, self.receive_state_update)

        logger.info(f"ROS Server {self.service.srv_name} started")
        # input("Press Enter to continue...")

    def receive_state_update(self, request, response) -> Any:
        """
        Receive state update from the simulator
        """

        logger.info(f"SERVER Received observation: {request.observation}")
        success: bool = self.agent.step(observation=request.observation)
        response.action = self.agent.get_action().flatten().tolist()
        logger.info(f"SERVER Sending action: {response.action}")

        return response

class ROSClientCommunicationHandler(ClientCommunicationHandler, Node):
    """
    Manages ROS Client for the simulator
    """

    # def __init__(self, service_name: str):
    def __init__(self, agent: Agent):
        self.service_name: str = f"{agent.name}/action_request"
        ClientCommunicationHandler.__init__(self, service_name=self.service_name)
        Node.__init__(self, node_name=f'simulator_{agent.name}')
        self.client: Client = self.create_client(ActionRequest, self.service_name)

        logger.info(f"ROS Client {self.client.srv_name} started")
        # input("Press Enter to continue...")

    def send_action_request(self, observation):
        logger.info(f"CLIENT Sending observation to {self.service_name}: {observation}")
        request = ActionRequest.Request()
        request.observation = observation.flatten().tolist()
        future: Future = self.client.call_async(request)
        return future
