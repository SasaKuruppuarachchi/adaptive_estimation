from enum import Enum, auto
from typing import List, Dict, Union, Optional, Any
from abc import ABC, abstractmethod

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_msgs.msg import String
from std_srvs.srv import Trigger  

from tpc.utils.types import AgentType as Agent


# Abstract base class for CommunicationHandler
class ClientCommunicationHandler(ABC):

    def __init__(self, service_name: str):
        self.service_name: str = service_name

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
        ServerCommunicationHandler.__init__(
            self, service_name=service_name)
        Node.__init__(self, node_name=agent.name)

        self.request: np.ndarray = np.zeros_like(agent.observation)

        self.service = self.create_service(
            Trigger, service_name, self.receive_state_update)

    def receive_state_update(self, request, response) -> Any:
        """
        Receive state update from the simulator
        """

        self.request[:] = request.data
        success: bool = self.agent.step(observation=self.request)
        response.message = self.agent.get_action()
        response.success = success

        return response

from std_msgs.msg import Float64MultiArray
class ROSClientCommunicationHandler(ClientCommunicationHandler, Node):
    """
    Manages ROS Client for the simulator
    """

    # def __init__(self, service_name: str):
    def __init__(self, agent: Agent):
        self.service_name: str = agent.name+'_action_request'
        ClientCommunicationHandler.__init__(self, service_name=self.service_name)
        Node.__init__(self, node_name='simulator')
        self.client = self.create_client(Float64MultiArray, self.service_name)

    def send_action_request(self, observation):
        request = Float64MultiArray
        # import pdb; pdb.set_trace()
        request.data = observation.flatten().tolist()
        future = self.client.call_async(request)
        return future
