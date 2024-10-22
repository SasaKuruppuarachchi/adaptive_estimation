from typing import List, Dict, Union, Optional, Any
from abc import ABC, abstractmethod

import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.service import Service
# from std_msgs.msg import String
# from std_srvs.srv import Trigger  
# from std_srvs.srv.Trigger import Response, Request

from tpc.utils.types import AgentType as Agent

# Abstract base class for CommunicationHandler
class ClientCommunicationHandler(ABC):

    def __init__(self, agent: Agent):
        self.agent: Agent = agent

    @abstractmethod
    def send_action_request(self, observation: np.ndarray) -> Any:
        pass

class ServerCommunicationHandler(ABC):

    def __init__(self, name: str, agent: Agent):
        self.name: str = name
        self.agent: Agent = agent

    @abstractmethod
    def receive_state_update(self, request: Any, response: Any) -> Any:
        pass

class LocalClientCommunicationHandler(ClientCommunicationHandler):

    def __init__(self, agent: Agent):
        super().__init__(agent=agent)

        self.agent: Agent = agent

    def send_action_request(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        """
        Send action request to the agent
        """

        # actions: Dict[str, np.ndarray] = {agent.name: agent.get_action() for agent in self.agents}
        success: bool = self.agent.step(observation=observation)
        action: np.ndarray = self.agent.get_action()
        return action

class LocalServerCommunicationHandler(ServerCommunicationHandler):

    def __init__(self, agent: Agent):
        super().__init__(name=agent.name, agent=agent)

    def receive_state_update(self, request: np.ndarray, response: np.ndarray) -> Any:
        """
        Here the client takes care of calling 
            `agent.step(observation=observation)` and `agent.get_action()`
            so no need to do anything here.
        """
        return None

# class ROSServerCommunicationHandler(ServerCommunicationHandler, Node):
#     """
#     Manages ROS Service for the simulator
#     """
#
#     def __init__(self, name: str, agent: Agent):
#         super().__init__(name=name, agent=agent)
#
#         self.service = self.create_service(Trigger, name, self.receive_state_update)
#
#
#     def receive_state_update(self, request: Request, response: Response) -> Response:
#         """
#         Receive state update from the simulator
#         """
#         success: bool = self.agent.step(observation=request)
#         response.message = self.agent.get_action()
#         response.success = success
#
#         return response
#
# class ROSClientCommunicationHandler(ClientCommunicationHandler):
#     """
#     Manages ROS Service for the agents
#     """
#     
#     def __init__(self, agent_names: List[str]):
#         super().__init__(agent_names)
#
#     def send_action_request(self, action: Dict[str, np.ndarray]):
#         """
#         Send action request to the agents
#         """
