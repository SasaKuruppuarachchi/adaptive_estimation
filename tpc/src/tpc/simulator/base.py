from typing import Dict, List, Tuple, Any
from abc import ABC, abstractmethod

import numpy as np
import gymnasium as gym
from loguru import logger

# from tpc.utils.utils import PendulumState, PendulumObservations
from tpc.utils.types import AgentType, SimulatorType
from tpc.communication.base import ClientCommunicationHandler as Client
from tpc.agent import Agent

class Simulator(ABC):

    def __init__(self):

        self.agents: Dict[str, Agent] = {}
        self.state_shape: np.ndarray
        self.action_shape: np.ndarray
        self.clients: List[Client] = []
        
    def init_communication_handler(self, clients: List[Client]):
        pass

    def start(self):
        if not self.clients:
            raise ValueError("No clients attached to the simulator")

        for client in self.clients:
            while not client.client.wait_for_service(timeout_sec=1.0):
                logger.info(f"Waiting for service {client.service_name}...")
    #
    # @abstractmethod
    # def stop(self):
    #     pass

    @abstractmethod
    def step(self):
        pass

    # @abstractmethod
    # def reset(self):
    #     pass
    # 
    # @abstractmethod
    # def run(self):
    #     pass
