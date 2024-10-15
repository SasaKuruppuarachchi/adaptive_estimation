from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
from abc import ABC, abstractmethod

@dataclass
class Agent(ABC):

    name: str
