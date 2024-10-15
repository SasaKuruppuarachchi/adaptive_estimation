from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional, TypedDict, Any
from abc import ABC, abstractmethod
from tpc.utils.types import ControlTypes, AgentType

@dataclass
class AgentClassArgs:
    name: str

@dataclass
class Agent(ABC):

    type: AgentType
    args: AgentClassArgs
    # args: TypedDict
    # args: Dict[str, Any]

