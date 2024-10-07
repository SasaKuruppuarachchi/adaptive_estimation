from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

@dataclass
class Simulator:

    name: str
    env_name: str = "BOO"

