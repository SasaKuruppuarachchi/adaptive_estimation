from tpc.config.root import Config
from tpc.config.agent.pendulum_agent import (
    PendulumAgent
)
from tpc.config.simulator.pendulum_simulator import (
    PendulumSimulator
)

def get_config(config_type: str) -> Config:
    if config_type == "pendulum":
        return Config(
            name="PendulumConfig",
            simulator=PendulumSimulator(),
            agent=PendulumAgent()
        )
    else:
        # Add other config types here
        return Config(
            name="OtherConfig",
            simulator=OtherSimulator(),
            agent=OtherAgent()
        )
