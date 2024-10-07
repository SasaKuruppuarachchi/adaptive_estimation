from pathlib import Path
import sys
from time import perf_counter, sleep
# import logging
# logger = logging.getLogger(__name__)
# logging.basicConfig(filename='logs/main.log', encoding='utf-8', level=logging.DEBUG)
from loguru import logger
logger.add("logs/main.log", rotation="500 MB")

from typing import List, Dict, Union, Optional
from omegaconf import DictConfig, ListConfig
from omegaconf import OmegaConf as oc

from tpc.simulator import Simulator
from tpc.agent import Agent
# from tpc.visualiser import Visualiser, PendulumVisualiser

from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
from tpc.agent.pendulum_agent import PendulumAgent

from tpc.config.root import Config
from tpc.config.agent.pendulum_agent import PendulumAgent as PendulumAgentConfig
from tpc.config.simulator.gymnasium_simulator import GymnasiumSimulator as GymnasiumSimulatorConfig

SIMULATOR_CONFIGS = {
    'GymnasiumSimulator': GymnasiumSimulatorConfig,
}

AGENT_CONFIGS = {
    'PendulumAgent': PendulumAgentConfig
}

def init_sim(config: Union[DictConfig, ListConfig]) -> Dict:
    sim: GymnasiumSimulator = GymnasiumSimulator(**config.simulator)

    state_shape = sim.state_shape
    action_shape = sim.action_shape
    agent: PendulumAgent = PendulumAgent(
        state_dim=state_shape,
        observation_dim=state_shape,
        action_dim=action_shape,
        **config.agent)

    # visualiser: PendulumVisualiser = PendulumVisualiser(
    #                                     simulator=sim, agent=agent,
    #                                     config=config)
    visualiser = None

    return { 'simulator': sim, 'agent': agent, 'visualiser': visualiser }


def main():

    ### Load config overrides
    experiment_name: str  = sys.argv[1] if len(sys.argv) > 1 else "default"
    config_path: Path = Path(f"src/tpc/config/experiment/{experiment_name}.yaml")
    yaml_config: Union[DictConfig, ListConfig] = oc.load(config_path)

    # Dynamically instantiate the correct simulator based on the YAML
    simulator_config_class = SIMULATOR_CONFIGS[yaml_config.simulator.name]
    simulator_config = simulator_config_class(**yaml_config.simulator)

    agent_config_class = AGENT_CONFIGS[yaml_config.agent.name]
    agent_config = agent_config_class(**yaml_config.agent)
    config_: Config = Config(
        name=yaml_config.name,
        simulator=simulator_config,
        agent=agent_config
    )


    structured_conf = oc.structured(config_)
    config: Union[DictConfig,ListConfig] = oc.merge(structured_conf, yaml_config)
    logger.info(f"Config: {config}")


    ### Init simulator and agent
    inits = init_sim(config)
    sim: GymnasiumSimulator = inits['simulator']
    agent: PendulumAgent = inits['agent']
    # visualiser: PendulumVisualiser = inits['visualiser']

    agent.attach(simulator=sim)
    sim.start()

    # while not sim.done:
    for i in range(100):
        time = perf_counter()
        # visualiser.update()
        sim.step()

        logger.info(f"Step {i}, {perf_counter() - time:.2f} seconds")

        sleep(config.simulator.dt)

if __name__ == "__main__":
    main()
