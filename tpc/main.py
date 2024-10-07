import sys
from time import perf_counter, sleep
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(filename='logs/main.log', encoding='utf-8', level=logging.DEBUG)

from typing import List, Dict, Union, Optional
from omegaconf import DictConfig, ListConfig
from omegaconf import OmegaConf as oc

from tpc.simulator import Simulator
from tpc.agent import Agent
# from tpc.visualiser import Visualiser, PendulumVisualiser

from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
from tpc.agent.pendulum_agent import PendulumAgent

from tpc.config.root import Config

def init_sim(config: Union[DictConfig, ListConfig]) -> Dict:
    sim: GymnasiumSimulator = GymnasiumSimulator(**config.simulator)

    agent: PendulumAgent = PendulumAgent(**config.agent)

    # visualiser: PendulumVisualiser = PendulumVisualiser(
    #                                     simulator=sim, agent=agent,
    #                                     config=config)
    visualiser = None

    return { 'simulator': sim, 'agent': agent, 'visualiser': visualiser }


def main():

    # config_path: str = sys.argv[1] if len(sys.argv) > 1 else "src/tpc/config/experiment/default.yaml"
    config_path: str = sys.argv[1] if len(sys.argv) > 1 else "src/tpc/config/experiment/gymnasium.yaml"
    yaml_config: Union[DictConfig, ListConfig] = oc.load(config_path)

    # config: Config = oc.load("config.yaml")
    # config: DictConfig = oc.structured(Config)

    structured_conf = oc.structured(Config)
    config: Union[DictConfig,ListConfig] = oc.merge(structured_conf, yaml_config)

    import pdb; pdb.set_trace()
    inits = init_sim(config)
    sim: GymnasiumSimulator = inits['simulator']
    agent: PendulumAgent = inits['agent']
    visualiser: PendulumVisualiser = inits['visualiser']

    agent.attach(simulator=sim)
    sim.start()

    # while not sim.done:
    for i in range(3):
        time = perf_counter()
        visualiser.update()
        sim.step()

        logger.info(f"Step {sim.step_count}, {perf_counter() - time:.2f} seconds")

        sleep(config.simulation.step_time)

if __name__ == "__main__":
    main()
