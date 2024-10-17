from typing import List, Dict, Union, Optional
from pathlib import Path
import sys
from time import perf_counter, sleep

from loguru import logger
logger.add("logs/main.log", rotation="500 MB")

from termcolor import colored, cprint
from omegaconf import DictConfig, ListConfig
from omegaconf import OmegaConf as oc
import numpy as np

from tpc.simulator import Simulator
from tpc.agent import Agent
from tpc.visualiser.visualiser import GymnasiumPendulumVisualizer as Visualizer

from tpc.utils.config import get_config
from tpc.utils.utils import init_sim

def main():

    ### Load config overrides
    experiment_name: str  = sys.argv[1] if len(sys.argv) > 1 else "gymnasium_pendulum"
    config_path: Path = Path(f"src/tpc/config/experiment/{experiment_name}.yaml")

    config: Union[DictConfig,ListConfig] = get_config(config_path)

    logger.info(f"Config: {config}")

    rng: np.random.Generator = np.random.default_rng(config.seed)

    ### Init simulator and agent
    inits = init_sim(config, rng=rng)
    sim: Simulator = inits['simulator']
    agents: List[Agent] = inits['agents']
    viz: Visualizer = inits['visualiser']
    # visualiser: PendulumVisualiser = inits['visualiser']

    sim.start()

    # while not sim.done:
    for i in range(1000):
        time = perf_counter()
        # visualiser.update()
        sim.step()

        # logger.info(f"Step {i}, {perf_counter() - time:.2f} seconds")
        cprint(f"Step {i}, {perf_counter() - time:.2f} seconds", 'green')

        # State
        # cprint(f"GymnasiumSimulator state: {sim.state}", 'red')
        # cprint(f"PendulumAgent state: {agent.state}", 'cyan')

        # Observation
        # cprint(f"GymnasiumSimulator observation: {(360/np.pi)*sim.observation)}", 'red')
        cprint(f"GymnasiumSimulator observation\n:", 'red')
        cprint(f"theta: {180/np.pi*sim.observation[0]}", 'red')
        cprint(f"theta_dot: {180/np.pi*sim.observation[1]}", 'red')
        for agent in agents:
            cprint(f"Agent {agent.name} PendulumAgent observation\n:", 'cyan')
            cprint(f"    theta: {180/np.pi*agent.observation_estimate[0]}", 'cyan')
            cprint(f"    theta_dot: {180/np.pi*agent.observation_estimate[1]}", 'cyan')

        # cprint(f"PendulumAgent action: {agent.action}", 'yellow')

        viz.step(
            observation = sim.observation,
            # observation_estimate = agent.observation_estimate,
            observation_estimate = np.concatenate([agent.observation_estimate[None,...] for agent in agents], axis=0),
        )
        sleep(config.simulator.dt)
        # input("Press Enter to continue...")

if __name__ == "__main__":
    main()
