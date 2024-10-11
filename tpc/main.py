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

from tpc.simulator.gymnasium_simulator import GymnasiumSimulator
from tpc.agent.pendulum_agent import PendulumAgent, tPCPendulumAgent

from tpc.config.root import Config
from tpc.config.agent.pendulum_agent import (
        PendulumAgent as PendulumAgentConfig,
        tPCPendulumAgent as tPCPendulumAgentConfig,
        KFPendulumAgent as KFPendulumAgentConfig
)

from tpc.config.simulator.gymnasium_simulator import GymnasiumSimulator as GymnasiumSimulatorConfig


SIMULATOR_CONFIGS = {
    'GymnasiumSimulator': GymnasiumSimulatorConfig,
}

AGENT_CONFIGS = {
    # 'PendulumAgent': PendulumAgentConfig,
    'tPCPendulumAgent': tPCPendulumAgentConfig,
    # 'KFPendulumAgent': KFPendulumAgentConfig
}

def init_tPC_matrices(rng: np.random.Generator, state_dim: int, action_dim: int, observation_dim: int) -> Dict:
    A = rng.normal(0, 1, (state_dim, state_dim))
    C = rng.normal(0, 1, (observation_dim, state_dim))

    B = np.zeros((state_dim, action_dim))
    B[-1, -1] = 1 

    return { 'A': A, 'B': B, 'C': C }

def init_sim(config: Union[DictConfig, ListConfig], rng: np.random.Generator) -> Dict:
    sim: GymnasiumSimulator = GymnasiumSimulator(**config.simulator)

    state_shape = sim.state_shape
    action_shape = sim.action_shape
    agent: Agent = tPCPendulumAgent(
        state_dim=state_shape,
        observation_dim=state_shape,
        action_dim=action_shape,
        **config.agent,
        **init_tPC_matrices(rng=rng, state_dim=state_shape[0], 
            action_dim=action_shape[0], observation_dim=state_shape[0]),
    )

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

    rng: np.random.Generator = np.random.default_rng(config.seed)

    ### Init simulator and agent
    inits = init_sim(config, rng=rng)
    sim: GymnasiumSimulator = inits['simulator']
    agent: PendulumAgent = inits['agent']
    # visualiser: PendulumVisualiser = inits['visualiser']

    agent.attach(simulator=sim)
    sim.start()


    viz: Visualizer = Visualizer(
            # state: np.ndarray,
            # state_estimate: np.ndarray,
            observation = sim.observation,
            observation_estimate = agent.predicted_observation,
            # state_error: np.ndarray,
            # observation_error: np.ndarray,
            # positions_x=positions_x, positions_y=positions_y
                                 )

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
        cprint(f"GymnasiumSimulator observation: {sim.observation}", 'red')
        cprint(f"PendulumAgent observation: {agent.predicted_observation}", 'cyan')

        viz.step(
            observation = sim.observation,
            observation_estimate = agent.predicted_observation,
        )
        sleep(config.simulator.dt)
        # input("Press Enter to continue...")

if __name__ == "__main__":
    main()
