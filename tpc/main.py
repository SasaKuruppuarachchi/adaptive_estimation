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
from tpc.utils.utils import init_sim, States

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
    states: States = inits['states']

    sim.start()

    # while not sim.done:
    for i in range(states.num_time_steps):
        time = perf_counter()

        sim.step()
        states.states[i] = sim.state
        states.observations[i] = sim.observation
        states.sim_dt[i] = sim.dt

        cprint(f"GymnasiumSimulator observation\n:", 'red')
        # cprint(f"theta: {180/np.pi*sim.observation[0]}", 'red')
        # cprint(f"theta_dot: {180/np.pi*sim.observation[1]}", 'red')
        cprint(f"theta: {180/np.pi*states.observations[i][0]}", 'red')
        cprint(f"theta_dot: {180/np.pi*states.observations[i][1]}", 'red')

        # Communicate states to agents
        for agent in agents:
            agent.step(observation=states.observations[i])
            states.agents_state_estimates[agent.name][i] = agent.state
            states.agents_observation_estimates[agent.name][i] = agent.observation_estimate
            states.agents_actions[agent.name][i] = agent.action

            cprint(f"Agent {agent.name} PendulumAgent observation:\n", 'cyan')
            cprint(
                f"    theta: "
                f"{180/np.pi*states.agents_observation_estimates[agent.name][i][0]}",
                'cyan')
            cprint(
                f"    theta_dot: "
                f"{180/np.pi*states.agents_observation_estimates[agent.name][i][1]}", 
                'cyan')

        viz.step(
            observation = sim.observation,
            # observation_estimate = agent.observation_estimate,
            observation_estimate = np.concatenate([agent.observation_estimate[None,...] for agent in agents], axis=0),
        )

if __name__ == "__main__":
    main()
