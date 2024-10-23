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
import rclpy

from tpc.simulator import Simulator
from tpc.agent import Agent
from tpc.visualiser.visualiser import GymnasiumPendulumVisualizer as Visualizer

from tpc.utils.config import get_config
from tpc.utils.utils import init_sim, States
from tpc.utils.types import CommunicationTypes

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

    # if config.communication_type == CommunicationTypes.ROS:
    #     import rclpy
    #     from threading import Thread
    #     # Spin the service nodes (agents)
    #     agent_threads = []
    #     for agent in agents:
    #         thread = Thread(target=rclpy.spin, args=(agent.server,))
    #         thread.start()
    #         agent_threads.append(thread)

    sim.start()

    # while not sim.done:
    for i in range(states.num_time_steps):
        time = perf_counter()

        # Step simulator.
        # Sim will request an action from the agents. Thus we can also update the agent states.
        sim.step()

        if config.communication_type == CommunicationTypes.ROS:
            # raise NotImplementedError("STUCK HERE MAIN THREAD HANGS")
            rclpy.spin_once(sim.clients[0], timeout_sec=0.01)
            # inits['server_executor'].spin_once(timeout_sec=0.1)
            # Spin the client intermittently to process the async response (if using call_async)

        states.states[i] = sim.state
        states.observations[i] = sim.observation
        states.sim_dt[i] = sim.dt

        # cprint(f"GymnasiumSimulator observation\n:", 'red')
        # cprint(f"theta: {180/np.pi*states.observations[i][0]}", 'red')
        # cprint(f"theta_dot: {180/np.pi*states.observations[i][1]}", 'red')

        # Communicate states to agents
        for agent in agents:
            # agent.step(observation=states.observations[i])
            states.agents_state_estimates[agent.name][i] = agent.state
            states.agents_observation_estimates[agent.name][i] = agent.observation_estimate
            states.agents_actions[agent.name][i] = agent.action

            # cprint(f"Agent {agent.name} PendulumAgent observation:\n", 'cyan')
            # cprint(
            #     f"    theta: "
            #     f"{180/np.pi*states.agents_observation_estimates[agent.name][i][0]}",
            #     'cyan')
            # cprint(
            #     f"    theta_dot: "
            #     f"{180/np.pi*states.agents_observation_estimates[agent.name][i][1]}", 
            #     'cyan')

        viz.step(
            observation = sim.observation,
            # observation_estimate = agent.observation_estimate,
            observation_estimate = np.concatenate([agent.observation_estimate[None,...] for agent in agents], axis=0),
        )

    viz.save_animation()

    if config.communication_type == CommunicationTypes.ROS:
        # Clean up: shutdown ROS and join threads
        rclpy.shutdown()
        # for thread in agent_threads:
        #     thread.join()

if __name__ == "__main__":
    main()
