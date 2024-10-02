from time import perf_counter, sleep
import logging
log = logging.getLogger(__name__)

from typing import List, Dict
from omegaconf import DictConfig
import hydra

from tpc.simulator import Simulator, PendulumSimulator
from tpc.agent import Agent, PendulumAgent
from tpc.visualiser import Visualiser, PendulumVisualiser


def init_sim(config: DictConfig) -> Dict
    sim: PendulumSimulator = PendulumSimulator(config=config.simulator)

    agent: PendulumAgent = PendulumAgent(config=config.agent)

    visualiser: PendulumVisualiser = PendulumVisualiser(
                                        simulator=sim, agent=agent,
                                        config=config)

    return { 'simulator': sim, 'agent': agent, 'visualiser': visualiser }


@hydra.main(config_path="config/config.yaml")
def main(config: DictConfig):

    inits = init_sim(config)
    sim: PendulumSimulator = inits['simulator']
    agent: PendulumAgent = inits['agent']
    visualiser: PendulumVisualiser = inits['visualiser']

    agent.attach(simulator=sim)
    sim.start()

    while not sim.done:
        time = perf_counter()
        agent.step()
        visualiser.update()
        sim.step()

        log.info(f"Step {sim.step_count}, {perf_counter() - time:.2f} seconds")

        sleep(config.simulation.step_time)
