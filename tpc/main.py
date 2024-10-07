from time import perf_counter, sleep
import logging
log = logging.getLogger(__name__)

from typing import List, Dict, Union, Optional
from omegaconf import DictConfig, ListConfig
import omegaconf as oc

from tpc.simulator import Simulator, PendulumSimulator
from tpc.agent import Agent, PendulumAgent
# from tpc.visualiser import Visualiser, PendulumVisualiser

from tpc.config import Config

def init_sim(config: Config) -> Dict:
    sim: PendulumSimulator = PendulumSimulator(config=config.simulator)

    agent: PendulumAgent = PendulumAgent(config=config.agent)

    visualiser: PendulumVisualiser = PendulumVisualiser(
                                        simulator=sim, agent=agent,
                                        config=config)

    return { 'simulator': sim, 'agent': agent, 'visualiser': visualiser }


def main():

    config: Config = oc.OmegaConf.load("config.yaml")

    inits = init_sim(config)
    sim: PendulumSimulator = inits['simulator']
    agent: PendulumAgent = inits['agent']
    visualiser: PendulumVisualiser = inits['visualiser']

    agent.attach(simulator=sim)
    sim.start()

    while not sim.done:
        time = perf_counter()
        visualiser.update()
        sim.step()

        log.info(f"Step {sim.step_count}, {perf_counter() - time:.2f} seconds")

        sleep(config.simulation.step_time)
