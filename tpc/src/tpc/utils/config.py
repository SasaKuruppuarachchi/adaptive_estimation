from typing import List, Dict, Union, Optional
from pathlib import Path

from loguru import logger

from omegaconf import DictConfig, ListConfig
from omegaconf import OmegaConf as oc

from tpc.config.root import Config
from tpc.config.agent.pendulum_agent import (
        PendulumAgentConfig,
        tPCPendulumAgentConfig,
        KFPendulumAgentConfig
)
from tpc.config.simulator.gymnasium_simulator import GymnasiumSimulator as GymnasiumSimulatorConfig

SIMULATOR_CONFIGS = {
    'GymnasiumSimulator': GymnasiumSimulatorConfig,
}

AGENT_CONFIGS = {
    'tPCPendulumAgent': tPCPendulumAgentConfig,
    'KFPendulumAgent': KFPendulumAgentConfig

}

def get_config(config_path: Path) -> Union[DictConfig,ListConfig]:
    yaml_config: Union[DictConfig, ListConfig] = oc.load(config_path)

    ### Load structured configs
    simulator_config_class = SIMULATOR_CONFIGS[yaml_config.simulator.name]
    # simulator_config = simulator_config_class(**yaml_config.simulator)

    # Create empty structured instance and merge with loaded values
    structured_simulator_config = oc.structured(simulator_config_class)
    simulator_config = oc.merge(structured_simulator_config, yaml_config.simulator)

    agents_configs: DictConfig = DictConfig({})
    for agent_key, agent_config in yaml_config.agents.items():
        agent_config_class = AGENT_CONFIGS[agent_config.type]
        structured_agent_config = oc.structured(agent_config_class)

        agent_config = oc.merge(structured_agent_config, agent_config)
        # agent_config = agent_config_class(**agent_config)
        # agents_configs.append(agent_config)
        agents_configs[agent_key] = agent_config

    structured_root_config: Config = oc.structured(Config(
        name=yaml_config.name,
        simulator=simulator_config,
        agents=agents_configs
    ))
    config: Union[DictConfig, ListConfig] = oc.merge(structured_root_config, yaml_config)

    return config
