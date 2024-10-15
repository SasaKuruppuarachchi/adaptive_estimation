from typing import Dict

import numpy as np
from loguru import logger

from tpc.utils.types import ControlTypes, ControlType
from simple_pid.PID import PID

def get_controller(control_type: ControlTypes, controller_args: Dict) -> ControlType:
    if control_type == ControlTypes.PID:
        controller: ControlType = PID(
            **controller_args
        )
    elif control_type == ControlTypes.RANDOM:
        controller: ControlType = lambda x: np.random.normal(0, 1, 1)
    elif control_type == ControlTypes.LQR:
        raise NotImplementedError("LQR not implemented yet")
    elif control_type == ControlTypes.NONE:
        controller: ControlType = lambda x: 0
    else:
        import pdb; pdb.set_trace()
        logger.error(
            f'Invalid control type: {control_type}. Only "PID", "Random" or "LQR" allowed.\n'
            f"Got {control_type}."
        )
        raise KeyError()

    return controller
