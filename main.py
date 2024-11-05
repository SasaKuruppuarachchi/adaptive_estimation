# import controller
# import estimator
from inpsimulator.isaac_simulator.launch_sim import InvertedPendulumApp
from inpsimulator.com_interface.cominterface_ros2 import ComInterfaceROS2
from inpsimulator.state.pendulum_state import PndulumState
# import logger


class SimulationManager:
    def __init__(self):
        self.sim = InvertedPendulumApp() # issaac sim version
        self.state = PndulumState()
        # self.controller = init_controller()
        # self.estimator = init_estimator()
        # self.logger = init_logger()
        self.node = ComInterfaceROS2()   
        
        # sim.init_callbacks(self.physics_step))
        self.sim._world.add_physics_callback(self.sim.pendulum._stage_prefix + "/sim_step", callback_fn=self.physics_step)
        
        print("Main class initialized")
        
    def physics_step(self,dt):
        current_sim_time = self.sim.simulation_context.current_time
        self.node.publish_clock(current_sim_time)
        print(self.sim.pendulum._state.theta_dot1)
        #self.state = self.state.update_gt(self.sim.get_gt(),dt)
        #if self.node.is_new_est_available:
        #    self.state = self.node.get_est()  # ros2 service call from node
        #    action = self.controller.update(self.state)
        #    self.sim.apply_action(action)
        
        # self.logger.log(self.state)
        # self.node.publish(self.state)
        print("Physics step")

    def run(self):
        self.sim.run()
        print("Main class running")

if __name__ == "__main__":
    main = SimulationManager()
    main.run()