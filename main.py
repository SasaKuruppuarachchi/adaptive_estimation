# import sim compontents
# import state
# import controller
# import estimator


class SimulationManager:
    def __init__(self):
        # self.sim = init_sim()
        # self.state = init_state()
        # self.controller = init_controller()
        # self.estimator = init_estimator()
        # self.logger = init_logger()
        # self.node = init_node()   
        
        # sim.init_callbacks(self.physics_step))
        print("Main class initialized")
        
    def physics_step(self,dt):
        self.state = self.state.update_gt(self.sim.get_gt(),dt)
        if self.node.is_new_est_available:
            self.state = self.node.get_est()  # ros2 service call from node
            action = self.controller.update(self.state)
            self.sim.apply_action(action)
        
        # self.logger.log(self.state)
        # self.node.publish(self.state)
        print("Physics step")

    def run(self):
        print("Main class running")

if __name__ == "__main__":
    main = SimulationManager()
    main.run()