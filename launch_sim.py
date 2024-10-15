from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")


from omni.isaac.core.utils import nucleus, stage
from omni.isaac.core.utils.stage import clear_stage, create_new_stage_async, update_stage_async, create_new_stage
import carb
import asyncio
import omni
from scipy.spatial.transform import Rotation
from omni.isaac.core import SimulationContext

from simulator.params import SIMULATION_ENVIRONMENTS, ROBOTS
from simulator.robot.pendulum import Pendulum

import rclpy
#from geometry_msgs.msg import 
from cominterface_ros2 import ComInterfaceROS2
        
class InvertedPendulumApp:
    def __init__(self):
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()
        self.assets_root_path = nucleus.get_assets_root_path()
        
        #setup world
        self.phy_dt = 300.0
        self.pub_dt = 100.0 # HZ = 1/dt
        self._world_settings = {"physics_dt": 1.0 / self.phy_dt, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 30.0}
        self._world = World(**self._world_settings)
        
        # Load environment
        self.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        
        # add robot
        self.pendulum = Pendulum(
            "/World/qservo",
            ROBOTS['qServo'],
            [0.0, 0.0, 0.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            self._world,
        ) #  orientation [w, x, y, z]
        
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        print(ROBOTS['qServo'])
        self._world.reset()
        self.stage = omni.usd.get_context().get_stage()
        self.pendulum_prim = self._world.stage.GetPrimAtPath(self.pendulum._stage_prefix + "/pendulum")
        
        #self.simulation_context = SimulationContext(physics_dt=1.0 / self.phy_dt, rendering_dt=1.0 / 30.0, stage_units_in_meters=1.0)

        
        # Initialize ROS 2
        rclpy.init()

        # Create ROS 2 publisher node
        self.node = ComInterfaceROS2()
        
        self.stop_sim = False
        
        self._world.add_physics_callback(self.pendulum._stage_prefix + "/sim_step", callback_fn=self.physics_step)
        
    def physics_step(self, step_size):
        print(Rotation.from_quat(self.pendulum._state.attitude).as_rotvec())
        
        #self.node.time_publisher.publish()
        return
        
    async def load_environment_async(self, usd_path: str, force_clear: bool=False):
        """Method that loads a given world (specified in the usd_path) into the simulator asynchronously.

        Args:
            usd_path (str): The path where the USD file describing the world is located.
            force_clear (bool): Whether to perform a clear before loading the asset. Defaults to False. 
            It should be set to True only if the method is invoked from an App (GUI mode).
        """

        # Reset and pause the world simulation (only if force_clear is true)
        # This is done to maximize the support between running in GUI as extension vs App
        if force_clear == True:

            # Create a new stage and initialize (or re-initialized) the world
            await create_new_stage_async()
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self._world = World.instance()

            await self._world.reset_async()
            await self._world.stop_async()

        # Load the USD asset that will be used for the environment
        try:
            self.load_asset(usd_path, "/World/layout")
        except Exception as e:
            carb.log_warn("Could not load the desired environment: " + str(e))

        carb.log_info("A new environment has been loaded successfully")
        
    def load_asset(self, usd_asset: str, stage_prefix: str):
        """
        Method that will attempt to load an asset into the current simulation world, given the USD asset path.

        Args:
            usd_asset (str): The path where the USD file describing the world is located.
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. 
        """

        # Try to check if there is already a prim with the same stage prefix in the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")

        # Create the stage primitive and load the usd into it
        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_asset)

        if not success:
            raise Exception("The usd asset" + usd_asset + "is not load at stage path " + stage_prefix)

    def load_environment(self, usd_path: str, force_clear: bool=False):
        """Method that loads a given world (specified in the usd_path) into the simulator. If invoked from a python app,
        this method should have force_clear=False, as the world reset and stop are performed asynchronously by this method, 
        and when we are operating in App mode, we want everything to run in sync.

        Args:
            usd_path (str): The path where the USD file describing the world is located.
            force_clear (bool): Whether to perform a clear before loading the asset. Defaults to False.
        """
        asyncio.ensure_future(self.load_environment_async(usd_path, force_clear))
        
    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()
        #self.simulation_context.play()
        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            simulation_app.update()
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        #self.simulation_context.stop()
        #self.timeline.stop()
        simulation_app.close()

def main():
    # Instantiate the template app
    pendulum_app = InvertedPendulumApp()

    # Run the application loop
    pendulum_app.run()

if __name__ == "__main__":
    main()