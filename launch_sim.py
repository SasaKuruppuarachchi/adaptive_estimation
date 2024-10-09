from omni.isaac.kit import SimulationApp
app = SimulationApp({"headless": False})

from omni.isaac.core import World
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from omni.isaac.core.utils import nucleus, stage
from omni.isaac.core.utils.stage import clear_stage, create_new_stage_async, update_stage_async, create_new_stage
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.usd import get_stage_next_free_path
from omni.isaac.core.robots.robot import Robot
import carb
import asyncio
import omni
from scipy.spatial.transform import Rotation
from omni.isaac.core import World
from simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from simulator.logic.interface.sim_interface import SimInterface


class ComInterfaceROS2(Node):
    def __init__(self):
        super().__init__('drone_location_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.time_publisher = self.create_publisher(Clock, 'clock', qos_profile)
        
        # subscribe to controls
        
class Pendulum(Robot):
    def __init__(
        self,
        stage_prefix: str,
        usd_path: str = None,
        init_pos=[0.0, 0.0, 0.0],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
    ):
        """
        Class that initializes a vehicle in the isaac sim's curent stage

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_path (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.0].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = InvertedPendulumApp().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(self._current_stage, stage_prefix, False)
        self._usd_file = usd_path

        # Get the vehicle name by taking the last part of vehicle stage prefix
        self._vehicle_name = self._stage_prefix.rpartition("/")[-1]

        # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim = get_prim_at_path(self._stage_prefix)
        self._prim.GetReferences().AddReference(self._usd_file)

        # Initialize the "Robot" class
        # Note: we need to change the rotation to have qw first, because NVidia
        # does not keep a standard of quaternions inside its own libraries (not good, but okay)
        super().__init__(
            prim_path=self._stage_prefix,
            name=self._stage_prefix,
            position=init_pos,
            orientation=[init_orientation[3], init_orientation[0], init_orientation[1], init_orientation[2]],
            articulation_controller=None,
        )

        # Add this object for the world to track, so that if we clear the world, this object is deleted from memory and
        # as a consequence, from the VehicleManager as well
        self._world.scene.add(self)
        
class InvertedPendulumApp:
    def __init__(self):
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()
        self.assets_root_path = nucleus.get_assets_root_path()
        
        self.interface = SimInterface()
        
        #setup world
        self.phy_dt = 300.0
        self.pub_dt = 100.0 # HZ = 1/dt
        self._world_settings = {"physics_dt": 1.0 / self.phy_dt, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 30.0}
        self._world = World(**self._world_settings)
        
        # Load environment
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        
        # add robot
        self.pendulum = Pendulum(
            "/World/drone0",
            ROBOTS['Agipix v2'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        ) #  orientation [w, x, y, z]
        self._world.scene.add(self.pendulum)
        
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()
        self.drone_prim = self.stage.GetPrimAtPath("/World/drone0/body")

        
        # Initialize ROS 2
        rclpy.init()

        # Create ROS 2 publisher node
        self.node = ComInterfaceROS2()
        
    