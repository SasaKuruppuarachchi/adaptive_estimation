import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
        
