import rclpy
import math
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped

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
        self.gt_pose_publisher = self.create_publisher(PoseStamped, 'pendulum0/gt_pose',qos_profile)
        self.theta1_publisher = self.create_publisher(TwistStamped,'pendulum0/gt_theta1',qos_profile)
        
        # subscribe to controls
        
    def publish_clock(self, sim_time):
        time_msg = Clock()
        time_msg.clock.sec = math.floor(sim_time)
        time_msg.clock.nanosec = int((sim_time - time_msg.clock.sec) * 1e9)
        self.time_publisher.publish(time_msg)
        
    def publish_theta1(self, theta1,sim_time):
        msg = TwistStamped()
        # @todo implement
        self.theta1_publisher.publish(msg)