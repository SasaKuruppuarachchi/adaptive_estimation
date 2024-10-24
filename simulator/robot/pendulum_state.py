import numpy as np
from scipy.spatial.transform import Rotation

from .state import State

class PndulumState(State):
    """
    Stores the state of a given vehicle.
    
    Note:
        from base class
        - position - A numpy array with the [x,y,z] of the vehicle expressed in the inertial frame according to an ENU convention.
        - orientation - A numpy array with the quaternion [qx, qy, qz, qw] that encodes the attitude of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the ENU inertial frame.
        - linear_velocity - A numpy array with [vx,vy,vz] that defines the velocity of the vehicle expressed in the inertial frame according to an ENU convention.
        - linear_body_velocity - A numpy array with [u,v,w] that defines the velocity of the vehicle expressed in the FLU body frame.
        - angular_velocity - A numpy array with [p,q,r] with the angular velocity of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the FLU body frame.
        - linear acceleration - An array with [x_ddot, y_ddot, z_ddot] with the acceleration of the vehicle expressed in the inertial frame according to an ENU convention.
        
        Special
        - theta1 - Orientation of the pendulum joint0 (Servo_joint) in  numpy array [qx, qy, qz, qw]
        - theta2 - Orientation of the pendulum joint (pendulum_joint) in numpy array  [qx, qy, qz, qw]
        - theta_dot1
        - theta_dot2
    """

    def __init__(self):
        """
        Initialize the State object
        """
        super().__init__()
        # @todo fill the pendulum state descriptions
        
        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.tip_position = np.array([0.0, 0.0, 0.0])
        
        # Orientation of the pendulum joint0 (Servo_joint) in  numpy array [qx, qy, qz, qw]
        self.theta0 = np.array([0.0, 0.0, 0.0, 1.0])

        # Orientation of the pendulum joint (pendulum_joint) in numpy array  [qx, qy, qz, qw]
        self.theta1 = np.array([0.0, 0.0, 0.0, 1.0])

        # 
        self.theta_dot0 = np.array([0.0, 0.0, 0.0])

        # 
        self.theta_dot1 = np.array([0.0, 0.0, 0.0])

