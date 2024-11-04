import numpy as np
from scipy.spatial.transform import Rotation


class State:
    """
    Stores the state of a given vehicle.
    
    Note:
        - position - A numpy array with the [x,y,z] of the vehicle expressed in the inertial frame according to an ENU convention.
        - orientation - A numpy array with the quaternion [qx, qy, qz, qw] that encodes the attitude of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the ENU inertial frame.
        - linear_velocity - A numpy array with [vx,vy,vz] that defines the velocity of the vehicle expressed in the inertial frame according to an ENU convention.
        - linear_body_velocity - A numpy array with [u,v,w] that defines the velocity of the vehicle expressed in the FLU body frame.
        - angular_velocity - A numpy array with [p,q,r] with the angular velocity of the vehicle's FLU body frame, relative to an ENU inertial frame, expressed in the FLU body frame.
        - linear acceleration - An array with [x_ddot, y_ddot, z_ddot] with the acceleration of the vehicle expressed in the inertial frame according to an ENU convention.
    """

    def __init__(self):
        """
        Initialize the State object
        """

        # The position [x,y,z] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.position = np.array([0.0, 0.0, 0.0])

        # The attitude (orientation) of the vehicle's body frame relative to the inertial frame of reference,
        # expressed in the inertial frame. This quaternion should follow the convention [qx, qy, qz, qw], such that "no rotation"
        # equates to the quaternion=[0, 0, 0, 1]
        self.attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # The linear velocity [u,v,w] of the vehicle's body frame expressed in the body frame of reference
        self.linear_body_velocity = np.array([0.0, 0.0, 0.0])

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        self.linear_velocity = np.array([0.0, 0.0, 0.0])

        # The angular velocity [wx, wy, wz] of the vehicle's body frame relative to the inertial frame, expressed in the body frame
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # The linear acceleration [ax, ay, az] of the vehicle's body frame relative to the inertial frame, expressed in the inertial frame
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])

