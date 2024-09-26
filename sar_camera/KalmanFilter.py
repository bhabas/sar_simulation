#!/usr/bin/env python

import rospy
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import numpy as np
from sar_msgs.msg import LandingTarget, LandingTargetArray


class UKFNode:
    def __init__(self):
        rospy.init_node('ukf_node', anonymous=True)

        # State dimension and measurement dimension
        self.n_x = 6 # State Dimension
        self.n_z = 6 # Measurement Dimension

        # Define sigma points
        sigmas= MerweScaledSigmaPoints(6, alpha=0.1, beta=2., kappa=1.)

        self.dt = 0
        self.t_prev = 0
        self.prev_position = np.array([0, 0, 0])

        # Initialize UKF
        self.ukf = UKF(dim_x=self.n_x, dim_z=self.n_z, fx=self.fx, hx=self.hx, dt=self.dt, points=sigmas)

        # Initial State and Covariance
        self.ukf.x = np.array([0, 0, 0, 0.5, 0, 0]) # Initial State
        self.ukf.P *= 10 # Initial Covariance

        # Process Noise Covariance
        self.ukf.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])

        # Measurement Noise Covariance
        self.ukf.R = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])

        # Subscribers and Publishers
        self.measurement_sub = rospy.Subscriber("/LandingTargets", LandingTargetArray, self.update_ukf)
        self.filtered_pub = rospy.Publisher("/LandingTargets_Filtered", LandingTargetArray, queue_size=10)

        

        

    def fx(self, x, dt):
        # State transition function
        F = np.array([[1, 0, 0, dt, 0, 0],
                      [0, 1, 0, 0, dt, 0],
                      [0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        return F @ x
    
    def hx(self,x):
        
        # Measurement function
        # Extract position from state vector
        p_x, p_y, p_z = x[0], x[1], x[2]

        # Extract velocity from state vector
        v_x, v_y, v_z = x[3], x[4], x[5]

        ## Future Details
        #
        #
        #

        # Combined measurement vector
        z = np.array([p_x, p_y, p_z, v_x, v_y, v_z])
        return z

    def update_ukf(self, LandingSurfaces_msg):

        if not LandingSurfaces_msg.LandingTargets:
            return
        
        self.dt = LandingSurfaces_msg.header.stamp.to_sec() - self.t_prev
        self.t_prev = LandingSurfaces_msg.header.stamp.to_sec()

        print("Time Step: ", self.dt)
        
        # Assuming single target for now
        target = LandingSurfaces_msg.LandingTargets[0]

        # Extract current position
        p_x = target.Pose_Centroid.position.x
        p_y = target.Pose_Centroid.position.y
        p_z = target.Pose_Centroid.position.z

        # Estimate velocity from position change (if not the first frame)
        if hasattr(self, 'previous_position'):
            v_x = (p_x - self.previous_position[0]) / self.dt
            v_y = (p_y - self.previous_position[1]) / self.dt
            v_z = (p_z - self.previous_position[2]) / self.dt
        else:
            v_x, v_y, v_z = 0.0, 0.0, 0.0  # Initialize velocities for the first frame


        # Store current position for next iteration
        self.previous_position = np.array([p_x, p_y, p_z])

        # Create the measurement vector z, including both position and estimated velocity
        z = np.array([p_x, p_y, p_z, v_x, v_y, v_z])

        
        print()
        
        # Perform UKF update
        self.ukf.predict()
        self.ukf.update(z)

        # Publish Fused State
        fused_target = LandingTarget()
        fused_target.Pose_Centroid.position.x = self.ukf.x[0]
        fused_target.Pose_Centroid.position.y = self.ukf.x[1]
        fused_target.Pose_Centroid.position.z = self.ukf.x[2]

        fused_target.Twist_Centroid.linear.x = self.ukf.x[3]
        fused_target.Twist_Centroid.linear.y = self.ukf.x[4]
        fused_target.Twist_Centroid.linear.z = self.ukf.x[5]

        LandingTargets = LandingTargetArray()
        LandingTargets.LandingTargets = [fused_target]
        LandingTargets.header = LandingSurfaces_msg.header


        self.filtered_pub.publish(LandingTargets)


if __name__ == '__main__':
    try:
        ukf_node = UKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
