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
        self.dim_x = 2 # State Dimension
        self.dim_z = 1 # Measurement Dimension

        self.dt = 0
        self.t_prev = 0

        # Define sigma points
        points = MerweScaledSigmaPoints(self.dim_x, alpha=0.1, beta=2., kappa=1.)

        # Initialize UKF
        self.ukf = UKF(dim_x=self.dim_x, dim_z=self.dim_z, fx=self.fx, hx=self.hx, dt=self.dt, points=points)

        # Initial State Estimate
        self.ukf.x = np.array([0., 0.]) # Initial State

        # Initial Covariance Estimate
        self.ukf.P *= 10 # Initial Covariance

        # Process Noise Covariance
        self.ukf.Q = np.diag([0.01, 0.01])

        # Measurement Noise Covariance
        self.ukf.R = np.diag([0.05])

        

        # Subscribers and Publishers
        self.measurement_sub = rospy.Subscriber("/LandingTargets", LandingTargetArray, self.update_ukf)
        self.filtered_pub = rospy.Publisher("/LandingTargets_Filtered", LandingTargetArray, queue_size=10)



        

        

    def fx(self, x, dt):
        # State transition function
        F = np.array([[1, dt],
                      [0, 1]])
        return np.dot(F, x)
        
    def hx(self,x):
        
        # Measurement function
        return np.array([x[0]])

    def update_ukf(self, LandingSurfaces_msg):

        if not LandingSurfaces_msg.LandingTargets:
            return
        
        self.dt = LandingSurfaces_msg.header.stamp.to_sec() - self.t_prev
        self.t_prev = LandingSurfaces_msg.header.stamp.to_sec()

        print("Time Step: ", self.dt)
        
        # Assuming single target for now
        target = LandingSurfaces_msg.LandingTargets[0]

        # Extract current position
        p_x_cam = target.Pose_Centroid_Cam_body.position.x
        p_y_cam = target.Pose_Centroid_Cam_body.position.y
        p_z_cam = target.Pose_Centroid_Cam_body.position.z

        # Extract current position from LiDAR
        p_x_lidar = target.Pose_Centroid_Lidar_body.position.x
        p_y_lidar = target.Pose_Centroid_Lidar_body.position.y
        p_z_lidar = target.Pose_Centroid_Lidar_body.position.z

        # Create the measurement vector z, including both position 
        z = np.array([
            p_x_lidar
            ])

        
        print()
        
        # Perform UKF update
        self.ukf.predict()
        self.ukf.update(z)

        # Publish Fused State
        fused_target = LandingTarget()
        fused_target.Pose_Centroid_Filtered_body.position.x = self.ukf.x[0]
        # fused_target.Pose_Centroid_Filtered_body.position.y = self.ukf.x[1]
        # fused_target.Pose_Centroid_Filtered_body.position.z = self.ukf.x[2]

        # fused_target.Twist_Centroid.linear.x = self.ukf.x[3]
        # fused_target.Twist_Centroid.linear.y = self.ukf.x[4]
        # fused_target.Twist_Centroid.linear.z = self.ukf.x[5]

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
