#!/usr/bin/env python

import rospy
import time
import numpy as np
import xlwt
import tf
from threading import Thread
import os, subprocess, signal
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist, Vector3Stamped
# from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyTopicMsg
from gazebo_connection import GazeboConnection

from mavros_offboard import MavrosOffboard
from pymavlink import mavutil


class QuadCopterEnv:

    def __init__(self):
        self.mavros_offb = MavrosOffboard("setUp")
        self.running_step = 0.001
        self.gazebo_p = None
        

    def init_simulation(self):
        #self.gazebo_p = subprocess.Popen('~/Desktop/test.bash >/dev/null 2>&1 &', 
        #    close_fds=True, preexec_fn=os.setsid, shell=True)
        self.gazebo_p = subprocess.Popen(
            "gnome-terminal --disable-factory -e '/home/pan/catkin_ws/src/robot\ landing/4.\ rl/src/test.bash'", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        
        time.sleep(30)

        #self.mavros_offb = MavrosOffboard("setUp")
        while not self.mavros_offb.setUp():
            if self.gazebo_p is not None:
                os.killpg(self.gazebo_p.pid, signal.SIGTERM)
                time.sleep(20)
            self.gazebo_p = subprocess.Popen(
                "gnome-terminal --disable-factory -e '/home/pan/catkin_ws/src/robot\ landing/4.\ rl/src/test.bash'",
                close_fds=True, preexec_fn=os.setsid, shell=True)
            time.sleep(30)
        self.mavros_offb.wait_for_topics(30)
        self.mavros_offb.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)
        #self.mavros_offb.log_topic_vars()
        
        self.mavros_offb.set_mode("OFFBOARD", 5)
        
        self.mavros_offb.set_arm(True, 5)

        self.gazebo = GazeboConnection()


    def reset(self):
        # 1st: resets simulation
        if self.gazebo_p is not None:
            os.killpg(self.gazebo_p.pid, signal.SIGTERM)
            time.sleep(20)

        self.init_simulation()

        if not self.mavros_offb.att_thread_done or not self.mavros_offb.vel_thread_done:
            self.mavros_offb.att_thread_done = True
            self.mavros_offb.vel_thread_done = True
        
        #self.gazebo.resetSim()
        #self.gazebo.resetWorld()
        
        # 2nd: Unpauses simulation
        self.gazebo.unpauseSim()

        # 3th: takes an observation
        pose = self.take_observation()
        
        # 4th: pauses simulation
        # self.gazebo.pauseSim()

        self.reward, self.distance, self.angle = None, None, None
        
        return pose

    
    def step(self, action):

        if action['dmp'] == 1:
            vz_d  = action['vz_d']
            vx_d = action['vx_d']
            pose = self.keep_velocity(vx = vx_d, vz = vz_d)
            # self.mavros_offb.publish_velocity(vx=vx_d, vy=0, vz=vz_d)
        elif action['dmp'] == 2:
            q_d = action['q_d']
            pose = self.pitch_up(q=q_d)
            # self.mavros_offb.publish_att(p=0, q=q_d, r=0)
        else:
            print("no such action")
        
        # pose = self.get_real_pose()
        d, distance, angle = self.calculate_distance(pose)

        if self.reward is None:
            self.reward = -d
            self.distance = distance
            self.angle = angle
        else:
            #self.reward = np.max([self.reward, -d])
            if self.reward < -d:
                self.reward = -d
                self.distance = distance
                self.angle = angle * 180/np.pi
        
        if d < 0.1:
            done = True
            self.reward += 10
        else:
            done = False

        info = [self.distance, self.angle]

        return pose, self.reward, done, info


    def take_observation (self):
        position = self.mavros_offb.local_position.pose.position
        orientation_q = self.mavros_offb.local_position.pose.orientation
        vel = self.mavros_offb.local_velocity.twist.linear
        omega = self.mavros_offb.local_velocity.twist.angular

        position = [position.x, position.y, position.z]
        orientation_q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        vel = [vel.x, vel.y, vel.z]
        omega = [omega.x, omega.y, omega.z]
        
        pose = np.hstack((position, vel, orientation_q, omega))

        return pose


    def get_real_pose(self):
        position = self.mavros_offb.real_pose.pose.pose.position
        orientation_q = self.mavros_offb.real_pose.pose.pose.orientation
        vel = self.mavros_offb.real_pose.twist.twist.linear
        omega = self.mavros_offb.real_pose.twist.twist.angular

        position = [position.x, position.y, position.z]
        orientation_q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        vel = [vel.x, vel.y, vel.z]
        omega = [omega.x, omega.y, omega.z]
        
        pose = np.hstack((position, vel, orientation_q, omega))

        return pose
        
    
    def calculate_distance(self, pose):

        pos = pose[0:3]
        #vel = pose[3:6]
        orientation_q = pose[6:10]
        # omega = pose[10:13]

        pos_z = pos[2]
        
        r = R.from_quat(orientation_q)        
        r = r.as_dcm()
        z_b = r[:,2]

        distance = abs(10-0.21 - pos_z)
        angle = np.arccos( np.dot(z_b, [0,0,-1]) )
        d = distance/4.0 + angle / np.pi
        d *= 5.0
        
        return d, distance, angle

    
    def keep_velocity(self, vx, vz):
        if self.mavros_offb.vel_thread_done:
            self.mavros_offb.att_thread_done = True
            
            self.mavros_offb.vel_thread_done = False
            self.mavros_offb.vel_thread = Thread(target=self.mavros_offb.send_vel, args=())
            self.mavros_offb.vel_thread.daemon = True
            self.mavros_offb.vel_thread.start()

        # self.gazebo.unpauseSim()
        self.mavros_offb.reach_vel(vx=vx, vy=0, vz=vz, p=0, q=0, r=0)
        # time.sleep(self.running_step)
        pose = self.get_real_pose()
        # self.gazebo.pauseSim()

        return pose


    def test_pitch_up(self, q):
        if self.mavros_offb.att_thread_done:
            self.mavros_offb.vel_thread_done = True
            
            self.mavros_offb.att_thread_done = False
            self.mavros_offb.att_thread = Thread(target=self.mavros_offb.send_att, args=())
            self.mavros_offb.att_thread.daemon = True
            self.mavros_offb.att_thread.start()

        q = q * 3.14/180
        self.gazebo.unpauseSim()
        self.mavros_offb.reach_omega(p=0, q=q, r=0)
        time.sleep(self.running_step)
        pose = self.take_observation()
        self.gazebo.pauseSim()
        time.sleep(0.1)

        done = False
        if position[2]>9.7:
            done = True
        
        return pose, done

    
    def pitch_up(self, q):
        if self.mavros_offb.att_thread_done:
            self.mavros_offb.vel_thread_done = True
            
            self.mavros_offb.att_thread_done = False
            self.mavros_offb.att_thread = Thread(target=self.mavros_offb.send_att, args=())
            self.mavros_offb.att_thread.daemon = True
            self.mavros_offb.att_thread.start()

        # q = q * 3.14/180
        # self.gazebo.unpauseSim()
        self.mavros_offb.reach_omega(p=0, q=q, r=0)
        # time.sleep(self.running_step)
        pose = self.get_real_pose()   
        # self.gazebo.pauseSim()

        return pose

    def create_xls(self, start_time, sigma, alpha, file_name):
        file_log =  xlwt.Workbook()
        sheet = file_log.add_sheet('Learning process')
        sheet.write(0,0, start_time)
        sheet.write(0,1, 'sigma')
        sheet.write(0,2, sigma)
        sheet.write(0,3, 'alpha')
        sheet.write(0,4, alpha)

        sheet.write(1,0, 'Ep')
        sheet.write(1,1, 'Date')
        sheet.write(1,2, 'Time')
        sheet.write(1,3, 'Vz_d (m/s)')
        sheet.write(1,4, 'Vx_d (m/s)')
        sheet.write(1,5, 'RREV (rad/s)')
        sheet.write(1,6, 'omega_y (rad/s)')
        sheet.write(1,7, 'Action (q deg/s)')
        sheet.write(1,8, 'Reward')
        sheet.write(1,9, 'Distance (m)')
        sheet.write(1,10, 'Angle (deg)')
        sheet.write(1,11, 'Theta')
        file_log.save(file_name)

        return file_log, sheet

    
    def add_record_xls(self, file_log, sheet, file_name,
            k_ep, start_time1, start_time2,
            vz_ini, vx_ini, state, action, reward, info, theta):
        sheet.write(k_ep+2,0, k_ep+1)
        sheet.write(k_ep+2,1, start_time1)
        sheet.write(k_ep+2,2, start_time2)
        sheet.write(k_ep+2,3, np.around(vz_ini,3))
        sheet.write(k_ep+2,4, np.around(vx_ini,3))
        sheet.write(k_ep+2,5, np.around(state[0],3))
        sheet.write(k_ep+2,6, np.around(state[1],3))
        sheet.write(k_ep+2,7, np.around(action,3))
        sheet.write(k_ep+2,8, np.around(reward,3))
        sheet.write(k_ep+2,9, np.around(info[0],3))
        sheet.write(k_ep+2,10, np.around(info[1],3))
        sheet.write(k_ep+2,11, np.around(theta[0],3))
        sheet.write(k_ep+2,12, np.around(theta[1],3))
        sheet.write(k_ep+2,13, np.around(theta[2],3))
        file_log.save(file_name)
