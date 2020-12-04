import rospy
from threading import Thread

from gazebo_communication_pkg.msg import GlobalState

class DashboardSubs:
    def __init__(self):
        print("[STARTING] DashboardSubs is starting...")
        rospy.init_node("dashboard_gui_node")

        self.global_state = GlobalState
        self.global_stateThread = Thread(target=self.global_stateSub,args=())
        self.global_stateThread.daemon=True
        self.global_stateThread.start()

        print("[COMPLETED] DashboardSubs is running...")

    def global_stateSub(self):
        rospy.Subscriber('/global_state',GlobalState,self.global_stateCallback)
        rospy.spin()
    
    def global_stateCallback(self,data):
        gs_msg = data # gs_msg <= global_state_msg

        ## SET TIME VALUE FROM TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.global_pose.position
        global_quat = gs_msg.global_pose.orientation
        global_vel = gs_msg.global_twist.linear
        global_omega = gs_msg.global_twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        position = [global_pos.x,global_pos.y,global_pos.z]
        orientation_q = [global_quat.w,global_quat.x,global_quat.y,global_quat.z]
        velocity = [global_vel.x,global_vel.y,global_vel.z]
        omega = [global_omega.x,global_omega.y,global_omega.z]


        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = [t] + position + orientation_q + velocity + omega ## t (float) -> [t] (list)
        