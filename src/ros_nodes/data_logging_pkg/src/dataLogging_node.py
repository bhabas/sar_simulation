import message_filters
import rospy
from threading import Thread

from gazebo_communication_pkg.msg import GlobalState
# from crazyflie_gazebo_sim.msg import RL_Data


def callback(global_state):
    print("Callback working")



rospy.init_node("dataLogging_node")
StateSub = message_filters.Subscriber("/global_state",GlobalState)
# RLSub = message_filters.Subscriber("/rl_data",RL_Data)

ats = message_filters.ApproximateTimeSynchronizer([StateSub],queue_size=20,slop=0.1)
ats.registerCallback(callback)
print("stug")
rospy.spin()
