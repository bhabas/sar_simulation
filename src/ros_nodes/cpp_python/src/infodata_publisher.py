#! /usr/bin/env python3
# Working example for cpp subscriber and python publisher w/ custom message and array of messages
# https://www.youtube.com/watch?v=ERz2YXAjJ88&list=PLK0b4e05LnzbrLrLhOhSvLdaQZOsJl59N&index=17
import rospy
from cpp_python.msg import InfoData, InfoDataArray

rospy.init_node('infodata_publisher_node')
pub = rospy.Publisher('/infodata',InfoDataArray,queue_size=1)
rate = rospy.Rate(10)

infodata = InfoData()
infodata_array = InfoDataArray()
i = 0


while not rospy.is_shutdown():
    i += 1

    infodata.num = 5 + i
    infodata.color = 'red'
    infodata.name = 'data1'

    for x in range(5):
        infodata_array.infos.append(infodata)
    pub.publish(infodata_array.infos)
    rate.sleep()