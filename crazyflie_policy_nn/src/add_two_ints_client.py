#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from crazyflie_msgs.srv import Policy_Values,AddTwoIntsResponse

def add_two_ints_client(x,y,z):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', Policy_Values)
        resp1 = add_two_ints(x, y,z)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# def usage():
#     return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 3:
    #     x = int(sys.argv[1])
    #     y = int(sys.argv[2])
    # else:
    #     print(usage())
    #     sys.exit(1)


    x,y,z = -3.3781,  1.1145, -0.9872

    print("Requesting %s+%s"%(x, y))
    print(add_two_ints_client(x,y,z))