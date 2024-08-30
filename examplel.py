import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        rospy.init_node('laser2pc_node')
        
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher('/laser2pc/pointcloud', PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laserCallback)

    def laserCallback(self, msg):
        cloud_out = self.laserProj.projectLaser(msg)
        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    laser2pc = Laser2PC()
    rospy.spin()