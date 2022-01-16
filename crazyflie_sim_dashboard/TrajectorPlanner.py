#!/usr/bin/env python3

import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from dashboard_node import DashboardNode
from nav_msgs.msg import Odometry
import os,rospkg
import rospy

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)

class Traj_Planner(QMainWindow,DashboardNode):
    def __init__(self):
        super().__init__()
        super(DashboardNode).__init__()

        #LOAD UI
        loadUi(f'{BASE_PATH}/crazyflie_sim_dashboard/Trajectory_Planner.ui', self)

        self.printSumthin()


if __name__ == '__main__':

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Traj_Planner()
    myApp.show()

    sys.exit(app.exec())