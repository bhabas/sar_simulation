#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
import pyqtgraph as pg

import os,rospkg
import numpy as np
import pandas as pd
import sys  # We need sys so that we can pass argv to QApplication
from dashboard_node import DashboardNode

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)


class Dashboard(QMainWindow,DashboardNode):
    def __init__(self):
        super().__init__()
        super(DashboardNode).__init__()

        #LOAD UI
        loadUi(f'{BASE_PATH}/crazyflie_sim_dashboard/DashboardWindow.ui', self)
        
        self.printSumthin()


if __name__ == "__main__":

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Dashboard()
    myApp.show()

    sys.exit(app.exec_())