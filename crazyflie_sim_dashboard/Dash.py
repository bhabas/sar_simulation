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

        self.resize(800,800)

        #LOAD UI
        loadUi(f'{BASE_PATH}/crazyflie_sim_dashboard/DashboardWindow.ui', self)

        self.plot_list = [
            self.Pos_Graph,
            self.Vel_Graph,
            self.PWM_Graph,
            self.Dist_Graph,
            self.Eul_Graph,
            self.OF_Graph,
            self.Omega_Graph,
            ]


        self.Battery_Voltage.setValue(50)

        ## ON BUTTON PRESS EXECUTE FUNCTIONS
        self.pauseButton.clicked.connect(self.pause_plots)
        self.rescaleButton.clicked.connect(self.rescale_plots)
        self.clearButton.clicked.connect(self.clear_plots)
        self.paused = False

        
        self.printSumthin()


    def pause_plots(self):
        if self.paused == False:

            self.paused = True
            for plot in self.plot_list:
                plot.pause(self.paused)
     
        else:
            
            self.paused = False
            for plot in self.plot_list:
                plot.pause(self.paused)



    def rescale_plots(self):
        for plot in self.plot_list:
            plot.reset_axes()

        # self.Mu_Graph.reset_axes()
        # self.Sigma_Graph.reset_axes()

    def clear_plots(self):
        for plot in self.plot_list:
            plot.clear_data()


if __name__ == "__main__":

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Dashboard()
    myApp.show()

    sys.exit(app.exec_())