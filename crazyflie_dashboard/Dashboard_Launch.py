#!/usr/bin/env python3
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
import pyqtgraph as pg
from PyQt5.QtGui import QPixmap

import os,rospkg,rospy
import numpy as np
import pandas as pd
import sys  # We need sys so that we can pass argv to QApplication
from Dashboard_Node import DashboardNode

from crazyflie_msgs.msg import CF_StateData
from nav_msgs.msg import Odometry

## CHECK IF SIM OR EXPERIMENT IS RUNNING
DATA_TYPE = rospy.get_param("DATA_TYPE")
if DATA_TYPE == "EXP":
    from crazyflie_msgs_exp.msg import Vicon_Filtered,CF_StateData,GenericLogData
    from geometry_msgs.msg import TransformStamped
    
from threading import Thread, Timer
import time

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

ICON_RED_LED = f'{BASE_PATH}/crazyflie_dashboard/Icons/Red_LED.png'
ICON_GREEN_LED = f'{BASE_PATH}/crazyflie_dashboard/Icons/Green_LED.png'
ICON_TASKBAR = f'{BASE_PATH}/crazyflie_dashboard/Icons/icon.png'


class Dashboard(QMainWindow,DashboardNode):
    def __init__(self):
        super().__init__()
        super(DashboardNode).__init__()
        self.setWindowIcon(QtGui.QIcon(ICON_TASKBAR))


        #LOAD UI
        loadUi(f'{BASE_PATH}/crazyflie_dashboard/DashboardWindow.ui', self)
        self.tabWidget.setCurrentIndex(0)


        self.plot_list = [
            self.Pos_Graph,
            self.Vel_Graph,
            self.PWM_Graph,
            self.Dist_Graph,
            self.Eul_Graph,
            self.Tau_Graph,
            self.OF_Graph,
            self.Omega_Graph,
            self.Thrust_Graph
            ]

        
        self.Vicon_LED.setPixmap(QPixmap(ICON_RED_LED))
        self.EmergencyStop.clicked.connect(self.Emergency_Stop)
        self.HomeButton.clicked.connect(self.HomeCmd)

        ## COMMAND BUTTON ACTIVATION
        self.Cmd_Button.clicked.connect(self.SendCmd)
        self.Cmd_Type_LineEdit.returnPressed.connect(self.SendCmd)
        self.Cmd_X_LineEdit.returnPressed.connect(self.SendCmd)
        self.Cmd_Y_LineEdit.returnPressed.connect(self.SendCmd)
        self.Cmd_Z_LineEdit.returnPressed.connect(self.SendCmd)
        self.Cmd_Flag_LineEdit.returnPressed.connect(self.SendCmd)



        ## ON BUTTON PRESS EXECUTE FUNCTIONS
        self.pauseButton.clicked.connect(self.pause_plots)
        self.rescaleButton.clicked.connect(self.rescale_plots)
        self.clearButton.clicked.connect(self.clear_plots)
        self.paused = False

        self.timerThread = Thread(target=self.updateValues)
        self.timerThread.start()


    def updateValues(self):
        while True:
            self.check_CFDC_Connection()
            self.update_LCD()

            if DATA_TYPE == "EXP":
                self.check_Vicon_Connection()
                self.check_UKF_Connection()
                self.check_CS_Connection()


            time.sleep(0.5)
          
            
    def update_LCD(self):
        self.K_ep_LCD.display(self.k_ep)
        self.K_run_LCD.display(self.k_run)
        self.Battery_Voltage.setText(f"{self.V_Battery:.3f}")
        self.Battery_Percentage.setValue(int((self.V_Battery-3.6)/(4.2-3.6)*100))

    def Emergency_Stop(self):       
        self.cmd_msg.cmd_type = 5
        self.RL_CMD_Publisher.publish(self.cmd_msg) 

    def HomeCmd(self):       
        self.cmd_msg.cmd_type = 0
        self.RL_CMD_Publisher.publish(self.cmd_msg)

    def SendCmd(self):

        ## CREATE LIST OF CMD LINE EDIT OBJECTS
        cmd_list = [
            self.Cmd_Type_LineEdit,
            self.Cmd_X_LineEdit,
            self.Cmd_Y_LineEdit,
            self.Cmd_Z_LineEdit,
            self.Cmd_Flag_LineEdit]

        ## SET ALL BLANK VALUES TO ZERO
        for cmd in cmd_list:
            if cmd.text() == '':
                cmd.setText('0')

        ## INSERT VALS INTO COMMAND MSG
        self.cmd_msg.cmd_type = int(cmd_list[0].text())
        self.cmd_msg.cmd_vals.x = float(cmd_list[1].text())
        self.cmd_msg.cmd_vals.y = float(cmd_list[2].text())
        self.cmd_msg.cmd_vals.z = float(cmd_list[3].text())
        self.cmd_msg.cmd_flag = float(cmd_list[4].text())

        ## PUBLISH COMMAND
        self.RL_CMD_Publisher.publish(self.cmd_msg)

        ## CLEAR LINE EDIT BOXES
        for cmd_val in cmd_list:
            cmd_val.clear()
        

        


    def check_Vicon_Connection(self): ## CHECK IF RECEIVING VALID VICON DATA
        try:
            rospy.wait_for_message('/vicon/cf1/cf1',TransformStamped,timeout=0.2)
            self.Vicon_LED.setPixmap(QPixmap(ICON_GREEN_LED))
        except:
            self.Vicon_LED.setPixmap(QPixmap(ICON_RED_LED))

    def check_UKF_Connection(self): ## CHECK IF RECEIVING VALID VICON DATA
        try:
            rospy.wait_for_message("/UKF/viconState_Filtered",Vicon_Filtered,timeout=0.2)
            self.Vicon_UKF_LED.setPixmap(QPixmap(ICON_GREEN_LED))
        except:
            self.Vicon_UKF_LED.setPixmap(QPixmap(ICON_RED_LED))
    
    def check_CS_Connection(self): ## CHECK IF RECEIVING VALID VICON DATA
        try:
            rospy.wait_for_message('/cf1/log1',GenericLogData,timeout=0.2)
            self.Crazyswarm_LED.setPixmap(QPixmap(ICON_GREEN_LED))
        except:
            self.Crazyswarm_LED.setPixmap(QPixmap(ICON_RED_LED))

    def check_CFDC_Connection(self): ## CHECK IF RECEIVING VALID VICON DATA
        try:
            rospy.wait_for_message('/CF_DC/StateData',CF_StateData,timeout=0.2)
            self.CF_DC_LED.setPixmap(QPixmap(ICON_GREEN_LED))
        except:
            self.CF_DC_LED.setPixmap(QPixmap(ICON_RED_LED))

    


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

    # from darktheme.widget_template import DarkPalette
    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)
    # app.setPalette(DarkPalette())

  
    ## INITIALIZE DASHBOARD WINDOW
    myApp = Dashboard()
    myApp.show()

    sys.exit(app.exec_())