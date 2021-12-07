
import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow
import pyqtgraph as pg
from dashboard_node import DashboardNode
from nav_msgs.msg import Odometry


import rospy

DashNode=DashboardNode()


class Dashboard(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(Dashboard, self).__init__(*args, **kwargs)
        ## LOOK INTO INITIALIZING DASHBOARDNODE CLASS

        #LOAD UI
        loadUi('crazyflie_sim_dashboard/mainwindow.ui', self)

        self.paused = False

        self.plot_list = [
            self.Pos_Graph,
            self.Vel_Graph,
            self.PWM_Graph,
            self.Dist_Graph,
            self.Eul_Graph,
            self.OF_Graph,
            self.Omega_Graph,]


        self.Battery_Voltage.setValue(50)

        self.pauseButton.clicked.connect(self.pause_plots)
        self.rescaleButton.clicked.connect(self.rescale_plots)

        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_LCD)
        self.timer.timeout.connect(self.check_Vicon_Comm)
        self.timer.start(500) # number of seconds (every 1000) for next update
        self.ptr = 0

    def update_LCD(self):
        self.ptr += 1
        self.K_ep_LCD.display(DashNode.k_ep)
        self.K_run_LCD.display(DashNode.k_run)

    def check_Vicon_Comm(self): ## CHECK IF RECEIVING VALID VICON DATA
        try:
            rospy.wait_for_message('/env/vicon_state',Odometry,timeout=0.1)
            self.Vicon_Connection.setStyleSheet("background-color: rgb(44, 160, 44);")
        except:
            self.Vicon_Connection.setStyleSheet("background-color: rgb(214,39,40);")



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

        self.Mu_Graph.reset_axes()
        self.Sigma_Graph.reset_axes()


if __name__ == '__main__':

    # DashNode=DashboardNode()
    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    dashboard = Dashboard()
    dashboard.show()

    sys.exit(app.exec())