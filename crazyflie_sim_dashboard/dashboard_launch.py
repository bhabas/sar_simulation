
import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow, QStackedLayout, QStackedWidget, QWidget

import pyqtgraph as pg
from pyqtgraph import PlotWidget, plot


class Dashboard(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(Dashboard, self).__init__(*args, **kwargs)

        #LOAD UI
        loadUi('crazyflie_sim_dashboard/mainwindow.ui', self)

        self.K_ep_num.display(50)
        self.K_run_num.display(88)

        self.Battery_Voltage.setValue(50)

        # self.pushButton.clicked.connect(self.Pos_Graph.pause)
        self.pauseButton.clicked.connect(self.pause_plots)
        self.rescaleButton.clicked.connect(self.rescale_plots)

        self.paused = False

        self.plot_list = [
            self.Pos_Graph,
            self.Vel_Graph,
            self.PWM_Graph,
            self.Dist_Graph,
            self.Eul_Graph,
            self.OF_Graph,
            self.Omega_Graph,]


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