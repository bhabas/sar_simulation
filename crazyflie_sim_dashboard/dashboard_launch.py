
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

        # self.Pos_Graph.reset_axes()
        self.pushButton.clicked.connect(self.Pos_Graph.reset_axes)

    def printButtonPressed(self):
        # This is executed when the button is pressed
        print('printButtonPressed')

if __name__ == '__main__':

    # DashNode=DashboardNode()
    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    dashboard = Dashboard()
    dashboard.show()

    sys.exit(app.exec())