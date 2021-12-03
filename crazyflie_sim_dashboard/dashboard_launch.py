
import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow, QStackedLayout, QStackedWidget, QWidget

import pyqtgraph as pg
from pyqtgraph import PlotWidget, plot

from dashboard_node import DashboardNode

class Dashboard(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(Dashboard, self).__init__(*args, **kwargs)

        #LOAD UI
        loadUi('crazyflie_sim_dashboard/mainwindow.ui', self)


def main():

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    dashboard = Dashboard()
    dashboard.show()

    sys.exit(app.exec())

if __name__ == '__main__':

    # DashNode=DashboardNode()
    main()
    