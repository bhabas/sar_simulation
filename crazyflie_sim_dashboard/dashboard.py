
import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow, QStackedLayout, QStackedWidget, QWidget

import pyqtgraph as pg
from pyqtgraph import PlotWidget, plot

class Dashboard(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(Dashboard, self).__init__(*args, **kwargs)

        #Load the UI Page
        loadUi('crazyflie_sim_dashboard/mainwindow.ui', self)


def main():
    app = QApplication(sys.argv)
    dashboard = Dashboard()
    dashboard.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
    