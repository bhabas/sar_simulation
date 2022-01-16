#!/usr/bin/env python3

import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import os,rospkg

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)

class Slider(QWidget):
    def __init__(self):
        super().__init__()

        self.layout = QHBoxLayout()
        self.var = QLabel()
        self.val = QLabel()
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)
        self.setLayout(self.layout)


        self.layout.addWidget(self.var)
        self.layout.addWidget(self.val)
        self.layout.addWidget(self.slider)
        



class Traj_Planner(QWidget):
    def __init__(self):
        super().__init__()

        self.pw1 = pg.PlotWidget(name='Plot1') # Plot window 1


        self.layout = QVBoxLayout()
        self.layout.addWidget(self.pw1)

        self.w1 = Slider()
        self.w1.var.setText("hello")
        self.layout.addWidget(self.w1)

        self.setLayout(self.layout)

        self.pw1 = pg.PlotWidget(name='Plot1') # Plot window 1


        self.p1 = self.pw1.plot() # Plot Window 1 -> plot



    def updateText(self):
        self.Label1.setText(f"{self.Slider1.value()}")
        self.Label2.setText(f"{self.Slider2.value()}")
        self.Label3.setText(f"{self.Slider3.value()}")




if __name__ == '__main__':

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Traj_Planner()
    myApp.show()

    sys.exit(app.exec())