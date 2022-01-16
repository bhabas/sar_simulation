#!/usr/bin/env python3

import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import os,rospkg
import pyqtgraph.opengl as gl
import numpy as np


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
        self.resize(500,500)

        glvw = gl.GLViewWidget()

        x = np.array([0,1]).reshape(-1,1)
        y = np.array([0,0]).reshape(-1,1)
        z = np.array([0,1]).reshape(-1,1)
        pos = np.hstack((x,y,z))

        self.sp1 = gl.GLLinePlotItem(pos=pos,width=5)
        xgrid = gl.GLGridItem()
        glvw.addItem(self.sp1)
        glvw.addItem(xgrid)


        self.layout = QVBoxLayout()
        self.layout.addWidget(glvw)
        # glvw.sizeHint = lambda: pg.QtCore.QSize(100, 100)

        self.w1 = Slider()
        self.w1.var.setText("hello")
        self.layout.addWidget(self.w1)

        self.setLayout(self.layout)



        self.w1.slider.valueChanged.connect(self.updateText)


    def updateText(self):
        self.w1.var.setText(f"{self.w1.slider.value()}")

        x = np.array([0,1,2,3]).reshape(-1,1)
        y = np.array([0,0,0,0]).reshape(-1,1)
        z = np.array([0,1,4,9]).reshape(-1,1)
        pos2 = np.hstack((x,y,z))

        self.sp1.setData(pos=pos2)





if __name__ == '__main__':

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Traj_Planner()
    myApp.show()

    sys.exit(app.exec())