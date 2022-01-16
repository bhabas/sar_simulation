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
    def __init__(self,label='Text',min=0,max=100):
        super().__init__()
        self.max = max
        self.min = min
        self.layout = QHBoxLayout()
        self.var = QLabel()
        self.val = QLabel()
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)
        self.setLayout(self.layout)
        
        self.var.setText(label)
        self.val.setText(f"{min:.3f}")


        self.layout.addWidget(self.var)
        self.layout.addWidget(self.val)
        self.layout.addWidget(self.slider)


        self.slider.valueChanged.connect(self.updateText)


    def updateText(self):

        self.val.setText(f"{self.slider.value()/(self.max-self.min):.3f}")


        



class Traj_Planner(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(800,800)


        ## INITIAL DATA
        self.t = np.linspace(0,0.5,50).reshape(-1,1)
        self.vel = 2
        self.theta = 90
        self.h_ceil = 2.1
        self.z_0 = 0.4
        vel_z = self.vel*np.sin(np.radians(self.theta))
        vel_x = self.vel*np.cos(np.radians(self.theta))

        d = self.h_ceil - (self.z_0 + vel_z*self.t)
        RREV = vel_z/d
        OFy = -vel_x/d

        data = np.hstack((OFy,RREV,d))


        ## INITIALIZE 3D PLOT
        glvw = gl.GLViewWidget()
        glvw.orbit(-135,45)

        self.traj = gl.GLLinePlotItem(pos=data,width=3)
        xgrid = gl.GLGridItem()
        xgrid.setSize(x=10,y=20)
        xgrid.setSpacing(x=1,y=1)
        glvw.addItem(self.traj)
        glvw.addItem(xgrid)


        self.layout = QVBoxLayout()
        self.layout.addWidget(glvw)

        ## INIT SLIDERS
        self.w1 = Slider(label="Vel:",min=1.0,max=4.0)
        self.w2 = Slider(label="Theta:",min=0,max=90)


        self.layout.addWidget(self.w1)
        self.layout.addWidget(self.w2)



        ## DIVIDE WIDGET
        self.layout.setStretch(0, 50)
        self.layout.setStretch(1, 5)
        self.layout.setStretch(2, 5)


        self.setLayout(self.layout)




        self.w1.slider.valueChanged.connect(self.updateText)
        self.w2.slider.valueChanged.connect(self.updateText)



    def updateText(self):
        
        theta = self.w1.slider.value()
        vel = self.w2.slider.value()

        vel_z = vel*np.sin(np.radians(theta))
        vel_x = vel*np.cos(np.radians(theta))


        d = self.h_ceil - (self.z_0 + vel_z*self.t)
        RREV = vel_z/d
        OFy = -vel_x/d

        data = np.hstack((OFy,RREV,d))

        self.traj.setData(pos=data)





if __name__ == '__main__':

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Traj_Planner()
    myApp.show()

    sys.exit(app.exec())