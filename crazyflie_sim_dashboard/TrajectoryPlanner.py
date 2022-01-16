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
    def __init__(self,min,max,label='Text'):
        super().__init__()
        
        ## DEFINE VARIABLES
        self.min = min
        self.max = max
        self.x = np.nan # Slider value

        ## DEFINE LAYOUT
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

        ## DEFINE OBJECTS
        self.var = QLabel()
        self.val = QLabel()
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)

        ## ADD OBJECTS TO LAYOUT
        self.layout.addWidget(self.var)
        self.layout.addWidget(self.val)
        self.layout.addWidget(self.slider)

        
        ## SET DEFAULT TEXT
        self.var.setText(label)
        self.val.setText(f"{min:.3f}")
        self.slider.valueChanged.connect(self.setLabelValue)



    def setLabelValue(self,value):

        self.x = self.min + (self.max - self.min)*(float(value)/(self.slider.maximum() - self.slider.minimum()))
        self.val.setText(f"{self.x:.3f}")


        



class Traj_Planner(QWidget):
    def __init__(self):
        super().__init__()
        self.resize(800,800)


        ## INITIAL DATA
        self.h_ceil = 2.1
        self.z_0 = 0.4
        self.RREV_max = 7

        
        self.vel = 2.0
        self.theta = 90

        self.vel_x = self.vel*np.cos(np.radians(self.theta))
        self.vel_z = self.vel*np.sin(np.radians(self.theta))

        self.t_c = (self.h_ceil - self.z_0)/self.vel_z - 1/self.RREV_max
        self.t = np.linspace(0,self.t_c,50).reshape(-1,1)


        d = self.h_ceil - (self.z_0 + self.vel_z*self.t)
        RREV = self.vel_z/d
        OFy = -self.vel_x/d
        data = np.hstack((OFy,RREV,d))

        self.layout = QVBoxLayout()


        ## INITIALIZE 3D PLOT
        glvw = gl.GLViewWidget()
        glvw.orbit(-135,45)
        self.layout.addWidget(glvw)

        self.traj = gl.GLLinePlotItem(pos=data,width=2)
        glvw.addItem(self.traj)


        ## INIT AXES 
        x_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[1,0,0]]),color=(255,0,0,1),width=1)
        y_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,1,0]]),color=(0,255,0,1),width=1)
        z_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,1]]),color=(0,0,255,1),width=1)
        glvw.addItem(x_axis)
        glvw.addItem(y_axis)
        glvw.addItem(z_axis)


        ## INIT GRIDS
        x_grid = gl.GLGridItem()
        x_grid.setSize(x=10,y=10)
        x_grid.setSpacing(x=1,y=1)
        x_grid.translate(-5,5,0)

        z_grid = gl.GLGridItem(color=(255, 255, 255, 20))
        z_grid.setSpacing(x=1,y=1)
        z_grid.setSize(x=10,y=10)
        z_grid.translate(5,5,0)
        z_grid.rotate(-90,0,1,0)

        
        glvw.addItem(x_grid)
        glvw.addItem(z_grid)
        


        

        ## INIT SLIDERS
        self.w1 = Slider(min=1.0,max=4.0,label="Vel:")
        self.w2 = Slider(min=20,max=90,label="Theta:")

        self.w1.slider.valueChanged.connect(self.updateText)
        self.w2.slider.valueChanged.connect(self.updateText)

        self.layout.addWidget(self.w1)
        self.layout.addWidget(self.w2)



        ## DIVIDE WIDGET
        self.layout.setStretch(0, 50)
        self.layout.setStretch(1, 5)
        self.layout.setStretch(2, 5)
        self.setLayout(self.layout)



    def updateText(self):
        
        self.vel = self.w1.x
        self.theta = self.w2.x

        self.vel_x = self.vel*np.cos(np.radians(self.theta))
        self.vel_z = self.vel*np.sin(np.radians(self.theta))

        self.t_c = (self.h_ceil - self.z_0)/self.vel_z - 1/self.RREV_max
        self.t = np.linspace(0,self.t_c,50).reshape(-1,1)

        d = self.h_ceil - (self.z_0 + self.vel_z*self.t)
        RREV = self.vel_z/d
        OFy = -self.vel_x/d
        data = np.hstack((OFy,RREV,d))


        

        self.traj.setData(pos=data)





if __name__ == '__main__':

    ## INITIALIZE APPLICATION
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Traj_Planner()
    myApp.show()

    sys.exit(app.exec())



