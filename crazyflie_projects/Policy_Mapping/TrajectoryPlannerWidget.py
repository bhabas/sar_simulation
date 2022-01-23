import sys,os,rospkg
import random

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt


import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import pandas as pd


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

DATA_PATH = f"{BASE_PATH}/crazyflie_projects/ICRA_DataAnalysis/Wide-Long_2-Policy/Wide-Long_2-Policy_Summary.csv"
df = pd.read_csv(DATA_PATH)
df = df.query('landing_rate_4_leg >= 0.7')

class Slider(QWidget):
    def __init__(self,min=3,max=10.5,label='Text'):
        super().__init__()
        
        ## DEFINE VARIABLES
        self.min = min
        self.max = max
        self.value = np.nan 

        ## DEFINE LAYOUT
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

        ## DEFINE OBJECTS
        self.var = QLabel()
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)
        
        self.numBox = QDoubleSpinBox()
        self.numBox.setMinimum(self.min)
        self.numBox.setMaximum(self.max)

        ## ADD OBJECTS TO LAYOUT
        self.layout.addWidget(self.var)
        self.layout.addWidget(self.numBox)
        self.layout.addWidget(self.slider)

        
        ## SET DEFAULT TEXT
        self.var.setText(f"{label}: \t")
        self.slider.sliderMoved.connect(self.sliderUpdate)
        self.numBox.editingFinished.connect(self.numboxUpdate)

    def sliderUpdate(self,sliderVal):

        self.value = self.min + (sliderVal - 0)/(100 - 0) * (self.max-self.min)
        self.numBox.setValue(self.value)
        print(f"slider: {self.value:.3f}")

    def numboxUpdate(self):

        self.value = self.numBox.value()

        slider_val = self.slider.minimum() + (self.value - self.min)/(self.max-self.min) * (self.slider.maximum()-self.slider.minimum())
        self.slider.setValue(int(np.round(slider_val,0)))
        print(f"spin: {self.value:.3f}")





class Demo(QWidget):

    def __init__(self):
        super().__init__()

        
        ## INITIATE SLIDER
        self.velSlider = Slider(min=1.0,max=4.0,label="Vel")
        self.thetaSlider = Slider(min=20,max=90,label="Theta")

        
        

        self.velSlider.slider.valueChanged.connect(self.updateValues)
        self.velSlider.numBox.valueChanged.connect(self.updateValues)

        self.thetaSlider.slider.valueChanged.connect(self.updateValues)
        self.thetaSlider.numBox.valueChanged.connect(self.updateValues)

        self.vel = self.velSlider.value
        self.theta = self.thetaSlider.value

        self.vel = 1.0
        self.theta = 90

        self.vx = self.vel*np.cos(np.radians(self.theta))
        self.vz = self.vel*np.sin(np.radians(self.theta))


        ## INITIATE PLOT
        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas,self)

        layout = QVBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        layout.addWidget(self.velSlider)
        layout.addWidget(self.thetaSlider)

        # random data
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
        
        self.ax = self.fig.add_subplot(111,projection='3d')
        self.ax.scatter(df["vx"],df["vz"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
        self.POI, = self.ax.plot([self.vx],[self.vz],[0.4],'ko')

        self.ax.set_xlim(0,4)
        self.ax.set_ylim(0,4)
        self.ax.set_zlim(0,2)

        self.ax.set_xlabel("Vel_x")
        self.ax.set_ylabel("Vel_z")
        self.ax.set_zlabel("d_ceil")

        


    def updateValues(self):
        
        self.vel = self.velSlider.value
        self.theta = self.thetaSlider.value

        self.vx = self.vel*np.cos(np.radians(self.theta))
        self.vz = self.vel*np.sin(np.radians(self.theta))

        ## SET POINT OF INTEREST COORDINATES FROM SLIDER
        self.POI.set_data([self.vx],[self.vz])
        self.POI.set_3d_properties([0.4])





if __name__ == "__main__":

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)
  
    ## INITIALIZE DASHBOARD WINDOW
    myApp = Demo()
    myApp.show()

    sys.exit(app.exec_())