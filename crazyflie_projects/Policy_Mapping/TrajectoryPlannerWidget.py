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
df = df.query('landing_rate_4_leg >= 0.7 and 6.0 < My_d < 7.0')
df = df.query('landing_rate_4_leg >= 0.7')

class Slider(QWidget):
    def __init__(self,min=3,max=10.5,label='Text'):
        super().__init__()
        
        ## DEFINE VARIABLES
        self.min = min
        self.max = max
        self.value = np.mean((self.max,self.min))

        ## DEFINE LAYOUT
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

        ## DEFINE OBJECTS
        self.var = QLabel()
        self.slider = QSlider()
        self.slider.setOrientation(Qt.Horizontal)
        self.slider.setValue(self.sliderConvert(self.value))
        

        
        self.numBox = QDoubleSpinBox()
        self.numBox.setMinimum(self.min)
        self.numBox.setMaximum(self.max)
        self.numBox.setValue(self.value)

        ## ADD OBJECTS TO LAYOUT
        self.layout.addWidget(self.var)
        self.layout.addWidget(self.numBox)
        self.layout.addWidget(self.slider)

        
        ## SET DEFAULT TEXT
        self.var.setText(f"{label}: \t")
        self.slider.sliderMoved.connect(self.sliderUpdate)
        self.numBox.editingFinished.connect(self.numboxUpdate)

    def sliderConvert(self,value):
        slider_val = self.slider.minimum() + (value - self.min)/(self.max-self.min) * (self.slider.maximum()-self.slider.minimum())
        return int(slider_val)

    def sliderUpdate(self,sliderVal):

        self.value = self.min + (sliderVal - 0)/(100 - 0) * (self.max-self.min)
        self.numBox.setValue(self.value)
        # print(f"slider: {self.value:.3f}")

    def numboxUpdate(self):

        self.value = self.numBox.value()

        slider_val = self.slider.minimum() + (self.value - self.min)/(self.max-self.min) * (self.slider.maximum()-self.slider.minimum())
        self.slider.setValue(int(np.round(slider_val,0)))
        # print(f"spin: {self.value:.3f}")


## SYSTEM CONSTANTS
H_CEIL = 2.1
Z_0 = 0.4
RREV_MAX = 8.0 # Cutoff value for plots (Practically contact at this point (0.125s))


class Demo(QWidget):

    def __init__(self):
        super().__init__()
        self.resize(1000,600)


        ## INITIATE SLIDERS
        self.velSlider = Slider(min=0.5,max=4.0,label="Vel")
        self.thetaSlider = Slider(min=20,max=90,label="Theta")
        self.d_ceilSlider = Slider(min=0.05,max=1.8,label="d_Ceil")


        ## INITIATE PLOT
        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas,self)

        ## VALUE LABELS
        hLayout = QHBoxLayout()
        self.RREV_label = QLabel()
        self.OFy_label = QLabel()
        self.d_ceil_label = QLabel()
        hLayout.addWidget(self.RREV_label)
        hLayout.addWidget(self.OFy_label)
        hLayout.addWidget(self.d_ceil_label)


        ## OVERALL PLOT LAYOUT
        layout = QVBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        layout.addLayout(hLayout)
        layout.addWidget(self.velSlider)
        layout.addWidget(self.thetaSlider)
        layout.addWidget(self.d_ceilSlider)

        vz_range = np.linspace(0.5,4.0,11)
        vx_range = np.linspace(0.5,4.0,11)
        XX,YY = np.meshgrid(vx_range,vz_range)
        data = np.vstack((XX.flatten(),YY.flatten())).T

        vx_range = data[:,0]
        vz_range = data[:,1]
        d_range = 0.2114*vz_range
        RREV_range = vz_range/d_range
        OFy_range = -vx_range/d_range



        ## PLOT SETUP
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=10)
        
        self.ax1 = self.fig.add_subplot(121,projection='3d')
        self.ax1.scatter(df["vx"],df["vz"],df["flip_d_mean"],c=df["My_d"],cmap=cmap,norm=norm)
        self.POI_1, = self.ax1.plot([],[],[],'ko')
        self.traj_1, = self.ax1.plot([],[],[])
        self.ax1.scatter(vx_range,vz_range,d_range)

        self.ax1.set_xlim(0,4)
        self.ax1.set_ylim(0,4)
        self.ax1.set_zlim(0,1.8) 

        self.ax1.set_xlabel("Vel_x [m/s]")
        self.ax1.set_ylabel("Vel_z [m/s]")
        self.ax1.set_zlabel("d_ceil [m]")

        self.ax2 = self.fig.add_subplot(122,projection='3d')
        self.ax2.scatter(df["OF_y_flip_mean"],df["RREV_flip_mean"],df["flip_d_mean"],c=df["My_d"],cmap=cmap,norm=norm)
        self.POI_2, = self.ax2.plot([],[],[],'ko')
        self.traj_2, = self.ax2.plot([],[],[])
        self.ax2.scatter(OFy_range,RREV_range,d_range)


        self.ax2.set_xlim(-15,0)
        self.ax2.set_ylim(0,8)
        self.ax2.set_zlim(0,1.8) 

        self.ax2.set_xlabel("OFy [rad/s]")
        self.ax2.set_ylabel("RREV [rad/s]")
        self.ax2.set_zlabel("d_ceil [m")

        self.updatePlots()



        ## UPDATE FUNCTIONS
        self.velSlider.slider.valueChanged.connect(self.updatePlots)
        self.velSlider.numBox.valueChanged.connect(self.updatePlots)

        self.thetaSlider.slider.valueChanged.connect(self.updatePlots)
        self.thetaSlider.numBox.valueChanged.connect(self.updatePlots)

        self.d_ceilSlider.slider.valueChanged.connect(self.updatePlots)
        self.d_ceilSlider.numBox.valueChanged.connect(self.updatePlots)
        

        


    def updatePlots(self):
        
        ## INPUT VALUES (POINT)
        self.vel_pt = self.velSlider.value
        self.theta_pt = self.thetaSlider.value
        self.d_pt = self.d_ceilSlider.value

        ## SENSORY VALUES (POINT)
        self.vz_pt = self.vel_pt*np.sin(np.radians(self.theta_pt))
        self.vx_pt = self.vel_pt*np.cos(np.radians(self.theta_pt))
        self.RREV_pt = self.vz_pt/self.d_pt
        self.OFy_pt = -self.vx_pt/self.d_pt

        ## TIME VALUES
        self.t_c = (H_CEIL - Z_0)/(self.vz_pt) - 1/RREV_MAX # t_cutoff
        self.t = np.linspace(0,self.t_c,50)

        ## VELOCITY VALUES (TRAJECTORY)
        self.vz_traj = self.vel_pt*np.sin(np.radians(self.theta_pt))*np.ones_like(self.t)
        self.vx_traj = self.vel_pt*np.cos(np.radians(self.theta_pt))*np.ones_like(self.t)
        
        ## SENSORY VALUES (TRAJECTORY)
        self.d_traj = H_CEIL - (Z_0 + self.vz_traj*self.t)
        self.RREV_traj = self.vz_traj/self.d_traj
        self.OFy_traj = -self.vx_traj/self.d_traj

        ## SET POINT OF INTEREST COORDINATES FROM SLIDER
        self.POI_1.set_data([self.vx_pt],[self.vz_pt])
        self.POI_1.set_3d_properties([self.d_pt])
        self.traj_1.set_data(self.vx_traj,self.vz_traj)
        self.traj_1.set_3d_properties(self.d_traj)

        self.POI_2.set_data([self.OFy_pt],[self.RREV_pt])
        self.POI_2.set_3d_properties([self.d_pt])
        self.traj_2.set_data(self.OFy_traj,self.RREV_traj)
        self.traj_2.set_3d_properties(self.d_traj)
        self.canvas.draw()

        self.RREV_label.setText(f"RREV: {self.RREV_pt:.2f}\t")
        self.OFy_label.setText(f"OFy: {self.OFy_pt:.2f}\t")
        self.d_ceil_label.setText(f"d_ceil: {self.d_pt:.2f}\t")





if __name__ == "__main__":

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)
  
    ## INITIALIZE DASHBOARD WINDOW
    myApp = Demo()
    myApp.show()

    sys.exit(app.exec_())