import sys
from PyQt5.QtWidgets import *
import pyqtgraph as pg

import os,rospkg
from dashboard_node import DashboardNode
import numpy as np



colors = {
    "blue": (31,119,180),
    "orange": (255,127,14),
    "red": (214,39,40),
    "green": (44,160,44),
    "purple": (148,103,189),
    "blue_alpha": (31,119,180,150),
    "orange_alpha": (255,127,14,150),
}
width = 2

## DEFINE HOW LONG TO PRESERVE DATA   
refresh_rate = 15                              # Refresh rate [Hz]
buffer_time = 10_000                           # Amount of data to save [ms]
update_interval = 1_000//refresh_rate          # Convert refresh_rate to ms [ms]
buffer_length = buffer_time//update_interval   # Number of datapoints in plot


## UPDATE X-AXIS TICKS
num_ticks = 6
buff_ticks = np.linspace(0,buffer_length,num_ticks)         # Tick locations
time_ticks = np.linspace(-buffer_time//1000,0,num_ticks)    # Tick values
tick_labels = dict(zip(buff_ticks,['{:.1f}'.format(tick) for tick in time_ticks]))
pg.setConfigOption('foreground', 'k')
# pg.setConfigOption('background', 'w')
pg.setConfigOptions(antialias=True)

DashNode=DashboardNode()


class Pos_Widget2(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Position [m]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-1,4)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.PW.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.PW.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.PW.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.pos[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.pos[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.pos[2]
        self.curve_z.setData(self.z_arr)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(-1,4)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)


class Vel_Widget2(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Velocity [m/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-1,4)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.PW.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.PW.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.PW.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.vel[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.vel[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.vel[2]
        self.curve_z.setData(self.z_arr)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(-1,4)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)

if __name__ == '__main__':

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Pos_Widget2()
    myApp.show()

    sys.exit(app.exec_())