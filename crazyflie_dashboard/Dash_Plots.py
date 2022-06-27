import sys
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import pyqtgraph as pg

import os,rospkg


BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

from Dashboard_Node import DashboardNode
import numpy as np



colors = {
    "gray": (25,25,25),
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


class Pos_Widget(QWidget):
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


        ## INIT POSITION CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.PW.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.PW.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.PW.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT SETPOINT CURVES
        self.xd_arr = np.zeros(buffer_length)
        self.curve_xd = self.PW.plot(self.xd_arr, pen=pg.mkPen(color=colors["red"], width=1.2,style=QtCore.Qt.DotLine))

        self.yd_arr = np.zeros(buffer_length)
        self.curve_yd = self.PW.plot(self.yd_arr, pen=pg.mkPen(color=colors["green"], width=1.2,style=QtCore.Qt.DotLine))

        self.zd_arr = np.zeros(buffer_length)
        self.curve_zd = self.PW.plot(self.zd_arr, pen=pg.mkPen(color=colors["blue"], width=1.2,style=QtCore.Qt.DotLine))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        ## UPDATE POSITION CURVES
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.posCF[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.posCF[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.posCF[2]
        self.curve_z.setData(self.z_arr)

        ## UPDATE SETPOINT CURVES
        self.xd_arr = np.roll(self.xd_arr,-1) 
        self.xd_arr[-1] = DashNode.x_d[0]
        self.curve_xd.setData(self.xd_arr)

        self.yd_arr = np.roll(self.yd_arr,-1) 
        self.yd_arr[-1] = DashNode.x_d[1]
        self.curve_yd.setData(self.yd_arr)

        self.zd_arr = np.roll(self.zd_arr,-1) 
        self.zd_arr[-1] = DashNode.x_d[2]
        self.curve_zd.setData(self.zd_arr)

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
        self.xd_arr = np.zeros(buffer_length)

        self.y_arr = np.zeros(buffer_length)
        self.yd_arr = np.zeros(buffer_length)

        self.z_arr = np.zeros(buffer_length)
        self.zd_arr = np.zeros(buffer_length)

class Vel_Widget(QWidget):
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

         ## INIT SETPOINT CURVES
        self.vx_d_arr = np.zeros(buffer_length)
        self.curve_vx_d = self.PW.plot(self.vx_d_arr, pen=pg.mkPen(color=colors["red"], width=1.2,style=QtCore.Qt.DotLine))

        self.vy_d_arr = np.zeros(buffer_length)
        self.curve_vy_d = self.PW.plot(self.vy_d_arr, pen=pg.mkPen(color=colors["green"], width=1.2,style=QtCore.Qt.DotLine))

        self.vz_d_arr = np.zeros(buffer_length)
        self.curve_vz_d = self.PW.plot(self.vz_d_arr, pen=pg.mkPen(color=colors["blue"], width=1.2,style=QtCore.Qt.DotLine))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
        
        ## UPDATE VELOCITY CURVES
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.velCF[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.velCF[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.velCF[2]
        self.curve_z.setData(self.z_arr)

        ## UPDATE SETPOINT CURVES
        self.vx_d_arr = np.roll(self.vx_d_arr,-1) 
        self.vx_d_arr[-1] = DashNode.v_d[0]
        self.curve_vx_d.setData(self.vx_d_arr)

        self.vy_d_arr = np.roll(self.vy_d_arr,-1) 
        self.vy_d_arr[-1] = DashNode.v_d[1]
        self.curve_vy_d.setData(self.vy_d_arr)

        self.vz_d_arr = np.roll(self.vz_d_arr,-1) 
        self.vz_d_arr[-1] = DashNode.v_d[2]
        self.curve_vz_d.setData(self.vz_d_arr)

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

class Omega_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Ang. Vel. [rad/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-40,40)
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
        self.x_arr[-1] = DashNode.omegaCF[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.omegaCF[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.omegaCF[2]
        self.curve_z.setData(self.z_arr)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(-40,40)
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

class Eul_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Angle [deg]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-90,90)
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
        self.x_arr[-1] = DashNode.eulCF[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.eulCF[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.eulCF[2]
        self.curve_z.setData(self.z_arr)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(-90,90)
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

class Tau_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Tau [s]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(0,1)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.Tau_arr = np.zeros(buffer_length)
        self.curve_Tau = self.PW.plot(self.Tau_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT ESTIMATE CURVES
        self.Tau_est_arr = np.zeros(buffer_length)
        self.curve_Tau_est = self.PW.plot(self.Tau_est_arr, pen=pg.mkPen(color=colors["blue"], width=1.2,style=QtCore.Qt.DotLine))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.Tau_arr = np.roll(self.Tau_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.Tau_arr[-1] = DashNode.Tau
        self.curve_Tau.setData(self.Tau_arr)

        ## UPDATE ESTIMATE CURVES
        self.Tau_est_arr = np.roll(self.Tau_est_arr,-1) 
        self.Tau_est_arr[-1] = DashNode.Tau_est
        self.curve_Tau_est.setData(self.Tau_est_arr)

    def reset_axes(self):
        self.PW.setYRange(0,1)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.Tau_arr = np.zeros(buffer_length)
        self.Tau_est_arr = np.zeros(buffer_length)


class OFy_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'OF [rad/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-10,10)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.OFy_arr = np.zeros(buffer_length)
        self.curve_OFy = self.PW.plot(self.OFy_arr, pen=pg.mkPen(color=colors["green"], width=width))

        ## INIT ESTIMATE CURVES
        self.OFy_est_arr = np.zeros(buffer_length)
        self.curve_OFy_est = self.PW.plot(self.OFy_est_arr, pen=pg.mkPen(color=colors["green"], width=1.2,style=QtCore.Qt.DotLine))


        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.OFy_arr = np.roll(self.OFy_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.OFy_arr[-1] = DashNode.OFy
        self.curve_OFy.setData(self.OFy_arr)

        ## UPDATE ESTIMATE CURVES
        self.OFy_est_arr = np.roll(self.OFy_est_arr,-1) 
        self.OFy_est_arr[-1] = DashNode.OFy_est
        self.curve_OFy_est.setData(self.OFy_est_arr)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(0,5)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)


class OFx_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'OF [rad/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(-10,10)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.OFx_arr = np.zeros(buffer_length)
        self.curve_OFx = self.PW.plot(self.OFx_arr, pen=pg.mkPen(color=colors["red"], width=width))

        ## INIT ESTIMATE CURVES
        self.OFx_est_arr = np.zeros(buffer_length)
        self.curve_OFx_est = self.PW.plot(self.OFx_est_arr, pen=pg.mkPen(color=colors["red"], width=1.2,style=QtCore.Qt.DotLine))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.OFx_arr = np.roll(self.OFx_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.OFx_arr[-1] = DashNode.OFx
        self.curve_OFx.setData(self.OFx_arr)
        
        ## UPDATE ESTIMATE CURVES
        self.OFx_est_arr = np.roll(self.OFx_est_arr,-1) 
        self.OFx_est_arr[-1] = DashNode.OFx_est
        self.curve_OFx_est.setData(self.OFx_est_arr)


    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(-10,10)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.OFx_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)

class Dist_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Distance [m]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(0,5)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.PW.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.d_ceil
        self.curve_x.setData(self.x_arr)


    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(0,5)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)


class PWM_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## DEFINE HOW LONG TO PRESERVE DATA
        self.buffer_time = 20_000                               # Amount of data to save [ms]
        update_interval = 1_000//refresh_rate                        # [ms]
        self.buffer_length = self.buffer_time//update_interval   # Number of datapoints in plot

        ## UPDATE X-AXIS TICKS
        num_ticks = 11
        buff_ticks = np.linspace(0,self.buffer_length,num_ticks)         # Tick locations
        time_ticks = np.linspace(-self.buffer_time//1000,0,num_ticks)    # Tick values
        tick_labels = dict(zip(buff_ticks,['{:.1f}'.format(tick) for tick in time_ticks]))

        
        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Motor Command [PWM]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)

        ay = self.PW.getAxis('left')
        ticks = [0,20e3,40e3,60e3]
        ay.setTicks([[(v, str(v)) for v in ticks ]])
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(0,70_000)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.M1_arr = np.zeros(buffer_length)
        self.curve_M1 = self.PW.plot(self.M1_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.M2_arr = np.zeros(buffer_length)
        self.curve_M2 = self.PW.plot(self.M2_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.M3_arr = np.zeros(buffer_length)
        self.curve_M3 = self.PW.plot(self.M3_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        self.M4_arr = np.zeros(buffer_length)
        self.curve_M4 = self.PW.plot(self.M4_arr, pen=pg.mkPen(color=colors["orange"], width=width))

        self.PWM_Max = np.ones(buffer_length)*65535
        self.curve_PWM_Max = self.PW.plot(self.PWM_Max, pen=pg.mkPen(color=colors["gray"], width=width, style=QtCore.Qt.DashLine))
        self.curve_PWM_Max.setAlpha(0.4,False)

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.M1_arr = np.roll(self.M1_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.M1_arr[-1] = DashNode.MS_PWM[0]+5
        self.curve_M1.setData(self.M1_arr)

        self.M2_arr = np.roll(self.M2_arr,-1)
        self.M2_arr[-1] = DashNode.MS_PWM[1]+4
        self.curve_M2.setData(self.M2_arr)

        self.M3_arr = np.roll(self.M3_arr,-1)
        self.M3_arr[-1] = DashNode.MS_PWM[2]-5
        self.curve_M3.setData(self.M3_arr)

        self.M4_arr = np.roll(self.M4_arr,-1)
        self.M4_arr[-1] = DashNode.MS_PWM[3]-4
        self.curve_M4.setData(self.M4_arr)


    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setYRange(0,70_000)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.M1_arr = np.zeros(buffer_length)
        self.M2_arr = np.zeros(buffer_length)
        self.M3_arr = np.zeros(buffer_length)
        self.M4_arr = np.zeros(buffer_length)

class MotorThrust_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        ## DEFINE HOW LONG TO PRESERVE DATA
        self.buffer_time = 20_000                               # Amount of data to save [ms]
        update_interval = 1_000//refresh_rate                        # [ms]
        self.buffer_length = self.buffer_time//update_interval   # Number of datapoints in plot

        ## UPDATE X-AXIS TICKS
        num_ticks = 11
        buff_ticks = np.linspace(0,self.buffer_length,num_ticks)         # Tick locations
        time_ticks = np.linspace(-self.buffer_time//1000,0,num_ticks)    # Tick values
        tick_labels = dict(zip(buff_ticks,['{:.1f}'.format(tick) for tick in time_ticks]))

        
        ## CREATE AXIS ITEM TO CREATE CUSTOM TICK LABELS
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        



        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Motor Thrust [g]', 'bottom':'Time [s]'},axisItems={'bottom': ax}) # Plot window 1
        self.layout.addWidget(self.PW)

        ay = self.PW.getAxis('left')
        ticks = np.arange(0,20,2)
        ay.setTicks([[(v, str(v)) for v in ticks ]])
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(0,20)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.MotorThrust1_arr = np.zeros(buffer_length)
        self.curve_M1 = self.PW.plot(self.MotorThrust1_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.MotorThrust2_arr = np.zeros(buffer_length)
        self.curve_M2 = self.PW.plot(self.MotorThrust2_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.MotorThrust3_arr = np.zeros(buffer_length)
        self.curve_M3 = self.PW.plot(self.MotorThrust3_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        self.MotorThrust4_arr = np.zeros(buffer_length)
        self.curve_M4 = self.PW.plot(self.MotorThrust4_arr, pen=pg.mkPen(color=colors["orange"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def update(self):
                        
        self.MotorThrust1_arr = np.roll(self.MotorThrust1_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.MotorThrust1_arr[-1] = DashNode.MotorThrusts[0]
        self.curve_M1.setData(self.MotorThrust1_arr)

        self.MotorThrust2_arr = np.roll(self.MotorThrust2_arr,-1)
        self.MotorThrust2_arr[-1] = DashNode.MotorThrusts[1]
        self.curve_M2.setData(self.MotorThrust2_arr)

        self.MotorThrust3_arr = np.roll(self.MotorThrust3_arr,-1)
        self.MotorThrust3_arr[-1] = DashNode.MotorThrusts[2]
        self.curve_M3.setData(self.MotorThrust3_arr)

        self.MotorThrust4_arr = np.roll(self.MotorThrust4_arr,-1)
        self.MotorThrust4_arr[-1] = DashNode.MotorThrusts[3]
        self.curve_M4.setData(self.MotorThrust4_arr)


    def reset_axes(self):
        self.PW.setYRange(0,20)
        self.PW.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.MotorThrust1_arr = np.zeros(buffer_length)
        self.MotorThrust2_arr = np.zeros(buffer_length)
        self.MotorThrust3_arr = np.zeros(buffer_length)
        self.MotorThrust4_arr = np.zeros(buffer_length)

class Reward_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)


        self.PW = pg.PlotWidget(labels =  {'left':'Reward', 'bottom':'Episode [K_ep]'}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(0,25)
        self.PW.setYRange(0,300)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.curve_reward = self.PW.plot([],[], pen=None,symbol='arrow_right',symbolBrush='k',symbolSize=22)
        self.curve_reward_avg = self.PW.plot([],[], pen=None,symbol='o',symbolBrush='r')



        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(500) # number of milliseconds for next update

    def update(self):
                        
        self.curve_reward.setData(DashNode.K_ep_list1,DashNode.r_list)
        self.curve_reward_avg.setData(DashNode.K_ep_list2,DashNode.r_avg_list)



    def reset_axes(self):
        self.PW.setYRange(0,300)
        self.PW.setXRange(0,25)


class Mu_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Mu', 'bottom':'Time [s]'}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,12)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 
        self.curve_1_mu = self.PW.plot([],[],  pen=pg.mkPen(color=colors["blue"], width=width))
        self.curve_1_SD1 = self.PW.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))
        self.curve_1_SD2 = self.PW.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))


        self.curve_2_mu = self.PW.plot([],[],  pen=pg.mkPen(color=colors["orange"], width=width))
        self.curve_2_SD1 = self.PW.plot([],[], pen=pg.mkPen(color=colors["orange_alpha"]))
        self.curve_2_SD2 = self.PW.plot([],[], pen=pg.mkPen(color=colors["orange_alpha"]))


        self.PW.addItem(pg.FillBetweenItem(self.curve_1_SD1,self.curve_1_mu,colors["blue_alpha"]))
        self.PW.addItem(pg.FillBetweenItem(self.curve_1_SD2,self.curve_1_mu,colors["blue_alpha"]))

        self.PW.addItem(pg.FillBetweenItem(self.curve_2_SD1,self.curve_2_mu,colors["orange_alpha"]))
        self.PW.addItem(pg.FillBetweenItem(self.curve_2_SD2,self.curve_2_mu,colors["orange_alpha"]))


        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(500) # number of milliseconds for next update

    def update(self):
                        
        self.curve_1_mu.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list)
        self.curve_1_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + 2*np.array(DashNode.sigma_1_list))
        self.curve_1_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + -2*np.array(DashNode.sigma_1_list))

        self.curve_2_mu.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list)
        self.curve_2_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + 2*np.array(DashNode.sigma_2_list))
        self.curve_2_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + -2*np.array(DashNode.sigma_2_list))

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,12)

class Sigma_Widget(QWidget):
    def __init__(self,parent=None):
        super().__init__()

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)


        self.PW = pg.PlotWidget(name='Plot1',labels =  {'left':'Sigma', 'bottom':'Time [s]'}) # Plot window 1
        self.layout.addWidget(self.PW)
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,3)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 
        self.curve_sig1 = self.PW.plot([],[], pen=pg.mkPen(color=colors["blue"], width=width))
        self.curve_sig2 = self.PW.plot([],[], pen=pg.mkPen(color=colors["orange"], width=width))



        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(500) # number of milliseconds for next update

    def update(self):
                        
        self.curve_sig1.setData(np.arange(0,DashNode.k_ep+1),np.array(DashNode.sigma_1_list)*10)
        self.curve_sig2.setData(np.arange(0,DashNode.k_ep+1),np.array(DashNode.sigma_2_list))

    def reset_axes(self):
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,3)


if __name__ == '__main__':

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Tau_Widget()
    myApp.show()

    sys.exit(app.exec_())