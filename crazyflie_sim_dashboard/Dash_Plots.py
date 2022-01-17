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
        self.x_arr[-1] = DashNode.omega[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.omega[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.omega[2]
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
        self.x_arr[-1] = DashNode.eul[0]
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.eul[1]
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.eul[2]
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


class OF_Widget(QWidget):
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
        self.x_arr[-1] = DashNode.RREV
        self.curve_x.setData(self.x_arr)

        self.y_arr = np.roll(self.y_arr,-1)
        self.y_arr[-1] = DashNode.OF_x
        self.curve_y.setData(self.y_arr)

        self.z_arr = np.roll(self.z_arr,-1)
        self.z_arr[-1] = DashNode.OF_y
        self.curve_z.setData(self.z_arr)

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
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)

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
        self.x_arr[-1] = DashNode.d_ceiling
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
        

        ## UPDATE PLOT 1
        self.PW.setBackground('w')
        self.PW.setXRange(buffer_length*0.0,buffer_length)
        self.PW.setYRange(0,65_000)
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
        self.PW.setYRange(0,65_000)
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

class RL_Widget(QWidget):
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
        self.PW.setYRange(0,200)
        self.PW.showGrid(x=True, y=True, alpha=0.2)


        # ## INIT DATA CURVES
        self.curve_reward = self.PW.plot([],[], pen=None,symbol='arrow_right',symbolBrush='k',symbolSize=22)
        self.curve_reward_avg = self.PW.plot([],[], pen=None,symbol='o',symbolBrush='r')



        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(500) # number of milliseconds for next update

    def update(self):
                        
        self.curve_reward.setData(DashNode.k_ep_list1,DashNode.r_list)
        self.curve_reward_avg.setData(DashNode.k_ep_list2,DashNode.r_avg_list)



    def reset_axes(self):
        self.PW.setYRange(0,200)
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
        self.PW.setYRange(0,16)
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
        self.curve_1_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + 2*DashNode.sigma_1_list)
        self.curve_1_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + -2*DashNode.sigma_1_list)

        self.curve_2_mu.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list)
        self.curve_2_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + 2*DashNode.sigma_2_list)
        self.curve_2_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + -2*DashNode.sigma_2_list)

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,16)

class Sigma_Widget(QWidget):
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
                        
        self.curve_sig1.setData(np.arange(0,DashNode.k_ep+1),DashNode.sigma_1_list)
        self.curve_sig2.setData(np.arange(0,DashNode.k_ep+1),DashNode.sigma_2_list)

    def reset_axes(self):
        self.PW.setXRange(0,20)
        self.PW.setYRange(0,3)


if __name__ == '__main__':

    ## INITIALIZE APPLICATION   
    app = QApplication(sys.argv)

    ## INITIALIZE DASHBOARD WINDOW
    myApp = Sigma_Widget()
    myApp.show()

    sys.exit(app.exec_())