import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from pyqtgraph.graphicsItems.GraphicsWidget import GraphicsWidget
from dashboard_node import DashboardNode
import rospy
from crazyflie_msgs.msg import RLData

DashNode=DashboardNode()

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



class CustomWidget(pg.GraphicsWindow):
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    ptr1 = 0
    def __init__(self, parent=None, **kargs):
        pg.GraphicsWindow.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')
        p1 = self.addPlot(labels =  {'left':'Voltage', 'bottom':'Time'})
        self.data1 = np.random.normal(size=10)
        self.curve1 = p1.plot(self.data1, pen=(3,3))

        self.data2 = np.random.normal(size=10)
        self.curve2 = p1.plot(self.data2, pen=(2,3))

        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(17) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.data1[:-1] = self.data1[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.data1[-1] = np.random.normal()
        self.curve1.setData(self.data1)
        self.curve1.setPos(self.ptr1, 0)

        self.data2[:-1] = self.data2[1:]    # shift data in the array one sample left
        self.data2[-1] = np.random.normal()
        self.curve2.setData(self.data2)
        self.curve2.setPos(self.ptr1,0)


class RL_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)


    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)

        self.setParent(parent)
        p1 = self.addPlot(labels =  {'left':'Reward', 'bottom':'Episode [K_ep]'})
        p1.setYRange(0,200)
        p1.setXRange(0,25)
        p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 1
        self.curve_reward = p1.plot([],[], pen=None,symbol='arrow_right',symbolBrush='k',symbolSize=22)
        self.curve_reward_avg = p1.plot([],[], pen=None,symbol='o',symbolBrush='r')


        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1000) # number of seconds (every 1000) for next update

    def update(self):
        self.curve_reward.setData(DashNode.k_ep_list1,DashNode.r_list)
        self.curve_reward_avg.setData(DashNode.k_ep_list2,DashNode.r_avg_list)

class Mu_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)

        self.setParent(parent)
        self.p1 = self.addPlot(labels =  {'left':'Mu', 'bottom':'Episode [K_ep]'})
        self.p1.setYRange(0,16)
        self.p1.setXRange(0,20)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 
        self.curve_1_mu = self.p1.plot([],[],  pen=pg.mkPen(color=colors["blue"], width=width))
        self.curve_1_SD1 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))
        self.curve_1_SD2 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))


        self.curve_2_mu = self.p1.plot([],[],  pen=pg.mkPen(color=colors["orange"], width=width))
        self.curve_2_SD1 = self.p1.plot([],[], pen=pg.mkPen(color=colors["orange_alpha"]))
        self.curve_2_SD2 = self.p1.plot([],[], pen=pg.mkPen(color=colors["orange_alpha"]))

        

        self.p1.addItem(pg.FillBetweenItem(self.curve_1_SD1,self.curve_1_mu,colors["blue_alpha"]))
        self.p1.addItem(pg.FillBetweenItem(self.curve_1_SD2,self.curve_1_mu,colors["blue_alpha"]))

        self.p1.addItem(pg.FillBetweenItem(self.curve_2_SD1,self.curve_2_mu,colors["orange_alpha"]))
        self.p1.addItem(pg.FillBetweenItem(self.curve_2_SD2,self.curve_2_mu,colors["orange_alpha"]))

        # rospy.wait_for_message('/rl_data',RLData)

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1000) # number of seconds (every 1000) for next update

    def update(self):

        self.curve_1_mu.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list)
        self.curve_1_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + 2*DashNode.sigma_1_list)
        self.curve_1_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_1_list + -2*DashNode.sigma_1_list)

        self.curve_2_mu.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list)
        self.curve_2_SD1.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + 2*DashNode.sigma_2_list)
        self.curve_2_SD2.setData(np.arange(0,DashNode.k_ep+1),DashNode.mu_2_list + -2*DashNode.sigma_2_list)
    
    def reset_axes(self):
        self.p1.setYRange(0,16)
        self.p1.setXRange(0,20)

class Sig_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)


    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)

        self.setParent(parent)
        self.p1 = self.addPlot(labels =  {'left':'Sigma', 'bottom':'Episode [K_ep]'})
        self.p1.setYRange(0,3)
        self.p1.setXRange(0,20)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 
        self.curve_sig1 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue"], width=width))
        self.curve_sig2 = self.p1.plot([],[], pen=pg.mkPen(color=colors["orange"], width=width))


        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(1000) # number of seconds (every 1000) for next update

    def update(self):
        self.curve_sig1.setData(np.arange(0,DashNode.k_ep+1),DashNode.sigma_1_list)
        self.curve_sig2.setData(np.arange(0,DashNode.k_ep+1),DashNode.sigma_2_list)

    def reset_axes(self):
        self.p1.setXRange(0,20)
        self.p1.setYRange(0,3)



class  Pos_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')


        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Position [m]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(-1,4)
        self.p1.setXRange(buffer_length*0.0,buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.p1.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.p1.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-1,4)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()
    
    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)


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


class  Vel_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')


        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Velocity [m/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(-1,4)
        self.p1.setXRange(buffer_length*(0.0),buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        
        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.p1.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.p1.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-1,4)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)


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


class  Omega_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')


        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Ang. Vel. [rad/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(-40,40)
        self.p1.setXRange(buffer_length*0.0,buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.p1.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.p1.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-40,40)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)

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

class  Eul_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')


        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## UPDATE Y-AXIS TICKS
        # tick_labels = {-90:'-90',-45:'-45',0:'0'}
        # ax = pg.AxisItem(orientation='left')
        # ax.setTicks([tick_labels.items()])


        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Eul Angle [deg]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(-90,90)
        self.p1.setXRange(buffer_length*0.0,buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.p1.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.p1.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-90,90)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)

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

class  OF_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        
        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'OF [rad/s]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(-10,10)
        self.p1.setXRange(buffer_length*0.0,buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.y_arr = np.zeros(buffer_length)
        self.curve_y = self.p1.plot(self.y_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.z_arr = np.zeros(buffer_length)
        self.curve_z = self.p1.plot(self.z_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-10,10)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)
        self.y_arr = np.zeros(buffer_length)
        self.z_arr = np.zeros(buffer_length)

    def update(self):
                           
        self.x_arr[:-1] = self.x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.RREV
        self.curve_x.setData(self.x_arr)

        self.y_arr[:-1] = self.y_arr[1:]    # shift data in the array one sample left
        self.y_arr[-1] = DashNode.OF_x
        self.curve_y.setData(self.y_arr)

        self.z_arr[:-1] = self.z_arr[1:]    # shift data in the array one sample left
        self.z_arr[-1] = DashNode.OF_y
        self.curve_z.setData(self.z_arr)


class  dist_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Distance [m]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(0,5)
        self.p1.setXRange(buffer_length*0.0,buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.x_arr = np.zeros(buffer_length)
        self.curve_x = self.p1.plot(self.x_arr, pen=pg.mkPen(color=colors["red"], width=width))


        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(0,5)
        self.p1.setXRange(buffer_length*0.0,buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.x_arr = np.zeros(buffer_length)


    def update(self):
                           
        self.x_arr = np.roll(self.x_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.x_arr[-1] = DashNode.d_ceiling
        self.curve_x.setData(self.x_arr)


class  PWM_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        
        ## DEFINE HOW LONG TO PRESERVE DATA
        self.buffer_time = 20_000                               # Amount of data to save [ms]
        update_interval = 1_000//refresh_rate                        # [ms]
        self.buffer_length = self.buffer_time//update_interval   # Number of datapoints in plot

        ## UPDATE X-AXIS TICKS
        num_ticks = 11
        buff_ticks = np.linspace(0,self.buffer_length,num_ticks)         # Tick locations
        time_ticks = np.linspace(-self.buffer_time//1000,0,num_ticks)    # Tick values
        tick_labels = dict(zip(buff_ticks,['{:.1f}'.format(tick) for tick in time_ticks]))


        ax = pg.AxisItem(orientation='bottom')
        ax.setTicks([tick_labels.items()])

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Motor Command [PWM]', 'bottom':'Time [s]'},axisItems={'bottom': ax})
        self.p1.setYRange(0,65_000)
        self.p1.setXRange(self.buffer_length*0.0,self.buffer_length)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        

        # ## INIT DATA CURVES
        self.M1_arr = np.zeros(self.buffer_length)
        self.curve_M1 = self.p1.plot(self.M1_arr, pen=pg.mkPen(color=colors["red"], width=width))

        self.M2_arr = np.zeros(self.buffer_length)
        self.curve_M2 = self.p1.plot(self.M2_arr, pen=pg.mkPen(color=colors["green"], width=width))

        self.M3_arr = np.zeros(self.buffer_length)
        self.curve_M3 = self.p1.plot(self.M3_arr, pen=pg.mkPen(color=colors["blue"], width=width))

        self.M4_arr = np.zeros(self.buffer_length)
        self.curve_M4 = self.p1.plot(self.M4_arr, pen=pg.mkPen(color=colors["orange"], width=width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(update_interval) # number of milliseconds for next update

    def reset_axes(self):
        # self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(0,65_000)
        self.p1.setXRange(self.buffer_length*0.0,self.buffer_length)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def clear_data(self):
        self.M1_arr = np.zeros(self.buffer_length)
        self.M2_arr = np.zeros(self.buffer_length)
        self.M3_arr = np.zeros(self.buffer_length)
        self.M4_arr = np.zeros(self.buffer_length)

    def update(self):
                           
        self.M1_arr = np.roll(self.M1_arr,-1) # shift data in the array one sample left  # (see also: np.roll)
        self.M1_arr[-1] = DashNode.MS_PWM[0]
        self.curve_M1.setData(self.M1_arr)

        self.M2_arr = np.roll(self.M2_arr,-1)
        self.M2_arr[-1] = DashNode.MS_PWM[1]
        self.curve_M2.setData(self.M2_arr)

        self.M3_arr = np.roll(self.M3_arr,-1)
        self.M3_arr[-1] = DashNode.MS_PWM[2]
        self.curve_M3.setData(self.M3_arr)

        self.M4_arr = np.roll(self.M4_arr,-1)
        self.M4_arr[-1] = DashNode.MS_PWM[3]
        self.curve_M4.setData(self.M4_arr)

        


if __name__ == '__main__':

    w = PWM_Widget()
    w.show()
    QtGui.QApplication.instance().exec()