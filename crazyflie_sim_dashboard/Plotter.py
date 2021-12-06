import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
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
l_width = 2
FPS = 60


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
        self.p1.setYRange(0,10)
        self.p1.setXRange(0,20)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 
        self.curve_1_mu = self.p1.plot([],[],  pen=pg.mkPen(color=colors["blue"], width=l_width))
        self.curve_1_SD1 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))
        self.curve_1_SD2 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue_alpha"]))


        self.curve_2_mu = self.p1.plot([],[],  pen=pg.mkPen(color=colors["orange"], width=l_width))
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
        self.p1.setXRange(0,20)
        self.p1.setYRange(0,10)

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
        self.curve_sig1 = self.p1.plot([],[], pen=pg.mkPen(color=colors["blue"], width=l_width))
        self.curve_sig2 = self.p1.plot([],[], pen=pg.mkPen(color=colors["orange"], width=l_width))


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

        self.ptr1 = 0
        self.pause_flag = False
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Position [m]', 'bottom':'Time [s]'})
        self.p1.setYRange(-1,4)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        # p1.setAutoPan(x=True,y=True)
        # p1.showGrid(x=True, y=True, alpha=0.2)

        ## INIT DATA CURVE 1
        self.pos_x_arr = np.zeros(100)
        self.curve_pos_x = self.p1.plot(self.pos_x_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.pos_y_arr = np.zeros(100)
        self.curve_pos_y = self.p1.plot(self.pos_y_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.pos_z_arr = np.zeros(100)
        self.curve_pos_z = self.p1.plot(self.pos_z_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(50) # number of seconds (every 1000) for next update

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-1,4)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()


    def update(self):
        self.ptr1 += 0
                           
        self.pos_x_arr[:-1] = self.pos_x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.pos_x_arr[-1] = DashNode.position[0]
        self.curve_pos_x.setData(self.pos_x_arr)
        self.curve_pos_x.setPos(self.ptr1, 0)

        self.pos_y_arr[:-1] = self.pos_y_arr[1:]    # shift data in the array one sample left
        self.pos_y_arr[-1] = DashNode.position[1]
        self.curve_pos_y.setData(self.pos_y_arr)
        self.curve_pos_y.setPos(self.ptr1,0)

        self.pos_z_arr[:-1] = self.pos_z_arr[1:]    # shift data in the array one sample left
        self.pos_z_arr[-1] = DashNode.position[2]
        self.curve_pos_z.setData(self.pos_z_arr)
        self.curve_pos_z.setPos(self.ptr1,0)

class  Vel_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Velocity [m/s]', 'bottom':'Time [s]'})
        self.p1.setYRange(-1,4)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 1
        self.vel_x_arr = np.zeros(100)
        self.curve_vel_x = self.p1.plot(self.vel_x_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.vel_y_arr = np.zeros(100)
        self.curve_vel_y = self.p1.plot(self.vel_y_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.vel_z_arr = np.zeros(100)
        self.curve_vel_z = self.p1.plot(self.vel_z_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.vel_x_arr[:-1] = self.vel_x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.vel_x_arr[-1] = DashNode.velocity[0]
        self.curve_vel_x.setData(self.vel_x_arr)
        self.curve_vel_x.setPos(self.ptr1, 0)

        self.vel_y_arr[:-1] = self.vel_y_arr[1:]    # shift data in the array one sample left
        self.vel_y_arr[-1] = DashNode.velocity[1]
        self.curve_vel_y.setData(self.vel_y_arr)
        self.curve_vel_y.setPos(self.ptr1,0)

        self.vel_z_arr[:-1] = self.vel_z_arr[1:]    # shift data in the array one sample left
        self.vel_z_arr[-1] = DashNode.velocity[2]
        self.curve_vel_z.setData(self.vel_z_arr)
        self.curve_vel_z.setPos(self.ptr1,0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-1,4)

class  Omega_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Ang Velocity [rad/s]', 'bottom':'Time [s]'})
        self.p1.setYRange(-40,40)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 1
        self.omega_x_arr = np.zeros(100)
        self.curve_omega_x = self.p1.plot(self.omega_x_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.omega_y_arr = np.zeros(100)
        self.curve_omega_y = self.p1.plot(self.omega_y_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.omega_z_arr = np.zeros(100)
        self.curve_omega_z = self.p1.plot(self.omega_z_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.omega_x_arr[:-1] = self.omega_x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.omega_x_arr[-1] = DashNode.omega[0]
        self.curve_omega_x.setData(self.omega_x_arr)
        self.curve_omega_x.setPos(self.ptr1, 0)

        self.omega_y_arr[:-1] = self.omega_y_arr[1:]    # shift data in the array one sample left
        self.omega_y_arr[-1] = DashNode.omega[1]
        self.curve_omega_y.setData(self.omega_y_arr)
        self.curve_omega_y.setPos(self.ptr1,0)

        self.omega_z_arr[:-1] = self.omega_z_arr[1:]    # shift data in the array one sample left
        self.omega_z_arr[-1] = DashNode.omega[2]
        self.curve_omega_z.setData(self.omega_z_arr)
        self.curve_omega_z.setPos(self.ptr1,0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-40,40)

class  Eul_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Euler Angle [deg]', 'bottom':'Time [s]'})
        self.p1.setYRange(-90,90)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        # # Fix Axes ticks and grid
        # for key in p1.axes:
        #     ax = p1.getAxis(key)

        ## INIT DATA CURVE 1
        self.eul_x_arr = np.zeros(100)
        self.curve_eul_x = self.p1.plot(self.eul_x_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.eul_y_arr = np.zeros(100)
        self.curve_eul_y = self.p1.plot(self.eul_y_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.eul_z_arr = np.zeros(100)
        self.curve_eul_z = self.p1.plot(self.eul_z_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-90,90)

    def update(self):
        self.ptr1 += 0
                           
        self.eul_x_arr[:-1] = self.eul_x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.eul_x_arr[-1] = DashNode.eul[0]
        self.curve_eul_x.setData(self.eul_x_arr)
        self.curve_eul_x.setPos(self.ptr1, 0)

        self.eul_y_arr[:-1] = self.eul_y_arr[1:]    # shift data in the array one sample left
        self.eul_y_arr[-1] = DashNode.eul[1]
        self.curve_eul_y.setData(self.eul_y_arr)
        self.curve_eul_y.setPos(self.ptr1,0)

        self.eul_z_arr[:-1] = self.eul_z_arr[1:]    # shift data in the array one sample left
        self.eul_z_arr[-1] = DashNode.eul[2]
        self.curve_eul_z.setData(self.eul_z_arr)
        self.curve_eul_z.setPos(self.ptr1,0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()


class  OF_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'Ang Velocity [rad/s]', 'bottom':'Time [s]'})
        self.p1.setYRange(-10,10)
        self.p1.showGrid(x=True, y=True, alpha=0.2)


        ## INIT DATA CURVE 1
        self.tau_arr = np.zeros(100)
        self.curve_tau = self.p1.plot(self.tau_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.OF_x_arr = np.zeros(100)
        self.curve_OF_x = self.p1.plot(self.OF_x_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.OF_y_arr = np.zeros(100)
        self.curve_OF_y = self.p1.plot(self.OF_y_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.OF_x_arr[:-1] = self.OF_x_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.OF_x_arr[-1] = DashNode.OF_x
        self.curve_OF_x.setData(self.OF_x_arr)
        self.curve_OF_x.setPos(self.ptr1, 0)

        self.OF_y_arr[:-1] = self.OF_y_arr[1:]    # shift data in the array one sample left
        self.OF_y_arr[-1] = DashNode.OF_y
        self.curve_OF_y.setData(self.OF_y_arr)
        self.curve_OF_y.setPos(self.ptr1,0)

        self.tau_arr[:-1] = self.tau_arr[1:]    # shift data in the array one sample left
        self.tau_arr[-1] = DashNode.RREV
        self.curve_tau.setData(self.tau_arr)
        self.curve_tau.setPos(self.ptr1,0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(-10,10)

class  dist_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
        
    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)

        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'d_ceiling [m]', 'bottom':'Time [s]'})
        self.p1.setYRange(0,5)
        self.p1.showGrid(x=True, y=True, alpha=0.2)

        ## INIT DATA CURVE 1
        self.d_ceiling_arr = np.zeros(100)
        self.curve_d_ceil = self.p1.plot(self.d_ceiling_arr, pen=pg.mkPen(color=colors["red"], width=l_width))


        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.d_ceiling_arr[:-1] = self.d_ceiling_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.d_ceiling_arr[-1] = DashNode.d_ceiling
        self.curve_d_ceil.setData(self.d_ceiling_arr)
        self.curve_d_ceil.setPos(self.ptr1, 0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(0,5)

    


class  PWM_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        self.p1 = self.addPlot(labels =  {'left':'MS [PWM]', 'bottom':'Time [s]'})
        self.p1.showGrid(x=True, y=True, alpha=0.2)
        self.p1.setYRange(0,65_000)

        ## INIT DATA CURVE 1
        self.M1_arr = np.zeros(200)
        self.curve_M1 = self.p1.plot(self.M1_arr, pen=pg.mkPen(color=colors["red"], width=l_width))

        self.M2_arr = np.zeros(200)
        self.curve_M2 = self.p1.plot(self.M2_arr, pen=pg.mkPen(color=colors["green"], width=l_width))

        self.M3_arr = np.zeros(200)
        self.curve_M3 = self.p1.plot(self.M3_arr, pen=pg.mkPen(color=colors["blue"], width=l_width))

        self.M4_arr = np.zeros(200)
        self.curve_M4 = self.p1.plot(self.M4_arr, pen=pg.mkPen(color=colors["orange"], width=l_width))

        ## INIT UPDATE TIMER
        self.timer = pg.QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 0
                           
        self.M1_arr[:-1] = self.M1_arr[1:]  # shift data in the array one sample left  # (see also: np.roll)
        self.M1_arr[-1] = DashNode.MS_PWM[0]
        self.curve_M1.setData(self.M1_arr)
        self.curve_M1.setPos(self.ptr1, 0)

        self.M2_arr[:-1] = self.M2_arr[1:]    # shift data in the array one sample left
        self.M2_arr[-1] = DashNode.MS_PWM[1]
        self.curve_M2.setData(self.M2_arr)
        self.curve_M2.setPos(self.ptr1,0)

        self.M3_arr[:-1] = self.M3_arr[1:]    # shift data in the array one sample left
        self.M3_arr[-1] = DashNode.MS_PWM[3]
        self.curve_M3.setData(self.M3_arr)
        self.curve_M3.setPos(self.ptr1,0)

        self.M4_arr[:-1] = self.M4_arr[1:]    # shift data in the array one sample left
        self.M4_arr[-1] = DashNode.MS_PWM[3]
        self.curve_M4.setData(self.M4_arr)
        self.curve_M4.setPos(self.ptr1,0)

    def pause(self,pause_flag):
        if pause_flag == True:
            self.timer.stop()
        else: 
            self.timer.start()

    def reset_axes(self):
        self.p1.enableAutoRange(enable=True)
        self.p1.setYRange(0,65_000)

if __name__ == '__main__':

    w = Mu_Widget()
    w.show()
    QtGui.QApplication.instance().exec_()