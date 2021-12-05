import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from dashboard_node import DashboardNode

DashNode=DashboardNode()

FPS = 60

class  PosWidget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    
        
    
    def __init__(self,parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)

        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        p1 = self.addPlot(labels =  {'left':'Position [m]', 'bottom':'Time [s]'})
        p1.setYRange(-2,2)
        p1.addLegend()

        ## INIT DATA CURVE 1
        self.pos_x_arr = np.zeros(100)
        self.curve_pos_x = p1.plot(self.pos_x_arr, pen=pg.mkPen('r', width=1))

        self.pos_y_arr = np.zeros(100)
        self.curve_pos_y = p1.plot(self.pos_y_arr, pen=pg.mkPen('g', width=1))

        self.pos_z_arr = np.zeros(100)
        self.curve_pos_z = p1.plot(self.pos_z_arr, pen=pg.mkPen('b', width=1))

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 1
                           
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

class  VelWidget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        p1 = self.addPlot(labels =  {'left':'Velocity [m/s]', 'bottom':'Time [s]'})
        # p1.setRange(yRange=[-2,2])
        p1.setYRange(-2,4)

        ## INIT DATA CURVE 1
        self.vel_x_arr = np.zeros(100)
        self.curve_vel_x = p1.plot(self.vel_x_arr, pen=pg.mkPen('r', width=1))

        self.vel_y_arr = np.zeros(100)
        self.curve_vel_y = p1.plot(self.vel_y_arr, pen=pg.mkPen('g', width=1))

        self.vel_z_arr = np.zeros(100)
        self.curve_vel_z = p1.plot(self.vel_z_arr, pen=pg.mkPen('b', width=1))

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 1
                           
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

class  OmegaWidget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        p1 = self.addPlot(labels =  {'left':'Ang Velocity [rad/s]', 'bottom':'Time [s]'})
        p1.setYRange(-2,2)

        ## INIT DATA CURVE 1
        self.omega_x_arr = np.zeros(100)
        self.curve_omega_x = p1.plot(self.omega_x_arr, pen=pg.mkPen('r', width=1))

        self.omega_y_arr = np.zeros(100)
        self.curve_omega_y = p1.plot(self.omega_y_arr, pen=pg.mkPen('g', width=1))

        self.omega_z_arr = np.zeros(100)
        self.curve_omega_z = p1.plot(self.omega_z_arr, pen=pg.mkPen('b', width=1))

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update

    def update(self):
        self.ptr1 += 1
                           
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

class  OF_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        p1 = self.addPlot(labels =  {'left':'Ang Velocity [rad/s]', 'bottom':'Time [s]'})
        p1.setYRange(-10,10)

        ## INIT DATA CURVE 1
        self.tau_arr = np.zeros(100)
        self.curve_tau = p1.plot(self.tau_arr, pen=pg.mkPen('r', width=1))

        self.OF_x_arr = np.zeros(100)
        self.curve_OF_x = p1.plot(self.OF_x_arr, pen=pg.mkPen('g', width=1))

        self.OF_y_arr = np.zeros(100)
        self.curve_OF_y = p1.plot(self.OF_y_arr, pen=pg.mkPen('b', width=1))

        

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update
    def update(self):
        self.ptr1 += 1
                           
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

class  PWM_Widget(pg.GraphicsLayoutWidget):

    ## SET STYLE
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    
    def __init__(self, parent=None, **kargs):
        pg.GraphicsLayoutWidget.__init__(self, **kargs)
        self.ptr1 = 0
        self.setParent(parent)
        self.setWindowTitle('pyqtgraph example: Scrolling Plots')

        ## INIT PLOT WINDOW
        p1 = self.addPlot(labels =  {'left':'MS [PWM]', 'bottom':'Time [s]'})
        p1.setYRange(0,65_000)

        ## INIT DATA CURVE 1
        self.M1_arr = np.zeros(100)
        self.curve_M1 = p1.plot(self.M1_arr, pen=pg.mkPen('r', width=1))

        self.M2_arr = np.zeros(100)
        self.curve_M2 = p1.plot(self.M2_arr, pen=pg.mkPen('g', width=1))

        self.M3_arr = np.zeros(100)
        self.curve_M3 = p1.plot(self.M3_arr, pen=pg.mkPen('b', width=1))

        self.M4_arr = np.zeros(100)
        self.curve_M4 = p1.plot(self.M4_arr, pen=pg.mkPen('k', width=1))

        ## INIT UPDATE TIMER
        timer = pg.QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(int(1000/FPS)) # number of seconds (every 1000) for next update
    def update(self):
        self.ptr1 += 1
                           
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

if __name__ == '__main__':

    w = PWM_Widget()
    w.show()
    QtGui.QApplication.instance().exec_()