from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="A 2d plot window")

p1 = pg.PlotWidget()

# try adding a 3d plot

glvw = gl.GLViewWidget()
z = pg.gaussianFilter(np.random.normal(size=(50,50)), (1,1))
p13d = gl.GLSurfacePlotItem(z=z, shader='shaded', color=(0.5, 0.5, 1, 1))
glvw.addItem(p13d)

# get a layout
layoutgb = QtGui.QGridLayout()
win.setLayout(layoutgb)

layoutgb.addWidget(glvw, 0, 0)
layoutgb.addWidget(p1, 0, 1)  ### uncommenting this line causes 
       # the plot widget to appear and the 3d widget to disappear

p1.sizeHint = lambda: pg.QtCore.QSize(100, 100)
glvw.sizeHint = lambda: pg.QtCore.QSize(100, 100)
glvw.setSizePolicy(p1.sizePolicy())

QtGui.QApplication.instance().exec_()