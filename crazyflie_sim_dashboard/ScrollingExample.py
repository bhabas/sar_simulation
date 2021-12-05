# -*- coding: utf-8 -*-
"""
Various methods of drawing scrolling plots.
"""

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from time import perf_counter

win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle('pyqtgraph example: Scrolling Plots')


# 3) Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100
# Remove chunks after we have 10
maxChunks = 10
startTime = perf_counter()
win.nextRow()
plot = win.addPlot(colspan=2)
plot.setLabel('bottom', 'Time', 's')
plot.setXRange(-10, 0)




curves = []
data = np.empty((chunkSize+1,2))
ptr = 0

def update3():
    global plot, data, ptr, curves
    
    now = perf_counter()
    for c in curves:
        c.setPos(-(now-startTime), 0)
    
    i = ptr % chunkSize
    if i == 0:
        curve = plot.plot()
        curves.append(curve)
        last = data[-1]
        data = np.empty((chunkSize+1,2))        
        data[0] = last
        while len(curves) > maxChunks:
            c = curves.pop(0)
            plot.removeItem(c)
    else:
        curve = curves[-1]
    data[i+1,0] = now - startTime
    data[i+1,1] = np.random.normal()
    curve.setData(x=data[:i+2, 0], y=data[:i+2, 1])
    ptr += 1


# update all plots
def update():
    update3()
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

if __name__ == '__main__':
    pg.exec()
