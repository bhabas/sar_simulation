## build a QApplication before building other widgets
import pyqtgraph as pg
import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QDial, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer
import numpy as np
import pyqtgraph.opengl as gl


class Plot3DDemo(gl.GLViewWidget):
    def __init__(self):
        gl.GLViewWidget.__init__(self)

        ## create three grids, add each to the view
        xgrid = gl.GLGridItem()
        ygrid = gl.GLGridItem()
        zgrid = gl.GLGridItem()
        self.addItem(xgrid)
        self.addItem(ygrid)
        self.addItem(zgrid)

        ## rotate x and y grids to face the correct direction
        xgrid.rotate(90, 0, 1, 0)
        ygrid.rotate(90, 1, 0, 0)

        ## scale each grid differently
        xgrid.scale(0.2, 0.1, 0.1)
        ygrid.scale(0.2, 0.1, 0.1)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # ## make a widget for displaying 3D objects
    # import pyqtgraph.opengl as gl
    # view = gl.GLViewWidget()
    # view.show()

    
    demo = Plot3DDemo()
    demo.show()

    sys.exit(app.exec_())