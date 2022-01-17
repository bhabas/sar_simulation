import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QSizePolicy, QSlider, QSpacerItem, \
    QVBoxLayout, QWidget

import pyqtgraph as pg
import numpy as np
import pyqtgraph.opengl as gl



class Slider(QWidget):
    def __init__(self, minimum, maximum, parent=None):
        super(Slider, self).__init__(parent=parent)
        self.verticalLayout = QVBoxLayout(self)
        self.label = QLabel(self)
        self.verticalLayout.addWidget(self.label)
        self.horizontalLayout = QHBoxLayout()
        spacerItem = QSpacerItem(0, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.slider = QSlider(self)
        self.slider.setOrientation(Qt.Vertical)
        self.horizontalLayout.addWidget(self.slider)
        spacerItem1 = QSpacerItem(0, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.resize(self.sizeHint())

        self.minimum = minimum
        self.maximum = maximum
        self.slider.valueChanged.connect(self.setLabelValue)
        self.x = None
        self.setLabelValue(self.slider.value())

    def setLabelValue(self, value):
        self.x = self.minimum + (float(value) / (self.slider.maximum() - self.slider.minimum())) * (
        self.maximum - self.minimum)
        self.label.setText("{0:.4g}".format(self.x))


class Widget(QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent=parent)
        self.horizontalLayout = QHBoxLayout(self)
        self.w1 = Slider(0, 10)
        self.horizontalLayout.addWidget(self.w1)

        self.w2 = Slider(0, 10)
        self.horizontalLayout.addWidget(self.w2)

        self.w3 = Slider(0, 10)
        self.horizontalLayout.addWidget(self.w3)

        self.win = gl.GLViewWidget()
        self.horizontalLayout.addWidget(self.win)
        # self.p6 = self.win.addPlot(title="My Plot")
        # self.p6.setYRange(0,25)
        # self.p6.setXRange(0,25)

        # self.curve = self.p6.plot(pen='r')
        # self.update_plot()

        # self.w1.slider.valueChanged.connect(self.update_plot)
        # self.w2.slider.valueChanged.connect(self.update_plot)
        # self.w3.slider.valueChanged.connect(self.update_plot)

    def update_plot(self):
        vx = self.w1.x
        vz = self.w2.x
        c = self.w3.x
        t = np.linspace(0, 7, 100)
        self.curve.setData(vx*t,vz*t)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(app.exec_())