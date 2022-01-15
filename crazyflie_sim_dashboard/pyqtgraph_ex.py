import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QDial, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer
import numpy as np

class PlotDemo(QWidget):

    def __init__(self):
        super().__init__()

        ## INITIALIZE WINDOW SIZE
        self.setWindowTitle('pyqtgraph example: PlotWidget')
        self.resize(800,800)

        ## INIT LAYOUT
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)


        ## INIT PLOTS
        self.pw1 = pg.PlotWidget(name='Plot1') # Plot window 1
        self.pw2 = pg.PlotWidget(name='Plot2')
        self.pw3 = pg.PlotWidget(name='Plot3')
        self.layout.addWidget(self.pw1)
        self.layout.addWidget(self.pw2)
        self.layout.addWidget(self.pw3)

        ## UPDATE PLOT 1
        self.pw1.setLabel('left', 'Value', units='V')
        self.pw1.setLabel('bottom', 'Time', units='s')
        self.pw1.setXRange(0, 2)
        self.pw1.setYRange(0, 1e-10)

        self.p1 = self.pw1.plot() # Plot Window 1 -> plot
        self.p1.setPen((200,200,100))


        ## START TIMER TO RAPIDLY UPDATE PLOT IN PW1
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateData)
        self.timer.start(50)

        ## UPDATE PLOT 2
        for i in range(0, 5):
            for j in range(0, 3):
                yd, xd = self.rand(10000)
                self.pw2.plot(y=yd*(j+1), x=xd, params={'iter': i, 'val': j})

        ## UPDATE PLOT 3
        self.p3 = self.pw3.plot(np.random.normal(size=100),clickable=True)
        self.p3.curve.setClickable(True)
        self.p3.setPen('w')
        self.p3.setShadowPen(pg.mkPen((70,70,30),width=6, cosmetic=True))
        self.p3.sigClicked.connect(self.clicked)

        lr = pg.LinearRegionItem([1,30],bounds=[0,100],movable=True)
        self.pw3.addItem(lr)

        line = pg.InfiniteLine(angle=45, movable=True)
        self.pw3.addItem(line)
        line.setBounds([0,200])


    def clicked(self):
        print("Plot 3 clicked")

    def rand(self,n):
        data = np.random.random(n)
        data[int(n*0.1):int(n*0.13)] += .5
        data[int(n*0.18)] += 2
        data[int(n*0.1):int(n*0.13)] *= 5
        data[int(n*0.18)] *= 20
        data *= 1e-12
        return data, np.arange(n, n+len(data)) / float(n)

    def updateData(self):
        yd, xd = self.rand(10000)
        self.p1.setData(y=yd, x=xd)






if __name__ == '__main__':
    app = QApplication(sys.argv)
    demo = PlotDemo()
    demo.show()

    sys.exit(app.exec_())