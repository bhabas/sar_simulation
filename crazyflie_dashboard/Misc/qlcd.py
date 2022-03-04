from PyQt5.QtWidgets import QApplication, QDialog,QLCDNumber, QVBoxLayout, QPushButton
import sys
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QTime, QTimer
from random import randint

class Window(QDialog):
    def __init__(self):
        super().__init__()

        ## WINDOW REQUIREMENTS LIKE GEOMETRY,ICON AND TITLE
        self.setGeometry(200,200,400,200)
        self.setWindowTitle("PyQt5 QLCDNumber")
        self.setWindowIcon(QIcon("python.png"))

        ## VBOX LAYOUT OBJECT
        vbox = QVBoxLayout()

        ## CREATE QLCDNUMBER OBJECT
        self.lcd = QLCDNumber()
        vbox.addWidget(self.lcd)

        self.button = QPushButton("Create Random Number")
        self.button.clicked.connect(self.rand_generator)
        vbox.addWidget(self.button)
        self.setLayout(vbox)

    def rand_generator(self):

        ## CREATE RANDOM NUMBER BETWEEN 1 AND 300
        random = randint(1,300)
        self.lcd.display(random)




if __name__ == '__main__':
    App = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(App.exec())