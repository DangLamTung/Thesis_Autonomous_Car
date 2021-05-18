import sys
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtGui import QPixmap, QPainter, QPen
import serial

import _thread
import utils
# import gpsmap
from QTImageView import QtImageViewer
from connect import *
from zmq_read import *
# from waypoint_gui import Ui_Dialog
import queue
import time # Optional (if using time.sleep() below)
connected = False
port = 'COM5'
baud = 115200
try:
    ser = serial.Serial(port, baud, timeout=0)
except Exception as e:
    print(e)
import threading
connected = False
def handle_data(data):
    print(data)

def read_from_port(ser):
    global connected
    while not connected:
        #serin = ser.read()
        connected = True

        while True:
        #    print("test")
           reading = ser.readline().decode()
           handle_data(reading)
           time.sleep(0.2)


class Menu(QMainWindow):

    def __init__(self):
        super().__init__()
        self.drawing = False
        self.lastPoint = QPoint()
        self.image = QPixmap("map_png.PNG")
        self.setGeometry(100, 100, 500, 300)
        self.resize(self.image.width(), self.image.height())
        self.show()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.image)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.lastPoint = event.pos()

    def mouseMoveEvent(self, event):
        if event.buttons() and Qt.LeftButton and self.drawing:
            painter = QPainter(self.image)
            painter.setPen(QPen(Qt.red, 3, Qt.SolidLine))
            painter.drawLine(self.lastPoint, event.pos())
            self.lastPoint = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button == Qt.LeftButton:
            self.drawing = False


if __name__ == '__main__':
    app = QApplication(sys.argv)
    try:
        thread = threading.Thread(target=read_from_port, args=(ser,))
        thread.start()
    except Exception as e:
        print(e)
    mainMenu = Menu()

    sys.exit(app.exec_())

    