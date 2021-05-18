#!/usr/bin/env python

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
from numpy import *
from PyQt5.QtWidgets import QApplication,QTabWidget, QWidget, QPushButton, QHBoxLayout,QVBoxLayout,QSlider,QLabel,QMessageBox,QListWidget,QTextEdit
import serial
from cubic_spline_planner import *
from stanley_controller import *

raw=serial.Serial("COM5",115200)
# raw.open()
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
rl = ReadLine(raw)             

    

class MyWidget(pg.GraphicsWindow):

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.windowWidth = 100                       # width of the window displaying the curve
        self.Xm = linspace(0,0,self.windowWidth)          # create array that will contain the relevant time series     
        self.ptr = -self.windowWidth   
                                                     # width of the window displaying the curve
        self.Xm1 = linspace(0,0,self.windowWidth)          # create array that will contain the relevant time series     
        self.ptr1 = -self.windowWidth  

        self.mainLayout = QtWidgets.QVBoxLayout()



        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(500) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        self.plotItem = self.addPlot(title="")

        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255,0,0), symbolSize=5, symbolPen=None)
        self.plotDataItem1 = self.plotItem.plot([], pen=None, 
            symbolBrush=(0,255,0), symbolSize=5, symbolPen=None)
        ax = [0.0, 0.5, 1, 1.5, 2.0]
        ay = [0.0, 0.0, 0, 0, 0.0]


        self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        self.setData(self.cx, self.cy)
        self.target_speed = 30.0 / 3.6  # [m/s]
       
        self.max_simulation_time = 100.0

            # Initial state
        self.state = State(x=-0.0, y=0.0, yaw=np.radians(0.0), v=0.0)

        self.last_idx = len(self.cx) - 1
        self.time = 0.0
        self.x = [self.state.x]
        self.y = [self.state.y]
        self.yaw = [self.state.yaw]
        self.v = [self.state.v]
        self.t = [0.0]
        self.robot_pos_x = []
        self.robot_pos_y = []
        self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
        self.setData1(self.x, self.y)
    def setData(self, x, y):
        self.plotDataItem.setData(x, y)
    def setData1(self, x, y):
        self.plotDataItem1.setData(x, y)
    di = 0
    def onNewData(self):
        
        value = rl.readline()             # read line (single value) from the serial port
        try:
            X =  (value.decode().replace('\x00','').replace('\x00','')).split(' ')
            
            # for i in X:

            self.robot_pos_x.append(float(X[0]))  
            self.robot_pos_y.append(float(X[1]))
            di += np.pi/60
            send_control(int(np.degrees(0)+90),100)
        #     # data.append(x_[1])
           
        
        #     if self.max_simulation_time >= self.time and self.last_idx > self.target_idx:
        #         ai = pid_control(self.target_speed, self.state.v)
        #         
        #         self.state.update(ai, di)
        
        #         self.time += dt
        #         print(int(np.degrees(di)+90))
        #         self.x.append(self.state.x)
        #         self.y.append(self.state.y)
        #         self.yaw.append(self.state.yaw)
        #         self.v.append(self.state.v)
        #         self.t.append(self.time)
        #         # self.setData1(self.x, self.y)
        #     self.setData1(self.robot_pos_x, self.robot_pos_y)
            # self.Xm[:-1] = self.Xm[1:]                      # shift data in the temporal mean 1 sample left
            # self.Xm1[:-1] = self.Xm1[1:]

            # self.Xm[-1] = x_[0]                 # vector containing the instantaneous values 
            # self.Xm1[-1] = -x_[1]
        except Exception as e:
            print(e)
            # data.append(data[len(data) - 1])
        
def control(button):
    if(button ==1):
        raw.write(b'w')
    if(button ==2):
        raw.write(b'x')
    if(button ==3):
        raw.write(b'd')
    if(button ==4):
        raw.write(b'd')
    if(button ==5):
        raw.write(b's')
    
def send_control(angle,velocity):
    raw.write(b's')
    raw.write(bytes([int(angle / 10)]))
    raw.write(bytes([angle % 10]))
    raw.write(bytes([velocity]))
    raw.write(bytes([(velocity + angle) % 37]))
    raw.write(b'e') 
import threading


connected = False
port = 'COM5'
baud = 115200

# serial_port = serial.Serial(port, baud, timeout=0)

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

def main():
    app = QtWidgets.QApplication([])

 
    pg.setConfigOptions(antialias=False) # True seems to work as well
    main_layout = QVBoxLayout()
    win_app = QWidget()

    # b_up = QPushButton('U')
    # b_down = QPushButton('D')
    # b_right = QPushButton('R')
    # b_left = QPushButton('F')
    # b_stop = QPushButton('S')

    # b_up.clicked.connect(lambda: control(1))
    # b_down.clicked.connect(lambda: control(2))    
    # b_right.clicked.connect(lambda: control(3))
    # b_left.clicked.connect(lambda: control(4))
    # b_stop.clicked.connect(lambda: control(5))

    win = MyWidget()
    # main_layout.addWidget(b_up)
    # main_layout.addWidget(b_down)
    # main_layout.addWidget(b_stop)
    # main_layout.addWidget(b_left)
    # main_layout.addWidget(b_right)
    main_layout.addWidget(win) 
    win_app.setLayout(main_layout)

    
    win_app.show()
    win_app.resize(800,600) 
    win_app.raise_()
    app.exec_()

if __name__ == "__main__":
    main()