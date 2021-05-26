
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
from numpy import *
from PyQt5.QtWidgets import QApplication,QTabWidget, QWidget, QPushButton, QHBoxLayout,QVBoxLayout,QSlider,QLabel,QMessageBox,QListWidget,QTextEdit
import serial
from QTImageView import QtImageViewer
from PyQt5 import QtWidgets, uic
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import sys
from PyQt5.QtGui import QPixmap, QPainter, QPen, QBrush, QColor, QBrush
from PyQt5.QtWidgets import QWidget, QLabel, QApplication, QHBoxLayout, QDialog, QGraphicsEllipseItem
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
import cv2
import sys  # We need sys so that we can pass argv to QApplication
import os


from cubic_spline_planner import *
from stanley_controller import *
from list_port import *
from connect import *
from zmq_read import *
# raw=serial.Serial("COM5",115200)
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
# rl = ReadLine(raw)             
ser = serial.Serial()
rl = ReadLine(ser)
detached = False
portName ='COM5'    

class MyWidget(pg.ViewBox):

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
        self.timer.setInterval(300) # in milliseconds
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

        self.setXRange(0, 10)
        self.setYRange(0, 10)
    def setData(self, x, y):
        self.plotDataItem.setData(x, y)
    def setData1(self, x, y):
        self.plotDataItem1.setData(x, y)
    di = 0
    def onNewData(self):
        global rl, ser
            # NB: for PySerial v3.0 or later, use property `in_waiting` instead of function `inWaiting()` below!
        if (ser.in_waiting>0):
            
            value = rl.readline() #read the bytes and convert from binary array to ASCII
        # print(data_str, end='') #print the incoming       # read line (single value) from the serial port
            try:
                X =  (value.replace('\x00','').replace('\x00','')).split(' ')
                
                # for i in X:
                print(float(X[0]),float(X[1]))
                self.robot_pos_x.append(float(X[0]))  
                self.robot_pos_y.append(float(X[1]))
                # di += np.pi/604
                self.state.update_real(float(X[0]),float(X[1]),float(X[2]),0)
                delta, current_target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
                print(delta, current_target_idx)
    
                if(float(X[0])< 0.5):
                    send_control(int(np.degrees(0)+90),100)
                else:
                    send_control(int(np.degrees(0)+90),0)
            #     # data.append(x_[1])
            
            
            #     if self.max_simulation_time >= self.time and self.last_idx > self.target_idx:
                ai = pid_control(self.target_speed, self.state.v)
                    
                #self.state.update(ai, di)
            
            #         self.time += dt
            #         print(int(np.degrees(di)+90))
            #         self.x.append(self.state.x)
            #         self.y.append(self.state.y)
            #         self.yaw.append(self.state.yaw)
            #         self.v.append(self.state.v)
            #         self.t.append(self.time)
            #         # self.setData1(self.x, self.y)
                self.setData1(self.robot_pos_x, self.robot_pos_y)
                # self.Xm[:-1] = self.Xm[1:]                      # shift data in the temporal mean 1 sample left
                # self.Xm1[:-1] = self.Xm1[1:]

                # self.Xm[-1] = x_[0]                 # vector containing the instantaneous values 
                # self.Xm1[-1] = -x_[1]
            except Exception as e:
                print(e)
                # data.append(data[len(data) - 1])
        
# def control(button):
#     if(button ==1):
#         ser.write(b'w')
#     if(button ==2):
#         raw.write(b'x')
#     if(button ==3):
#         raw.write(b'd')
#     if(button ==4):
#         raw.write(b'd')
#     if(button ==5):
#         raw.write(b's')
    
def send_control(angle,velocity):
    global ser
    ser.write(b's')
    ser.write(bytes([int(angle / 10)]))
    ser.write(bytes([angle % 10]))
    ser.write(bytes([velocity]))
    ser.write(bytes([(velocity + angle) % 37]))
    ser.write(b'e') 



class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        #Load the UI Page
        uic.loadUi('main_window.ui', self)

        self.serial_list = list_serial_ports()

        for (i,ser_name) in enumerate(self.serial_list):
            self.listWidget.insertItem(i, ser_name)
        self.listWidget.clicked.connect(self.list_clicked)
        # self.plot([1,2,3,4,5,6,7,8,9,10], [30,32,34,32,33,31,29,32,35,45])
        self.pushButton.clicked.connect(self.connectCOM)
        self.pushButton_2.clicked.connect(self.detachCOM)
        
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(10) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)

        #Show map #######################################################################
        self.drawing = False
        self.lastPoint = QPoint()
        self.map_image = QPixmap("map_png.PNG")
        self.map_png = cv2.imread("./map_png.png")


        self.widget_2.aspectRatioMode = Qt.KeepAspectRatio
        self.widget_2.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.widget_2.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.widget_2.canZoom = True
        # Allow panning with left mouse button.
        self.widget_2.canPan = True
        self.widget_2.setImage(self.map_image)
        self.widget_2.leftMouseButtonPressed.connect(self.handleLeftClick)

        self.check_point = []

        ax = [0.0, 2, 4, 5, 10.0]
        ay = [0.0, 0.0, 0, 0, 0.0]

        
        self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        # self.setData(self.cx, self.cy
        
        self.target_speed = 30.0 / 3.6  # [m/s]
        ##################################################################################   
        self.state = State(x=-0.0, y=0.0, yaw=np.radians(0.0), v=0.0)
        self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
        
    di = 0
        #Handle map click   ######################
    def handleLeftClick(win,x, y):
        row = int(y)
        column = int(x)
        print("Pixel (row="+str(row)+", column="+str(column)+")")
        scene = win.viewer.scene

        item = QGraphicsEllipseItem(x, y, 10, 10)
        item.setPen(QPen(QColor("blue")))
        item.setBrush(QBrush(QColor("blue")))
        scene.addItem(item)
        
        gps = pix2global_gui(column,row)
        win.check_point.append(gps)
        
        i = global2pix_gui(gps[0],gps[1])
        # print(i)
        item = QGraphicsEllipseItem(i[1], i[0], 10, 10)
        item.setPen(QPen(QColor("red")))
        item.setBrush(QBrush(QColor("red")))
        scene.addItem(item)

        win.viewer.setScene(scene)
        
        print(pix2global_gui(x,y))


    def list_clicked(self):
        global portName
        portName = self.listWidget.currentItem().text()
    def list_clicked1(self):
        global portName
        portName = self.listWidget.currentItem().text()

    def onNewData(self):
        global rl, ser
            # NB: for PySerial v3.0 or later, use property `in_waiting` instead of function `inWaiting()` below!
        if (ser.in_waiting>0):
            
            value = rl.readline()#read the bytes and convert from binary array to ASCII
        # print(data_str, end='') #print the incoming       # read line (single value) from the serial port
            try:
                X =  (value.decode().replace('\x00','').replace('\x00','')).split(' ')


                self.state.update_real(float(X[0]),float(X[1]),float(X[2]),0)
                delta, current_target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
                print(delta, current_target_idx)
                # for i in X:
                print(float(X[0]),float(X[1]))
                self.widget.robot_pos_x.append(float(X[0]))  
                self.widget.robot_pos_y.append(float(X[1]))
                # di += np.pi/604

                self.state.update_real(float(X[0]),float(X[1]),float(X[2]),0)
                # delta, current_target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.last_target_idx)
                if(float(X[0])< 5):
                    send_control(int(np.degrees(0)+90),100)
                else:
                    send_control(int(np.degrees(0)+90),0)
                # data.append(x_[1])
            
            
            #     if self.max_simulation_time >= self.time and self.last_idx > self.target_idx:
                # ai = pid_control(self.target_speed, self.state.v)
                    
                #self.state.update(ai, di)
            
            #         self.time += dt
            #         print(int(np.degrees(di)+90))
            #         self.x.append(self.state.x)
            #         self.y.append(self.state.y)
            #         self.yaw.append(self.state.yaw)
            #         self.v.append(self.state.v)
            #         self.t.append(self.time)
            #         # self.setData1(self.x, self.y)
                self.widget.setData1(self.widget.robot_pos_x, self.widget.robot_pos_y)
                # self.Xm[:-1] = self.Xm[1:]                      # shift data in the temporal mean 1 sample left
                # self.Xm1[:-1] = self.Xm1[1:]

                # self.Xm[-1] = x_[0]                 # vector containing the instantaneous values 
                # self.Xm1[-1] = -x_[1]
            except Exception as e:
                print(e)
            # data.append(data[len(data) - 1])          
    
    def detachCOM(self):
        global ser, connect_check
        connect_check = False
        ser.close() 

    def connectCOM(self):
        try:                     # replace this port name by yours!
            baudrate = 115200
            global ser,portName,detached, rl, connect_check
            ser = serial.Serial(portName,baudrate)
            rl = ReadLine(ser)
            print("ok")
            
            if detached:
                detached = False
            alert = QMessageBox()
            connect_check = True
            alert.setText('Connect OK!')
                    # ser.open()

        except Exception as e: 
            print(e)
            alert = QMessageBox()
            alert.setText('Error connect COM!')
                
            alert.exec_()      
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
    global ser
    ser.write(b's')
    ser.write(bytes([int(angle / 10)]))
    ser.write(bytes([angle % 10]))
    ser.write(bytes([velocity]))
    ser.write(bytes([(velocity + angle) % 37]))
    ser.write(b'e') 
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
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()