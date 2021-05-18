import sys
from numpy import *

import pyqtgraph as pg
import serial
from time import sleep
import numpy as np
import crc8
import time
import struct
from list_port import list_serial_ports
from threading import Thread
import threading
from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal)

import ekf_layout
import pid_layout
import control_layout

from PyQt5.QtWidgets import QApplication,QTabWidget, QWidget, QPushButton, QHBoxLayout,QVBoxLayout,QSlider,QLabel,QMessageBox,QListWidget,QTextEdit
current_time = 0
connect_check = False
value = []


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
def advance_uart(value):
    data = []
    b = []
    b3 = value[15]
    b4 = value[19]
    b5 = value[23]
    
    temp = (b5 + b4 + b3) % 37
    crc = value[24]
    print(crc)  
    # struct.unpack('f', b)
    if(crc == temp):
        # print("OK")
        for i in range(6):
            b = value[4*i:4*(i+1)]
            data.append(round(float(struct.unpack('f',b)[0]),3))
        print(data)
    else:
        print("not OK")
    return data


ser = serial.Serial()
rl = ReadLine(ser)

lay = ekf_layout.EKF_layout()
lay_pid = pid_layout.PID_layout()
lay_control = control_layout.control_layout()

portName = "COM40" 
detached = False
plot_control = False
def listToString(s):  
    
    # initialize an empty string 
    str1 = ""  
    
    # traverse in the string   
    for ele in s:  
        str1 += ele   
    
    # return string   
    return str1  

main_layout = QVBoxLayout()
main_layout_1 = QVBoxLayout()
main_layout_2 = QVBoxLayout()
main_layout_2 = QVBoxLayout()
def setPID_all():
    global ser
    try: 
        values = []
        sleep(0.05)
        data = []
        data1 = []
        start = 's'
        end = 'e'
        CRC = ""
        value = int(format(lay.slider4.value(),'04'))

        esc1 = bin(value)
        esc2 = bin(value)
        esc3 = bin(value)
        esc4 = bin(value)
        
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc1)+2)]  + esc1[2:len(esc1)]
        print(len(esc))   
        b1 = esc
        #value 2
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc2)+2)]  + esc2[2:len(esc2)]
        b2 = esc
        #value 3
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc3)+2)]  + esc3[2:len(esc3)]
        b3 = esc
        
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc4)+2)]  + esc4[2:len(esc4)]
        b4 = esc 
        data_frame = b1+ b2 + b3 +b4

        
        frame1 = '0b' + data_frame[0:8]
        frame2 = '0b' + data_frame[8:16]
        frame3 = '0b' + data_frame[16:24]
        frame4 = '0b' + data_frame[24:32]
        frame5 = '0b' + data_frame[32:40]
        frame6 = '0b' + data_frame[40:44] + '0000'
        
        frame_check = 4*value %37
         
        data.append(int(frame1,2))
        data.append(int(frame2,2))
        data.append(int(frame3,2))
        data.append(int(frame4,2))
        data.append(int(frame5,2))
        data.append(int(frame6,2))
        data.append(frame_check)
      
        lay.label4.setText("All Motor \n" + str(lay.slider4.value()))
        ser.write(b's')
        for i in data:
            ser.write(bytes([i]))
            print(i)
        ser.write(b'e')   
       
    except Exception as e: 
        print(e)
        alert = QMessageBox()
        alert.setText('Error connect COM!')
        lay.textEdit.setPlainText(str(e) + "\n")
        alert.exec_()

def sendPIDvalue():
    global ser
    try: 
        values = []
        sleep(0.05)
        data = []
        data1 = []
        start = 's'
        end = 'e'
        CRC = ""
        
        value = int(format(lay_pid.slider.value(),'04'))
        value1 = int(format(lay_pid.slider1.value(),'04'))
        value2 = int(format(lay_pid.slider2.value(),'04'))
        value3 = int(format(lay_pid.slider3.value(),'04'))
        value4 = int(format(lay_pid.slider4.value(),'04'))
        value5 = int(format(lay_pid.slider5.value(),'04'))
        value6 = int(format(lay_pid.slider6.value(),'04'))
        value7 = int(format(lay_pid.slider7.value(),'04'))
        value8 = int(format(lay_pid.slider8.value(),'04'))
        value9 = int(format(lay_pid.slider9.value(),'04'))
        value10 = int(format(lay_pid.slider10.value(),'04'))
        value11 = int(format(lay_pid.slider11.value(),'04'))

        save_data = str(value) +" "+ str(value1)+" " + str(value2) +" "+ str(value3) +" " + str(value4) +" " + str(value5)
        save_data1 = str(value6) +" "+ str(value7)+" " + str(value8) +" "+ str(value9) +" " + str(value10) +" " + str(value11) 
        file_pid = open("./data/pid_save_file.txt","w")
        file_pid.write(save_data + " " + save_data1 +"\n")
        file_pid.close()
        esc1 = bin(value)
        esc2 = bin(value1)
        esc3 = bin(value2)
        esc4 = bin(value3)

        esc5 = bin(value4)
        esc6 = bin(value5)
        esc7 = bin(value6)
        esc8 = bin(value7)

        esc9 = bin(value8)
        esc10 = bin(value9)
        esc11 = bin(value10)
        esc12 = bin(value11)

        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc1)+2)]  + esc1[2:len(esc1)]
        # print(len(esc))   
        b1 = esc
        #value 2
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc2)+2)]  + esc2[2:len(esc2)]
        b2 = esc
        #value 3
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc3)+2)]  + esc3[2:len(esc3)]
        b3 = esc
        
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc4)+2)]  + esc4[2:len(esc4)]
        b4 = esc 

        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc5)+2)]  + esc5[2:len(esc5)]
        # print(len(esc))   
        b5 = esc
        #value 2
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc6)+2)]  + esc6[2:len(esc6)]
        b6 = esc
        #value 3
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc7)+2)]  + esc7[2:len(esc7)]
        b7 = esc
        
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc8)+2)]  + esc8[2:len(esc8)]
        b8 = esc 


        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc9)+2)]  + esc9[2:len(esc9)]
        # print(len(esc))   
        b9 = esc
        #value 2
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc10)+2)]  + esc10[2:len(esc10)]
        b10 = esc
        #value 3
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc11)+2)]  + esc11[2:len(esc11)]
        b11 = esc
        
        esc = '0b00000000000000'
        esc = esc[2:(len(esc)-len(esc12)+2)]  + esc12[2:len(esc12)]
        b12 = esc 


        data_frame = b1+ b2 + b3 +b4 +b5+ b6 + b7 +b8 +b9+ b10 + b11 +b12

        
        frame1 = '0b' + data_frame[0:8]
        frame2 = '0b' + data_frame[8:16]
        frame3 = '0b' + data_frame[16:24]
        frame4 = '0b' + data_frame[24:32]
        frame5 = '0b' + data_frame[32:40]
        frame6 = '0b' + data_frame[40:48]
        
        frame7 = '0b' + data_frame[48:56]
        frame8 = '0b' + data_frame[56:64]
        frame9 = '0b' + data_frame[64:72]

        frame10 = '0b' + data_frame[72:80]
        frame11 = '0b' + data_frame[80:88]
        frame12 = '0b' + data_frame[88:96]
        
        frame13 = '0b' + data_frame[96:104]
        frame14 = '0b' + data_frame[104:112]
        frame15 = '0b' + data_frame[112:120]

        frame16 = '0b' + data_frame[120:128]
        frame17 = '0b' + data_frame[128:136]
        frame18 = '0b' + data_frame[136:144]
        
        frame19 = '0b' + data_frame[144:152]
        frame20 = '0b' + data_frame[152:160]
        frame21 = '0b' + data_frame[160:168]
        
        # frame22 = '0b' + data_frame[168:176]
        # frame23 = '0b' + data_frame[176:184]
        # frame24 = '0b' + data_frame[184:192]

        frame_check = (value + value1 + value2 + value3 + value4 + value5 + value6 + value7 +value8 + value9 + value10 + value11) %37
         
        data.append(int(frame1,2))
        data.append(int(frame2,2))
        data.append(int(frame3,2))
        data.append(int(frame4,2))
        data.append(int(frame5,2))
        data.append(int(frame6,2))

        data.append(int(frame7,2))
        data.append(int(frame8,2))
        data.append(int(frame9,2))
        data.append(int(frame10,2))
        data.append(int(frame11,2))
        data.append(int(frame12,2))

        data.append(int(frame13,2))
        data.append(int(frame14,2))
        data.append(int(frame15,2))
        data.append(int(frame16,2))
        data.append(int(frame17,2))
        data.append(int(frame18,2))

        data.append(int(frame19,2))
        data.append(int(frame20,2))
        data.append(int(frame21,2))
        # data.append(int(frame22,2))
        # data.append(int(frame23,2))
        # data.append(int(frame24,2))

        data.append(frame_check)
      
        # lay.label4.setText("All Motor \n" + str(lay.slider4.value()))
        ser.write(b's')
        for i in data:
            ser.write(bytes([i]))
            print(i)
        ser.write(b'e')   
       
    except Exception as e: 
        print(e)
        alert = QMessageBox()
        alert.setText('Error connect COM!')
        lay.textEdit.setPlainText(str(e) + "\n")
        alert.exec_()

def setPID():
    global ser
    try: 
        values = []
        sleep(0.05)
        data = []
        data1 = []
        start = 's'
        end = 'e'
        CRC = ""
        value = int(format(lay.slider.value(),'04'))
        value1 = int(format(lay.slider1.value(),'04'))
        value2 = int(format(lay.slider2.value(),'04'))
        value3 = int(format(lay.slider3.value(),'04'))

        esc1 = bin(value + 180)
        esc2 = bin(value1 + 500)
        esc3 = bin(value2)
        esc4 = bin(value3)
    
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc1)+2)]  + esc1[2:len(esc1)]
        # print(len(esc))   
        b1 = esc
        #value 2
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc2)+2)]  + esc2[2:len(esc2)]
        b2 = esc
        #value 3
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc3)+2)]  + esc3[2:len(esc3)]
        b3 = esc
        
        esc = '0b00000000000'
        esc = esc[2:(len(esc)-len(esc4)+2)]  + esc4[2:len(esc4)]
        b4 = esc 

        frame_check = (value + value1 + value2 + value3 + 500 + 180) %37
        data_frame = b1 + b2 + b3 + b4

        
        frame1 = '0b' + data_frame[0:8]
        frame2 = '0b' + data_frame[8:16]
        frame3 = '0b' + data_frame[16:24]
        frame4 = '0b' + data_frame[24:32]
        frame5 = '0b' + data_frame[32:40]
        frame6 = '0b' + data_frame[40:44] + '0000'
        


        data.append(int(frame1,2))
        data.append(int(frame2,2))
        data.append(int(frame3,2))
        data.append(int(frame4,2))
        data.append(int(frame5,2))
        data.append(int(frame6,2))
        data.append(frame_check)
        # print(data)
        ser.write(b's')
        for i in data:
            ser.write(bytes([i]))
            print(i)
        ser.write(b'e') 
        #    
       
    except Exception as e: 
        print(e)
        alert = QMessageBox()
        alert.setText('Error connect COM!')
        lay.textEdit.setPlainText(str(e) + "\n")
        alert.exec_()
def emergencyStop():
    global ser, plot_control
    ser.write(b'd')
def startRun():
    global ser, plot_control
    plot_control = True
def Arm():
    global ser, plot_control
    plot_control = True
    # data = convert(1000,1000, 1000,1000)
    ser.write(b'a')
def Test():
    global ser, plot_control
    plot_control = True
    # data = convert(1000,1000, 1000,1000)
    ser.write(b't')

def pid_read():
    with open("./data/pid_save_file.txt", "r+") as pid_file:
    # pid_file = open("./data/pid_save_file.txt","r")
       pid_saved = pid_file.readline()
    pid_file.close()
    pid_array = pid_saved.split(' ')
    pid_data = []
    for i in pid_array:
        pid_data.append(float(i))
    lay_pid.slider.setValue(pid_data[0])
    lay_pid.slider1.setValue(pid_data[1])
    lay_pid.slider2.setValue(pid_data[2])

    lay_pid.slider3.setValue(pid_data[3])
    lay_pid.slider4.setValue(pid_data[4])
    lay_pid.slider5.setValue(pid_data[5])

    lay_pid.slider6.setValue(pid_data[6])
    lay_pid.slider7.setValue(pid_data[7])
    lay_pid.slider8.setValue(pid_data[8])

    lay_pid.slider9.setValue(pid_data[9])
    lay_pid.slider10.setValue(pid_data[10])
    lay_pid.slider11.setValue(pid_data[11])

def stopRun():
    global ser, plot_control
    plot_control = False
    # data = convert(1000,1000, 1000,1000)
    ser.write(b'd')
        # print(data)
    # for i in data:
    #     ser.write(bytes([i]))
            
    #     # ser.write(b'6')
    # ser.write(b'e')   

def connectCOM():
    try:                     # replace this port name by yours!
        baudrate = 115200
        global ser,portName,detached, rl, connect_check
        ser = serial.Serial(portName,baudrate)
        rl = ReadLine(ser)
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
        lay.textEdit.setPlainText(str(e) + "\n")
        alert.exec_()


def detachCOM():
    global ser, connect_check
    connect_check = False
    ser.close()
def list_clicked():
    global portName
    portName = lay.list_widget.currentItem().text()
def list_clicked1():
    global portName
    portName = lay_pid.list_widget.currentItem().text()
def update_slider():
    setPID()
    lay.label.setText("Motor 1 \n" + str(lay.slider.value()))
    lay.label1.setText("Motor 2 \n" + str(lay.slider1.value()))
    lay.label2.setText("Motor 3 \n" + str(lay.slider2.value()))
    lay.label3.setText("Motor 4 \n" + str(lay.slider3.value()))
    lay.label4.setText("All Motor \n" + str(lay.slider4.value()))
    lay.label5.setText("Kd 2 \n" + str(lay.slider5.value()))
    lay.label6.setText("Motor 1 \n" + str(lay.slider6.value()))
    lay.label7.setText("Motor 2 \n" + str(lay.slider7.value()))  

def update_pid():
    # setPID()
    lay_pid.label.setText("Kp1 (r)\n" + str(lay_pid.slider.value()/100))
    lay_pid.label1.setText("Ki1 (r)\n" + str(lay_pid.slider1.value()/100))
    lay_pid.label2.setText("Kd1 (r)\n" + str(lay_pid.slider2.value()/100))
    lay_pid.label3.setText("Kp2 (p)\n" + str(lay_pid.slider3.value()/100))
    lay_pid.label4.setText("Ki2 (p)\n" + str(lay_pid.slider4.value()/100))
    lay_pid.label5.setText("Kd2 (p)\n" + str(lay_pid.slider5.value()/100))
    lay_pid.label6.setText("Kp3 (y)\n" + str(lay_pid.slider6.value()/100))
    lay_pid.label7.setText("Ki3 (y)\n" + str(lay_pid.slider7.value()/100))  
    lay_pid.label8.setText("Kd3 (y)\n" + str(lay_pid.slider8.value()/100))
    lay_pid.label9.setText("Kp4 (alt)\n" + str(lay_pid.slider9.value()/100))
    lay_pid.label10.setText("Ki4 (alt)\n" + str(lay_pid.slider10.value()/100))
    lay_pid.label11.setText("Kd4 (alt)\n" + str(lay_pid.slider11.value()/100))

main_layout.addLayout(lay.layout)
main_layout.addLayout(lay.layout_inner1)
main_layout.addLayout(lay.layout_value)
main_layout.addLayout(lay.plot_batch)
main_layout.addWidget(lay.textEdit)

lay.connect_button.clicked.connect(connectCOM)
lay.detatch_button.clicked.connect(detachCOM)
lay.list_widget.clicked.connect(list_clicked)
lay.button.clicked.connect(emergencyStop)

lay.button1.clicked.connect(startRun)
lay.button2.clicked.connect(stopRun)
lay.button3.clicked.connect(Arm)
lay.button4.clicked.connect(Test)

lay.slider.valueChanged.connect(update_slider)
lay.slider1.valueChanged.connect(update_slider)
lay.slider2.valueChanged.connect(update_slider)
lay.slider3.valueChanged.connect(update_slider)
lay.slider4.valueChanged.connect(setPID_all)
lay.slider5.valueChanged.connect(update_slider)
lay.slider7.valueChanged.connect(setPID_all)

tabs = QTabWidget()
tab1 = QWidget()
tab2 = QWidget()
tab3 = QWidget()

lay_pid.connect_button.clicked.connect(connectCOM)
lay_pid.detatch_button.clicked.connect(detachCOM)
lay_pid.list_widget.clicked.connect(list_clicked1)


#######
lay_pid.button.clicked.connect(sendPIDvalue)
#########
lay_pid.button1.clicked.connect(pid_read)
lay_pid.button2.clicked.connect(startRun)

lay_pid.slider.valueChanged.connect(update_pid)
lay_pid.slider1.valueChanged.connect(update_pid)
lay_pid.slider2.valueChanged.connect(update_pid)
lay_pid.slider3.valueChanged.connect(update_pid)
lay_pid.slider4.valueChanged.connect(update_pid)
lay_pid.slider5.valueChanged.connect(update_pid)
lay_pid.slider6.valueChanged.connect(update_pid)
lay_pid.slider7.valueChanged.connect(update_pid)
lay_pid.slider8.valueChanged.connect(update_pid)
lay_pid.slider9.valueChanged.connect(update_pid)
lay_pid.slider10.valueChanged.connect(update_pid)
lay_pid.slider11.valueChanged.connect(update_pid)

main_layout_2.addLayout(lay_pid.layout)
main_layout_2.addLayout(lay_pid.layout_inner1)
main_layout_2.addLayout(lay_pid.layout_value)
main_layout_2.addLayout(lay_pid.plot_batch)




tab1.setLayout(main_layout)

tab2.setLayout(main_layout_2)



tabs.addTab(tab1,"Plot")
tabs.addTab(tab2,"Tune")
tabs.addTab(tab3,"Control")

main_layout_1.addWidget(tabs)

win.setLayout(main_layout_1)
win.show()


windowWidth = 500                       # width of the window displaying the curve
Xm = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr = -windowWidth                      # set first x position

Xm3 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr3 = -windowWidth                      # set first x position

Xm1 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr1 = -windowWidth      

Xm2 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr2 = -windowWidth                          # set first x position

Xm4 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr4 = -windowWidth                      # set first x position

Xm5 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr5 = -windowWidth      

Xm6 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr6 = -windowWidth                          # set first x position

Xm7 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr7 = -windowWidth                      # set first x position

Xm8 = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
ptr8 = -windowWidth  
# Realtime data plot. Each time this function is called, the data display is updated
def thread_function():
    global rl
    value = rl.readline()


def update(x_):
    global lay, ptr, ptr1, ptr2,ptr3, ptr4, ptr5,ptr6, ptr7, ptr8, Xm, Xm1,Xm2,Xm3, Xm4,Xm5,Xm6, Xm7,Xm8, detached, plot_control ,current_time
    Xm[:-1] = Xm[1:]                      # shift data in the temporal mean 1 sample left
    Xm1[:-1] = Xm1[1:]
    Xm2[:-1] = Xm2[1:]

    Xm3[:-1] = Xm3[1:]                      # shift data in the temporal mean 1 sample left
    Xm4[:-1] = Xm4[1:]
    Xm5[:-1] = Xm5[1:]

    Xm6[:-1] = Xm6[1:]                      # shift data in the temporal mean 1 sample left
    Xm7[:-1] = Xm7[1:]
    Xm8[:-1] = Xm8[1:]

    try:
        # x_ = []
        # x_ = advance_uart(value)
        # print(x_)
            # data.append(temp1)
        # file_data.write(string + "\n")
        # print(x_)
        
        lay.label_duty1.setText("Roll: " + str(x_[0]))
        lay.label_duty2.setText("Pitch: " + str(x_[1]))
        lay.label_duty3.setText("Yaw: " + str(x_[2]))
        Xm[-1] = x_[0]                 # vector containing the instantaneous values 
        Xm1[-1] = x_[1]
        Xm2[-1] = x_[2]

        # Xm3[-1] = x_[3]                 # vector containing the instantaneous values 
        # Xm4[-1] = x_[4]
        # Xm5[-1] = x_[5] 

        # Xm6[-1] = x_[6]                 # vector containing the instantaneous values 
        # Xm7[-1] = x_[7]
        # Xm8[-1] = x_[8] 
        if(len(x_)>2): 
            millis = int(round(time.time() * 1000)) - current_time
            current_time = int(round(time.time() * 1000))
            lay.label_duty10.setText("Time: " + str(millis))    
    except Exception as e: 
        print(e)   
        # textEdit.setPlainText(str(e) + "\n")
        Xm[-1] = Xm[-2]
        Xm1[-1] = Xm1[-2]
        Xm2[-1] = Xm2[-2]
    #ser.close()
    ptr8 += 1                              # update x position for displaying the curve
    lay.curve8.setData(Xm8)                     # set the curve with this data
    lay.curve8.setPos(ptr8,0)    
    ptr7 += 1                              # update x position for displaying the curve
    lay.curve7.setData(Xm7)                     # set the curve with this data
    lay.curve7.setPos(ptr7,0)                   # set x position in the graph to 0

    ptr6 += 1                              # update x position for displaying the curve
    lay.curve6.setData(Xm6)                     # set the curve with this data
    lay.curve6.setPos(ptr6,0)                   # set x position in the graph to 0

    ptr5 += 1                              # update x position for displaying the curve
    lay.curve5.setData(Xm5)                     # set the curve with this data
    lay.curve5.setPos(ptr5,0)    
    ptr4 += 1                              # update x position for displaying the curve
    lay.curve4.setData(Xm4)                     # set the curve with this data
    lay.curve4.setPos(ptr4,0)                   # set x position in the graph to 0
    ptr3 += 1                              # update x position for displaying the curve
    lay.curve3.setData(Xm3)                     # set the curve with this data
    lay.curve3.setPos(ptr3,0)                   # set x position in the graph to 0

    ptr2 += 1                              # update x position for displaying the curve
    lay.curve2.setData(Xm2)                     # set the curve with this data4
    lay.curve2.setPos(ptr2,0)    
    ptr1 += 1                              # update x position for displaying the curve
    lay.curve1.setData(Xm1)                     # set the curve with this data
    lay.curve1.setPos(ptr1,0)                   # set x position in the graph to 0
    ptr += 1                              # update x position for displaying the curve
    lay.curve.setData(Xm)                     # set the curve with this data
    lay.curve.setPos(ptr,0)                   # set x position in the graph to 0
    

### MAIN PROGRAM #####    
# this is a brutal infinite loop calling your realtime data plot
def plot_data():
    try:
        if(connect_check & plot_control):
            value = []
            value = rl.readline()
        
            X =  (value.decode().replace('\x00','').replace('\x00','')).split(' ')
            data = []
            string = ""
            for i in X:
                data.append(float(i))
                string += i + " "
                    # data.append(temp1)
            # file_data.write(string + "\n")   
            if(len(data) == 12):
                get_pid(data)
            #     data = []
            #     b = []
            #     # print(value[15])
            #     b3 = value[15]
            #     b4 = value[19]
            #     b5 = value[23]
            #     # setPID()
            #     temp = (b5 + b4 + b3) % 37
            #     crc = value[24]
            #     # print(crc)  
            #     # struct.unpack('f', b)
            #     if(crc == temp):
            #         for i in range(6):
            #             b = value[4*i:4*(i+1)]
            #             data.append(round(float(struct.unpack('f',b)[0]),3))
                        
            update(data)
                # else:
                #     print("not OK")
                # print(x_)
                # if(len(value)<4):
                #     while(value.decode() == 'd$f'):
                #         setPID()
                # if(): 
                
        # worker = Worker()
        # threadpool.start(worker)
    except Exception as e: 
        print(e)   
### END QtApp ####
class AThread(QThread):

    def run(self):
        count = 0
        while True:
            time.sleep(0.2)
            plot_data()
            # count += 1

thread = AThread()
thread.finished.connect(app.exit)
thread.start()