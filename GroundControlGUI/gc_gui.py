# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gc_gui.ui'
#
# Created by: PyQt5 UI code generator 5.9
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtGui import QPixmap, QPainter, QPen, QBrush, QColor, QBrush
from PyQt5.QtWidgets import QWidget, QLabel, QApplication, QHBoxLayout, QDialog, QGraphicsEllipseItem

import cv2
import connect

import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO
import _thread
import utils
import gpsmap
from QTImageView import QtImageViewer
from connect import *
# from waypoint_gui import Ui_Dialog

check_point = []

class myLabel(QLabel):
    clicked = pyqtSignal()
    def mouseReleaseEvent(self, QMouseEvent):
        if QMouseEvent.button() == Qt.LeftButton:
            self.clicked.emit()
    # def paintEvent(self, event):
    #     size = self.size()
    #     painter = QtGui.QPainter(self)
    #     point = QtCore.QPoint(0,0)
    #     map_image = QPixmap("map_png.PNG")
    #     scaledPix = map_image.scaled(size, Qt.KeepAspectRatio, transformMode = Qt.SmoothTransformation)
    #     # start painting the label from left upper corner
    #     point.setX((size.width() - scaledPix.width())/2)
    #     point.setY((size.height() - scaledPix.height())/2)
 
    #     painter.drawPixmap(point, scaledPix)
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1145, 697)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 10, 1131, 631))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.groupBox_3 = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_3.setGeometry(QtCore.QRect(470, 210, 361, 171))
        self.groupBox_3.setObjectName("groupBox_3")
        self.label_5 = QtWidgets.QLabel(self.groupBox_3)
        self.label_5.setGeometry(QtCore.QRect(20, 20, 321, 131))
        self.label_5.setObjectName("label_5")


        self.groupBox_2 = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_2.setGeometry(QtCore.QRect(470, 10, 361, 171))
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_6 = QtWidgets.QLabel(self.groupBox_2)
        self.label_6.setGeometry(QtCore.QRect(20, 20, 321, 141))
        self.label_6.setObjectName("label_6")
        self.groupBox = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 441, 591))
        self.groupBox.setObjectName("groupBox")



        #Show map #######################################################################
        self.drawing = False
        self.lastPoint = QPoint()
        self.map_image = QPixmap("map_png.PNG")
        self.map_png = cv2.imread("./map_png.png")
        self.viewer = QtImageViewer(self.groupBox)
        self.viewer.setGeometry(QtCore.QRect(20, 30, 411, 551))
        self.viewer.aspectRatioMode = Qt.KeepAspectRatio
        self.viewer.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.viewer.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.viewer.canZoom = True
        # Allow panning with left mouse button.
        self.viewer.canPan = True
        self.viewer.setImage(self.map_image)
        self.viewer.leftMouseButtonPressed.connect(self.handleLeftClick)

        self.check_point = []
        ##################################################################################


        self.groupBox_4 = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_4.setGeometry(QtCore.QRect(470, 410, 361, 171))
        self.groupBox_4.setObjectName("groupBox_4")
        self.label_8 = QtWidgets.QLabel(self.groupBox_4)
        self.label_8.setGeometry(QtCore.QRect(20, 20, 321, 131))
        self.label_8.setObjectName("label_8")
        self.pushButton = QtWidgets.QPushButton(self.tab_3)

        self.pushButton.mousePressEvent = self.on_Button_clicked


        self.pushButton.setGeometry(QtCore.QRect(930, 440, 93, 28))
        self.pushButton.setObjectName("Find Road")
        self.pushButton_2 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_2.setGeometry(QtCore.QRect(930, 490, 93, 28))
        self.pushButton_2.setObjectName("pushButton_2")

        self.pushButton_2.mousePressEvent = self.clearMap

        self.pushButton_3 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_3.setGeometry(QtCore.QRect(930, 540, 93, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        self.groupBox_5 = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_5.setGeometry(QtCore.QRect(840, 10, 251, 381))
        self.groupBox_5.setObjectName("groupBox_5")
        self.label_9 = QtWidgets.QLabel(self.groupBox_5)
        self.label_9.setGeometry(QtCore.QRect(10, 20, 211, 41))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.groupBox_5)
        self.label_10.setGeometry(QtCore.QRect(10, 60, 211, 41))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.groupBox_5)
        self.label_11.setGeometry(QtCore.QRect(10, 100, 211, 41))
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(self.groupBox_5)
        self.label_12.setGeometry(QtCore.QRect(10, 140, 211, 41))
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.groupBox_5)
        self.label_13.setGeometry(QtCore.QRect(10, 180, 211, 41))
        self.label_13.setObjectName("label_13")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.tabWidget.addTab(self.tab_4, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1145, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")

        MainWindow.setStatusBar(self.statusbar)
        # size = self.label_7.size()


        # self.map_image = self.map_image.scaled(self.label_7.size(), Qt.KeepAspectRatio)
    
        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_3.setTitle(_translate("MainWindow", "RoadView"))
        self.label_5.setText(_translate("MainWindow", "TextLabel"))
        self.groupBox_2.setTitle(_translate("MainWindow", "BirdEyeView"))
        self.label_6.setText(_translate("MainWindow", "TextLabel"))
        self.groupBox.setTitle(_translate("MainWindow", "Map"))

        # self.label_7.setPixmap(self.map_image)
        # self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        # # self.label_7.setScaledContents(True)
        
        # self.label_7.setMinimumSize(1,1)

        self.groupBox_4.setTitle(_translate("MainWindow", "OccupacyMap"))
        self.label_8.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton.setText(_translate("MainWindow", "Find Road"))
        self.pushButton_2.setText(_translate("MainWindow", "PushButton"))
        self.pushButton_3.setText(_translate("MainWindow", "PushButton"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Status"))
        self.label_9.setText(_translate("MainWindow", "Orientation:"))
        self.label_10.setText(_translate("MainWindow", "Position:"))
        self.label_11.setText(_translate("MainWindow", "Velocity:"))
        self.label_12.setText(_translate("MainWindow", "Lat:"))
        self.label_13.setText(_translate("MainWindow", "Lon:"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Tab 1"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "Tab 2"))

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
        
        gps = pix2global(column,row)
        win.check_point.append(gps)
        
        i = global2pix(gps[0],gps[1])
        print(i)
        item = QGraphicsEllipseItem(i[1], i[0], 10, 10)
        item.setPen(QPen(QColor("red")))
        item.setBrush(QBrush(QColor("red")))
        scene.addItem(item)

        win.viewer.setScene(scene)

        
        print(pix2global(x,y))


    def showData(self, event):
        global check_point
        # # pos = self.label_7.mapFromParent(event.pos())

        # x = pos.x()
        # y = pos.y()
        # self.map_png = cv2.circle(self.map_png,(x,y), radius=1, color=(0, 0, 255), thickness=10)
        # pixmap = self.convert_cv_qt(self.map_png)
        # self.label_7.setPixmap(pixmap)
        # print(x,y)
    def clearMap(self, checked=None):
        scene = self.viewer.scene
        scene.clear()
        self.viewer.setScene(scene)
        self.viewer.setImage(self.map_image)

    
    def on_Button_clicked(self, checked=None):
        path = findRoute(self.check_point)
        pathOnMap = convertPath(path)
        scene = self.viewer.scene

        for i in pathOnMap:
            item = QGraphicsEllipseItem(i[1], i[0], 10, 10)
            item.setPen(QPen(QColor("red")))
            item.setBrush(QBrush(QColor("red")))
            scene.addItem(item)
        self.viewer.setScene(scene)
        self.check_point = []


model = None
prev_image_array = None

MAX_SPEED = 25
MIN_SPEED = 10

speed_limit = MAX_SPEED

IMAGE_H = 66
IMAGE_W = 200



sio = socketio.Server()
app_flask = Flask(__name__)
@sio.on('telemetry')
def telemetry(sid, data):
    global image
    print("ok")
    if data:

        # The current steering angle of the car
        steering_angle = float(data["steering_angle"])
        # The current throttle of the car
        throttle = float(data["throttle"])
        # The current speed of the car
        speed = float(data["speed"])

        Lat = float(data["Lat"])

        Lon = float(data["Lon"])

        roll = float(data["r"])

        pitch = float(data["p"])

        yaw = float(data["y"])
        # The current image from the center camera of the car
        image = Image.open(BytesIO(base64.b64decode(data["image"])))
        # save frame
        # if args.image_folder != '':
        #     timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
        #     image_filename = os.path.join(args.image_folder, timestamp)
        #     image.save('{}.jpg'.format(image_filename))
            
        try:
            image = np.asarray(image)       # from PIL image to numpy array
            
            image = utils.preprocess(image) # apply the preprocessing
            
            
            #waits for user to press any key  
            #(this is necessary to avoid Python kernel form crashing) 
            
            # _thread.start_new_thread( print_time, ("Thread-1", image,Lat,Lon,pitch ) )
            # _thread.start_new_thread( draw_map, ("Thread-1",pitch, Lat,Lon ) )
            image = np.array([image])       # the model expects 4D array
            sio.emit('manual', data={}, skip_sid=True)
            print("ok")
        except Exception as e:
            print(e)
        
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)
def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)

import threading
def serve_app(sio, app):
    app_flask = socketio.Middleware(sio,app)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app_flask)
if __name__ == "__main__":
    import sys


    

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    

   

