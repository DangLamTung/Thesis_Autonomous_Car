# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gc_gui1.ui'
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
# import gpsmap
from QTImageView import QtImageViewer
from connect import *
from zmq_read import *
# from waypoint_gui import Ui_Dialog
import queue

check_point = []


class myLabel(QLabel):
    clicked = pyqtSignal()
    def mouseReleaseEvent(self, QMouseEvent):
        if QMouseEvent.button() == Qt.LeftButton:
            self.clicked.emit()

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(604, 572)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(0, 20, 361, 491))
        self.groupBox.setObjectName("groupBox")
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setGeometry(QtCore.QRect(30, 20, 311, 451))
        self.label_7.setObjectName("label_7")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(460, 380, 93, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(460, 330, 93, 28))
        self.pushButton_2.setObjectName("pushButton_2")
        self.groupBox_5 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QtCore.QRect(410, 10, 191, 251))
        self.groupBox_5.setObjectName("groupBox_5")
        self.label_9 = QtWidgets.QLabel(self.groupBox_5)
        self.label_9.setGeometry(QtCore.QRect(10, 40, 151, 41))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.groupBox_5)
        self.label_10.setGeometry(QtCore.QRect(10, 80, 171, 41))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.groupBox_5)
        self.label_11.setGeometry(QtCore.QRect(10, 120, 171, 41))
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(self.groupBox_5)
        self.label_12.setGeometry(QtCore.QRect(10, 150, 151, 41))
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.groupBox_5)
        self.label_13.setGeometry(QtCore.QRect(10, 190, 161, 41))
        self.label_13.setObjectName("label_13")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(460, 280, 93, 28))
        self.pushButton.setObjectName("pushButton")

        self.pushButton.mousePressEvent = self.on_Button_clicked


        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 604, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
 #Show map #######################################################################
        self.drawing = False
        self.lastPoint = QPoint()
        self.map_image = QPixmap("map_png.PNG")
        self.map_png = cv2.imread("./map_png.png")
        self.viewer = QtImageViewer(self.groupBox)
        self.viewer.setGeometry(QtCore.QRect(20, 30, 311, 451))
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
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Map"))
        self.label_7.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton_3.setText(_translate("MainWindow", "PushButton"))
        self.pushButton_2.setText(_translate("MainWindow", "PushButton"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Status"))
        self.label_9.setText(_translate("MainWindow", "Orientation:"))
        self.label_10.setText(_translate("MainWindow", "Position:"))
        self.label_11.setText(_translate("MainWindow", "Velocity:"))
        self.label_12.setText(_translate("MainWindow", "Lat:"))
        self.label_13.setText(_translate("MainWindow", "Lon:"))
        self.pushButton.setText(_translate("MainWindow", "PushButton"))
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
        # scene.clear()
        self.viewer.setScene(scene)
        self.viewer.setImage(self.map_image)

    
    def on_Button_clicked(self, checked=None):
        global global_path 
        global_path = findRoute(self.check_point)
        global_path = enhanceWaypoint(global_path)

    
        pathOnMap = convertPath(global_path)
        # print(pathOnMap)
        _thread.start_new_thread( uart_connect,(global_path,))
        scene = self.viewer.scene
        
        
        for i in pathOnMap:
            item = QGraphicsEllipseItem(i[1], i[0], 10, 10)
            item.setPen(QPen(QColor("red")))
            item.setBrush(QBrush(QColor("red")))
            scene.addItem(item)
        self.viewer.setScene(scene)
        self.check_point = []



if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

