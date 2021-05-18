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

from PIL import Image

import _thread
import utils
import gpsmap
from QTImageView import QtImageViewer
from connect import *
from gc_gui import *
from zmq_read import *




if __name__ == "__main__":
    import sys


    

    app = QtWidgets.QApplication(sys.argv)
  
    # wst = threading.Thread(target=serve_app, args=(sio,app_flask))
    # wst.daemon = True
    # wst.start()
    _thread.start_new_thread( zmq_connect, ("zmq", ) )
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(gc_gui.MainWindow)
    MainWindow.show()
    
    sys.exit(app.exec_())
    # deploy as an eventlet WSGI server