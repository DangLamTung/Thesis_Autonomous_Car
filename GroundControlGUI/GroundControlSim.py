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
from gc_gui import *

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


    

    # app = QtWidgets.QApplication(sys.argv)
    app_flask = socketio.Middleware(sio, app_flask)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app_flask)
    # wst = threading.Thread(target=serve_app, args=(sio,app_flask))
    # wst.daemon = True
    # wst.start()

    # MainWindow = QtWidgets.QMainWindow()
    # ui = Ui_MainWindow()
    # ui.setupUi(MainWindow)
    # MainWindow.show()
    # sys.exit(app.exec_())
    # deploy as an eventlet WSGI server