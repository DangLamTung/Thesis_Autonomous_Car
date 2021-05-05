import numpy as np
import cv2
import cv2.aruco as aruco
cap = cv2.VideoCapture(0)  # Get the camera source
def track(matrix_coefficients, distortion_coefficients):
    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
