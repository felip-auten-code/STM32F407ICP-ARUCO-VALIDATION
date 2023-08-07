import cv2
import matplotlib.pyplot as plt
import math
import time
import os
import numpy as np
import pandas as pd

VIDFolder = "./data/"
VIDFile = "./data/Experiment01VIDaruco.mp4"

capture = cv2.VideoCapture(VIDFile)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

mtx = np.array([[2.068268675143158816e+03, 0.000000000000000000e+00, 9.864133721217920083e+02],
                [0.000000000000000000e+00, 2.162385610454082780e+03, 5.558514673165518616e+02],
                [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])

dist = np.array([[-4.344115590524044168e-01],
                 [5.946729271049551180e-01],
                 [2.404662846493914666e-02],
                 [-6.525542953777989374e-02],
                 [3.666177449982374714e+00],
                 [5.919204966661361089e-01],
                 [-1.897748795248646259e+00],
                 [7.758460492255061069e+00],
                 [0.000000000000000000e+00],
                 [0.000000000000000000e+00],
                 [0.000000000000000000e+00],
                 [0.000000000000000000e+00],
                 [0.000000000000000000e+00],
                 [0.000000000000000000e+00]])

def pose_esitmation(frame, aruco_dict, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,parameters=parameters )

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            imaxis = cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(imaxis, mtx, dist, rvec[i], tvec[i], 0.01)

    return frame

# THE PROCESS
'''
    mtx    -> matrix coefficients      / CAMERA INSTRINSIC
    dist   -> distortion coefficients  /
    
    rvec   -> rotation vectors         / OF EACH IDENTIFIED MARKER
    tvec   -> translation vectors      /
'''

while capture.isOpened():
    frameID = capture.get(1)
    ret, frame = capture.read()
    if(ret != True):
        break

    output = pose_esitmation(frame, aruco_dict, mtx, dist)
    # dsize
    scale_percent = 50
    width = int(output.shape[1] * scale_percent / 100)
    height = int(output.shape[0] * scale_percent / 100)
    dsize = (width, height)

    # resize image
    out2 = cv2.resize(output, dsize)
    print(frameID)
    cv2.imshow('Estimated Pode', out2)
    
    key = cv2.waitKey(1) & 0xFF
    if(key == ord('q')):
        break
capture.release()
cv2.destroyAllWindows()