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

def pose_esitmation(frame, aruco_dict, matrix_coefficients, distortion_coefficients, frameID):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()

    tvecs = []
    rvecs = []
    markers = []

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,parameters=parameters )

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec         ---     (different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            imaxis = cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(imaxis, mtx, dist, rvec, tvec, 0.01)
            
           #tvecs.append([ids[i], frameID])
            tvecs.extend([ids[i][0], tvec[i][0][0], tvec[i][0][1], tvec[i][0][2] , frameID])
            rvecs.append([ids[i][0], rvec[i][0], frameID])
            # Store translation vectors
        
        #TvecsTable = pd.DataFrame(data = tvecs)
        #print(tvecs)

    return frame, tvecs, corners

# THE PROCESS
'''
    mtx    -> matrix coefficients      / CAMERA INSTRINSIC
    dist   -> distortion coefficients  /
    
    rvec   -> rotation vectors         / OF EACH IDENTIFIED MARKER
    tvec   -> translation vectors      /
'''

tvecs = []

capture.set(cv2.CAP_PROP_POS_FRAMES, 800)

corners = []

while capture.isOpened():
    frameID = capture.get(1)
    ret, frame = capture.read()
    if(ret != True):
        break

    outputFrame, tvec, corner = pose_esitmation(frame, aruco_dict, mtx, dist, frameID)
    tvecs.append(tvec)
    corners.append([int(corner[0][0][0][1]), int(corner[0][0][0][1])])
    print(corner[0][0][0])
    # dsize
    scale_percent = 50
    width = int(outputFrame.shape[1] * scale_percent / 100)
    height = int(outputFrame.shape[0] * scale_percent / 100)
    dsize = (width, height)
    cv2.line(outputFrame, (int(corner[0][0][0][0]), int(corner[0][0][0][1])), (int(corner[0][0][0][0]), int(corner[0][0][0][1])) , (0, 0, 255), 100)
    # resize image
    out2 = cv2.resize(outputFrame, dsize)
    print(frameID)
    cv2.imshow('Estimated POSE', out2)
    
    if(frameID == 1300):
        break
    
    key = cv2.waitKey(1) & 0xFF
    if(key == ord('q')):
        break
capture.release()
cv2.destroyAllWindows()


x = corners[0:][0]
print(x)

#rint(tvecs.shape)

TvecsTable = pd.DataFrame(data = tvecs[0:], columns = ["markerID", "tx", "ty", "tz", "frameID"] )

print(TvecsTable)

TvecsTable.to_csv("./data/TvecsARUCO.csv")

fig1 = plt.figure(figsize = (7,7))

ax = plt.plot(TvecsTable.tx, TvecsTable.ty)
#plt.show()

fig2 = plt.figure(figsize=(7,7))
ax2 = plt.plot(corners)
plt.grid()
plt.show()