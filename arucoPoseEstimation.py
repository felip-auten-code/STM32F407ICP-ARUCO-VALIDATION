import cv2
import matplotlib.pyplot as plt
import math as m
import time
import os
import numpy as np
import pandas as pd
import algebra as alg


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
    
    x_coords = []
    y_coords = []

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict,parameters=parameters )

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec         ---     (different from those of camera coefficients)
            rvec_list , tvec_list, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 8, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            imaxis = cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.drawFrameAxes(imaxis, mtx, dist, rvec_list , tvec_list, 10)
            
           #tvecs.append([ids[i], frameID])
           # [ MarkerID            Tx          Ty          Tz              FrameID]
           
            tvecs.extend([ids[i][0] , tvec_list[i][0][2] , tvec_list[i][0][1] , tvec_list[i][0][0] ,  frameID])             # flipped
            rvecs.extend([ids[i][0] , rvec_list[i][0][2] , rvec_list[i][0][1] , rvec_list[i][0][0] ,  frameID])             # flipped
            
            
            #print(tvecs)
            t = tvec_list[0][0]
            r = rvec_list[0][0]
            
            t_ = t * -1                 # flip
            r_ = r * -1                 # flip
            
            
            rotM2, jacobian = cv2.Rodrigues(r_)                                         # rotation matrix with rvec angles
            rotM = alg.RotationMatrixEuler(r[0], r[1], r[2])                            # my rotation bleh
            
            real_O_tvec = np.dot(rotM, t_)
            real_O_tvec2 = np.dot(rotM2, t_)
            pitch, roll, yaw = alg.rotationMatrixToEulerAngles(rotM2)
            print(yaw, pitch, roll)
            
            tvec_str = "x = %4.0f  y = %4.0f  z = %4.0f "%(real_O_tvec[0], real_O_tvec[1], real_O_tvec[2])
            tvec_str2 = "yaw = %4.0f"%( m.degrees(yaw))
            
            cv2.putText(frame, tvec_str, (5,100), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3, cv2.LINE_AA)
            cv2.putText(frame, tvec_str2, (15,200), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3, cv2.LINE_AA)
            #cv2.line(frame, (int(real_O_tvec[0]), int(real_O_tvec[1])), (int(t[0]), int(t[1])), (0,0,255), 170)
            
            xg = t[0]
            yg = t[1]
            
            x_coords.append(xg)
            y_coords.append(yg)
            
            # Store translation vectors
        
        #TvecsTable = pd.DataFrame(data = tvecs)
        #print(tvecs)

    return frame, tvecs, rvecs, corners, x_coords, y_coords

# THE PROCESS
'''
    mtx    -> matrix coefficients      / CAMERA INSTRINSIC
    dist   -> distortion coefficients  /
    
    rvec   -> rotation vectors         / OF EACH IDENTIFIED MARKER
    tvec   -> translation vectors      /
'''

tvecs = []
rvecs = []
x_coords = []
y_coords = []

capture.set(cv2.CAP_PROP_POS_FRAMES, 800)

corners = []

while capture.isOpened():
    frameID = capture.get(1)
    ret, frame = capture.read()
    if(ret != True):
        break

    outputFrame, tvec, rvec, corner, x, y = pose_esitmation(frame, aruco_dict, mtx, dist, frameID)
    
    x_coords.append(x)
    y_coords.append(y)
    tvecs.append(tvec)
    rvecs.append(rvec)
    corners.append([int(corner[0][0][1][1]), int(corner[0][0][1][0])])
    #print(corner[0][0][1])
    

    # dsize
    scale_percent = 50
    width = int(outputFrame.shape[1] * scale_percent / 100)
    height = int(outputFrame.shape[0] * scale_percent / 100)
    dsize = (width, height)
    cv2.line(outputFrame, (int(corner[0][0][2][0]), int(corner[0][0][2][1])), (int(corner[0][0][2][0]), int(corner[0][0][2][1])) , (0, 0, 255), 70)
    
    # resize image
    out2 = cv2.resize(outputFrame, dsize)
    print(frameID)
    cv2.imshow('Estimated POSE', out2)
    
    if(frameID == 1300):
        break
    
    key = cv2.waitKey(1) & 0xFF
    if(key == ord('q')):
        break
    if(key == ord('p')):
        cv2.waitKey(-1)     #wait until any key is pressed
capture.release()
cv2.destroyAllWindows()


x = corners[0:][0]
print(x)

#rint(tvecs.shape)

TvecsTable = pd.DataFrame(data = tvecs[0:], columns = ["markerID", "tx", "ty", "tz", "frameID"] )
RvecsTable = pd.DataFrame(data = rvecs, columns= ["markerID", "RotX", "RotY", "RotZ", "FrameID"])

print(RvecsTable)


TvecsTable.to_csv("./data/TvecsARUCO.csv")

fig1 = plt.figure(figsize = (7,7))

ax = plt.plot(TvecsTable.tx, TvecsTable.ty)
#plt.show()

fig2 = plt.figure(figsize=(7,7))
print (corners, corners[1])
ax2 = plt.plot(corners)
fig3= plt.figure()
ax3 = plt.plot(x_coords, y_coords)
plt.grid()
plt.show()