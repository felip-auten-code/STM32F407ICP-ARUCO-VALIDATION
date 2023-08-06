import cv2
import matplotlib.pyplot as plt
import math
import time
import os
import numpy as np

#board = cv2.aruco.CharucoBoard((3,3)  1, 0.8, dictionary=cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
dict = cv2.aruco.DICT_6X6_250
board = cv2.aruco.CharucoBoard((3,3) , 1, 0.8, aruco_dict)

#board = cv2.aruco.GridBoard((3,3), 1, 0.8, dict)

imgs_folder = "./data/imgs/"

def wait_(tim):
    time.sleep(tim*0.3)
    print(".")
    time.sleep(tim*0.3)
    print(".")
    time.sleep(tim*0.3)
    print(".")

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-6))

    print("finished")
    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def read_chessboard(imgs):
    print("CHARUCO POSE ESTIMATION STARTING")
    wait_(1)
    allCorners = []
    allIDS = []
    decimator = 0

    for i in imgs:
        print("=> Processing image {0}".format(i))
        frame = cv2.imread(i)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #parameters =  cv2.aruco.DetectorParameters()
        #res = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        res = cv2.aruco.detectMarkers(gray, aruco_dict )

        if(len(res[0]) > 0):
            res2 = cv2.aruco.interpolateCornersCharuco(res[0], res[1], gray, board)
            if(     res2[1] is not None 
                        and res2[2] is not None 
                        and len(res2[1]) > 3
                        and decimator%1==0):
                allCorners.append(res2[1])
                allIDS.append(res2[2])

        decimator+=1        

    img_size = gray.shape
    print("finished")
    return allCorners, allIDS, img_size
    

images = [imgs_folder + f for f in os.listdir(imgs_folder) if f.startswith("image_")]

allCorners, allIDS, imsize = read_chessboard(images)

ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIDS, imsize)

print(mtx)
print(ret)
print(dist)

np.savetxt(imgs_folder+"calib_mtx_cellcamVID.csv", mtx)
np.savetxt(imgs_folder+"calib_dist_cellcamVID.csv", dist)
