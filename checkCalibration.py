import cv2
import matplotlib.pyplot as plt
import math
import time
import os
import numpy as np
import pandas as pd

imagesFolder = "./data/imgs/"

mtx = np.array([[1.88211483e+03, 0.00000000e+00, 9.49235560e+02],
 [0.00000000e+00, 2.04967658e+03, 5.35696081e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist =np.array([[-0.02959507],
 [-1.41403305],
 [-0.00487649],
 [-0.10976022],
 [ 4.44020016],
 [ 0.02561852],
 [ 1.54381655],
 [-2.32133622],
 [ 0.        ],
 [ 0.        ],
 [ 0.        ],
 [ 0.        ],
 [ 0.        ],
 [ 0.        ]])


# i=24 # select image id
# plt.figure()
# frame = cv2.imread(imagesFolder + "image_100.jpg".format(i))
# img_undist = cv2.undistort(frame,mtx,dist,None)
# plt.subplot(211)
# plt.imshow(frame)
# plt.title("Raw image")
# plt.axis("off")
# plt.subplot(212)
# plt.imshow(img_undist)
# plt.title("Corrected image")
# plt.axis("off")
# plt.show()


frame = cv2.imread(imagesFolder + "image_336.jpg")
plt.figure()
plt.imshow(frame)
plt.show()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

size_of_marker =  0.0145 # side lenght of the marker in meter
rvecs,tvecs, trash = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

# rvecs     -->
# tvecs     --> 

frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

length_of_axis = 0.01
imaxis = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
for i in range(len(tvecs)):
    imaxis = cv2.drawFrameAxes(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)


plt.figure()
plt.imshow(imaxis)
plt.show()

conn = np.array([0, 1, 2, 3, 0])
plt.figure()
plt.imshow(frame_markers)
plt.legend()
plt.show()


##          DATA  OUTPUTS

data = pd.DataFrame(data = tvecs.reshape(len(tvecs),3), columns = ["tx", "ty", "tz"],
                    index = ids.flatten())
data.index.name = "marker"
data.sort_index(inplace= True)