
import cv2
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import time
import os


imagesFolder = "./data/CharucoBoard"
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((3,3) , 1, 0.8, aruco_dict)
imboard = board.generateImage((4000, 4000))
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
ax.axis("off")
cv2.imwrite(imagesFolder + "/chessboard.tiff",imboard)
plt.savefig(imagesFolder + "/chessboard.pdf")
plt.grid()
plt.show()
print("Imprimer le damier de calibration!")