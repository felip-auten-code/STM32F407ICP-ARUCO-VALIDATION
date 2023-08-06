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

def pose_estimation_aruco(frame, aruco_dict, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    pass

if __name__ == '__main__':
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
