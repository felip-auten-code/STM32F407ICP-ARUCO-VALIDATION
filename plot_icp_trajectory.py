import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os
import pandas as pd

start_position = [0, 0]
start_orientation = [0]

track_position = [0, 0]
track_orientation = [0]

all_2D_transformations = []
for i in range(0,200):
    all_2D_transformations.append([0, 0, 0])

all_positions = []
for i in range(0,200):
    all_positions.append([0,0])

all_positions_x = []
all_positions_y = []
for i in range(0,200):
    all_positions_x.append(0.)
    all_positions_y.append(0.)
#print(all_2D_transformations)

with open('./data/PC_ICP_output9.txt', mode = 'r') as f:
    ctt_line=0
    for line in f:
        ctt_item =0
        for item in line.split(' '):
            #item.remove(' ')
            item.replace(" ", "")
            if(item != ''):
                value = float(item)
                #print( value, '\t', end="")
                all_2D_transformations[ctt_line][ctt_item] = value
                ctt_item+=1
        ctt_line+=1
        print()


def getTransformation(vec3):
    T = np.array([  [ np.cos(vec3[2]), -np.sin(vec3[2]), vec3[0] ],
                    [ np.sin(vec3[2]),  np.cos(vec3[2]), vec3[1] ],
                    [ 0              ,  0              , 1       ] ])
    #print(T)
    return T


def computeTransform(points, transf):
    temp=[]
    for i in points:
        # print(i)
        T = getTransformation(transf)
        temp = (np.dot(T, points))
    return temp


tt = getTransformation([1., 1., 0.453])

tr = computeTransform([1,1,1], [1., 1., 0.453])
print(tr)
for i in range(1,200):
    transformationM = getTransformation(all_2D_transformations[i-1])
    sx = np.sqrt(pow(transformationM[0][0], 2) + pow(transformationM[1][0], 2))
    sy = np.sqrt(pow(transformationM[0][1], 2) + pow(transformationM[1][1], 2))
    all_positions_x[i] =  (all_positions_x[i-1] * np.cos(all_2D_transformations[i-1][2]) - all_positions_y[i-1] * np.sin(all_2D_transformations[i-1][2])) + all_2D_transformations[i-1][0] 
    all_positions_y[i] =  (all_positions_x[i-1] * np.sin(all_2D_transformations[i-1][2]) + all_positions_y[i-1] * np.cos(all_2D_transformations[i-1][2])) + all_2D_transformations[i-1][1] 
    #all_positions_x[i] =  all_positions_x[i-1] + all_2D_transformations[i-1][0]
    #all_positions_y[i] =  all_positions_y[i-1] + all_2D_transformations[i-1][1]

Points2d = pd.read_table("./data/OutPositions2.txt", sep = " ")
#print(Points2d.head())

plt.plot(Points2d["x"], Points2d["y"], '.')
plt.plot(all_positions_x, all_positions_y, '.')

plt.show()