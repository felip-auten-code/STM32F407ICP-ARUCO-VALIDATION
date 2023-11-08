import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os

start_position = [0, 0]
start_orientation = [0]

track_position = [0, 0]
track_orientation = [0]

all_2D_transformations = []
for i in range(0,200):
    all_2D_transformations.append([0,0, 0])

all_positions = []
for i in range(0,200):
    all_positions.append([0,0])

all_positions_x = []
all_positions_y = []
for i in range(0,200):
    all_positions_x.append(0.)
    all_positions_y.append(0.)
#print(all_2D_transformations)

with open('./data/PC_ICP_output5.txt', mode = 'r') as f:
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


for i in range(1,200):
    all_positions_x[i] = all_positions_x[i-1] + all_2D_transformations[i-1][0]
    all_positions_y[i] = all_positions_y[i-1] + all_2D_transformations[i-1][1]

print(all_positions_y)

plt.plot(all_positions_x, all_positions_y)
plt.show()