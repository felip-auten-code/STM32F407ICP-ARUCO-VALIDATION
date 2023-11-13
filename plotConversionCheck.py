import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os


matplotlib.rcParams['font.family'] = ['sans-serif']

axisX = np.arange(0, 360, 1)

x =[]
y =[]

x1 =[]
y1 =[]

xT =[]
yT = []


ranges = []

with open("./data/SecondFrame.txt", mode = 'r') as f:
    for line in f:
        xx = 1
        yy = 0
        for coord in line.split(' '):
            if(coord != ''):
                if(xx):
                    x1.append(float(coord)*-1);    
                    yy = 1
                    xx = 0
                elif(yy):
                    y1.append(float(coord))
                    yy = 0
                    xx = 1
        
with open("./data/TransformedFrame.txt", mode = 'r') as f:
    for line in f:
        xx = 1
        yy = 0
        for coord in line.split(' '):
            if(coord != ''):
                if(xx):
                    xT.append(float(coord)*-1);    
                    yy = 1
                    xx = 0
                elif(yy):
                    yT.append(float(coord))
                    yy = 0
                    xx = 1
        
    

with open("./data/CheckConversionToCartesian.txt", mode = 'r') as f:
    for line in f:
        xx = 1
        yy = 0
        for coord in line.split(' '):
            if(coord != ''):
                if(xx):
                    x.append(float(coord)*-1);    
                    yy = 1
                    xx = 0
                elif(yy):
                    y.append(float(coord))
                    yy = 0
                    xx = 1

with open('./data/CheckRanges.txt', mode ='r') as f:
    for line in f:
        for item in line.split(','):
            if(item != ''):
                print(float(item))
                ranges.append(float(item))

#print (x)
#print (y)

plt.figure(1)
plt.plot(x, y, '.', label = "Frame Anterior")
plt.plot(x1, y1, '.', label = "Frame Atual")
plt.plot(xT, yT, '.', label = "Após transformação aplicada pelo ICP")
plt.legend()

plt.figure(2)
plt.plot(axisX, ranges)
plt.show()
