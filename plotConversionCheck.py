import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os
from statistics import mean


matplotlib.rcParams['font.family'] = ['sans-serif']

axisX = np.arange(0, 360, 1)

x =[]
y =[]

x1 =[]
y1 =[]

xT =[]
yT = []

xU =[]
yU =[]


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
        

with open("./data/UpdatedFrame.txt", mode = 'r') as f:
    for line in f:
        xx = 1
        yy = 0
        for coord in line.split(' '):
            if(coord != ''):
                if(xx):
                    xU.append(float(coord)*-1);    
                    yy = 1
                    xx = 0
                elif(yy):
                    yU.append(float(coord))
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
def centroid(vec):
    xU =0.
    yU =0.
    c = 0
    for i in vec:
        xU+= i[0]
        yU+= i[1] 
        c+=1
    if(c > 0):
        xU = float(xU/ c)
        yU = float(yU/ c)
    return xU, yU


cloud_1 = np.array([])

for i in range(0, len(x)):
    np.append(cloud_1, [x[i],y[i]])

cloud_2 = np.array([])

for i in range(0, len(xT)):
    np.append(cloud_2, [xT[i],yT[i]])


plt.figure(1)
plt.plot(x, y, '.', label = "Frame Anterior")
plt.plot(x1, y1, '.', label = "Frame Atual")
#plt.plot(xT, yT, '.', label = "Após transformação aplicada pelo ICP")
plt.plot(xU, yU, '.', label = "Frame anterior após o a linhamento final \n aplicado pelo ICP implementado")
plt.xlabel("Eixo x do sensor LiDAR [cm]")
plt.ylabel("Eixo y do sensor LiDAR [cm]")
plt.legend()




# plt.figure(2)
# plt.plot(axisX, ranges)
plt.show()
