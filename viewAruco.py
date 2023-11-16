import cv2
import matplotlib.pyplot as plt
import scipy as ss
import pandas as pd
import numpy as np

axisX = np.arange(0, 23, 1)

ArucoData = pd.read_csv("./data/TvecsARUCO.csv")                                    # ARUCO DATA
RotData = pd.read_csv("./data/RvecsARUCO.csv")
ICP_Points = pd.read_table("./data/OutPositions2.txt", sep = " ")                   # ICP trajectory
ICP_output = pd.read_table("./data/PC_ICP_output8.txt", sep = " ")

print(ICP_output.head())

now_ang = [0]
increase_ang =[]
for i in ICP_output["theta"]:
    increase_ang.append(i)
for i in range(1, len(increase_ang)):
    now_ang.append(now_ang[i-1] + increase_ang[i-1])


now_ang = now_ang[1:]
ANG = pd.DataFrame(now_ang)
print(now_ang, " ", len(now_ang))

samples = len(ICP_Points)
sigX = ArucoData["tx"]
sigX = sigX[1:]
sigY = ArucoData["ty"]
sigY = sigY[1:]

sigTheta = RotData["RotZ"]

axisSample = np.arange(0, len(sigTheta), 1)

#print(sigX)
X = ss.signal.resample(sigX, samples)
Y = ss.signal.resample(sigY, samples)
THETA = ss.signal.resample(sigTheta, samples)

Y = Y[1:]
X = X[1:]
THETA = THETA[1:]
#print(X)

print(THETA)

xx = ICP_Points["x"]
yy = ICP_Points["y"]
xx = xx[1:]
yy = yy[1:]

plt.figure(1)
plt.plot(X, Y)
plt.plot(yy, - xx )                     # the trasformation to same coodinates is only here (ztheta(PI/2)  90°)


ICP_Points["difX"] =  yy.sub(X, axis=0)
ICP_Points["difY"] = -xx.sub(Y, axis=0)

diffTheta = ANG.sub(THETA, axis=0)
#diffTheta = diffTheta[1:]
xdiff = ICP_Points["difX"]
ydiff = ICP_Points["difY"]
xdiff = xdiff[1:]
ydiff = ydiff[1:]

plt.figure(2)
plt.title("Reamostragem da estimativa da componente angular \n pela odometria visual")
plt.plot(axisSample, sigTheta)
plt.plot(axisX*30, THETA, '.')
plt.figure(3)
plt.title("Erro em radianos da estimativa do algoritmo ICP \n em relação à odometria visual")
plt.plot(axisX, diffTheta)


plt.figure(4)
plt.title("Erro em x [cm]")
plt.plot(axisX, xdiff)

plt.figure(5)
plt.title("Erro em y [cm]")
plt.plot(axisX, ydiff, '.')

plt.figure(6)
plt.title("Erro em relação aos parâmetros x e y [cm]")
plt.boxplot([xdiff, ydiff], labels=["Erro em x [cm]", "Erro em y [cm]"])




plt.show()