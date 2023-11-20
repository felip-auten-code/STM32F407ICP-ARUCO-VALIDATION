import cv2
import matplotlib.pyplot as plt
import scipy as ss
import pandas as pd
import numpy as np



ArucoData = pd.read_csv("./data/TvecsARUCO.csv")                                    # ARUCO DATA
RotData = pd.read_csv("./data/RvecsARUCO.csv")
ICP_Points = pd.read_table("./data/OutPositions5.txt", sep = " ")                   # ICP trajectory
ICP_output = pd.read_table("./data/PC_ICP_output10.txt", sep = " ")

print(ICP_output.head())

now_ang = [0]
increase_ang =[]
for i in ICP_output["theta"]:
    increase_ang.append(i)
for i in range(1, len(increase_ang)):
    now_ang.append(now_ang[i-1] + increase_ang[i-1])


now_ang = now_ang[1:]
ANG = pd.DataFrame(now_ang)
#print(now_ang, " ", len(now_ang))

samples = len(ICP_Points)
sigX = ArucoData["tx"]
sigX = sigX[1:586]
sigY = ArucoData["ty"]
sigY = sigY[1:586]

sigTheta = RotData["RotZ"]
sigTheta= sigTheta [1:586]
axisSample = np.arange(0, len(sigTheta[0:585]), 1)
axisX = np.arange(0, len(ICP_Points)-1, 1)


#print(sigX)
X = ss.signal.resample(sigX[0:585], samples)
Y = ss.signal.resample(sigY[0:585], samples)
THETA = ss.signal.resample(sigTheta[0:585], samples)

Y = Y[1:]
X = X[1:]
THETA = THETA[1:]
#print(X)

#print(THETA)

xx = ICP_Points["x"]
yy = ICP_Points["y"]
xx = xx[1:]
yy = yy[1:]

plt.figure(1)
# plt.plot(X, Y)
# plt.plot(yy, - xx, 'black' )                     # the trasformation to same coodinates is only here (ztheta(PI/2)  90°)
plt.plot(-Y, X, '.',  color="green", label="Amostras odometria visual")
plt.plot(xx, yy, '.',  color="blue",  label="Amostras odometria LiDAR" )                     # the trasformation to same coodinates is only here (ztheta(PI/2)  90°)



ICP_Points["difX"] =  yy.sub(X, axis=0)
ICP_Points["difY"] = xx.sub(-Y, axis=0)

diffTheta = ANG.sub(THETA, axis=0)
#diffTheta = diffTheta[1:]
xdiff = ICP_Points["difX"]
ydiff = ICP_Points["difY"]
xdiff = xdiff[1:]
ydiff = ydiff[1:]

print(ICP_Points.head(60))
#aux = plt.axes()
auxX = np.array(xx)
auxY = np.array(yy)
auxDX = np.array(-Y)
auxDY = np.array(X)


vecdiff = []                                # distancia translacional modulo cartesiano
x_D = np.array(xdiff)
y_D = np.array(xdiff)
print(x_D)
for i in range(0, len(xdiff), 1):
    distancevec = np.sqrt(pow(x_D[i], 2) + pow(y_D[i], 2))              # modulo cartesiano
    vecdiff.append(distancevec)


for i in range (1,  len(xx)):
    plt.plot( [auxX[i], auxDX[i]], [auxY[i], auxDY[i]] , '--',  color='red')

plt.legend(["Amostras da trajetória \n da odometria visual", "Amostras da trajetória \n da odometria LidAR", "Distâncias translacionais"])
plt.xlabel("x [cm]")
plt.ylabel("y [cm]")
plt.legend()

plt.figure(2)
plt.title("Reamostragem da estimativa da componente angular θ \n pela odometria visual (rotação em relação ao eixo z)")
plt.plot(axisSample / 30, sigTheta, '.', label= "Amostras")
plt.plot(axisX* (len(axisSample)/len(axisX)) / 30, THETA, '.', label= "Reamostragem para sincronização")
plt.ylabel("valor [rad]")
plt.xlabel("tempo [s]")
plt.legend()


plt.figure(1234)
plt.plot(-Y, X, '-',  color="green",  label="Trajetória gerada pela odometria visual")
plt.plot(xx, yy, '-',  color="blue",  label="Trajetória gerada pela odometria LiDAR" )      
plt.xlabel("x [cm]")
plt.ylabel("y [cm]")
plt.legend()


plt.figure(13334)
plt.plot(-Y, X, '.',  color="green",  label="Amostras da trajetória da odometria visual")
plt.plot(xx, yy, '.',  color="blue",  label="Amostras da trajetória da odometria LiDAR" )      
plt.xlabel("x [cm]")
plt.ylabel("y [cm]")
plt.legend()

plt.figure(12)
plt.title("Reamostragem da coordenada X (da odometria visual)")
plt.plot(axisSample / 30, sigX, '.', label= "Amostras")
plt.plot(axisX* (len(axisSample)/len(axisX)) / 30 , X, '.', label= "Reamostragem para sincronização")
#plt.plot(axisX* (len(axisSample)/len(axisX)), yy, '.')
plt.ylabel("valor [cm]")
plt.xlabel("tempo [s]")
plt.legend()

plt.figure(13)
plt.title("Reamostragem da coordenada Y (da odometria visual)")
plt.plot(axisSample/ 30, sigY, '.', label= "Amostras")
plt.plot(axisX*  (len(axisSample)/len(axisX)) / 30 , Y, '.', label= "Reamostragem para sincronização")
#plt.plot(axisX*  (len(axisSample)/len(axisX)), -xx, '.')
plt.ylabel("valor [cm]")
plt.xlabel("tempo [s]")
plt.legend()

plt.figure(3)
plt.title("Erro em radianos da estimativa do algoritmo ICP \n em relação à odometria visual")
plt.plot(axisX*.3, abs(diffTheta))
plt.ylabel("Erro absoluto [rad]")
plt.xlabel("Tempo [s]")

plt.figure(4)
plt.title("Erro em relação a coordenada x [cm]")
plt.plot(axisX*0.3, (abs(xdiff)) )
plt.ylabel("Erro absoluto [cm]")
plt.xlabel("Tempo [s]")


plt.figure(5)
plt.title("Erro em relação a coordenada y [cm]")
plt.plot(axisX*0.3, abs(ydiff), '-')
plt.ylabel("Erro absoluto [cm]")
plt.xlabel("Tempo [s]")

## BOXPLOT
plt.figure(6)
plt.title("Erro em relação aos parâmetros x e y [cm]")
plt.boxplot([abs(xdiff), abs(ydiff)], labels=["Distribuição do erro \n do parâmetro x", "Distribuição do erro\n do parâmetro y"])
plt.ylabel("Erro absoluto [cm]")


plt.figure(7)
plt.title("Erro em relação ao parâmetro θ [rad]")
plt.boxplot(abs(diffTheta), labels=["Distribuição do erro angular em radianos [rad]"])
plt.ylabel("Erro absoluto [rad]")

plt.figure(17)
plt.title("Erro em relação ao parâmetro θ [°]")
plt.boxplot(abs(diffTheta)*180/np.pi, labels=["Distribuição do erro angular em graus [°]"])
plt.ylabel("Erro absoluto [°]")

plt.figure(8)
plt.title("Erro translacional")
plt.plot(axisX, vecdiff)
plt.ylabel("Erro absoluto [cm]")

plt.show()