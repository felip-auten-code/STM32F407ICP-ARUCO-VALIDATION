import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

Data = pd.read_csv("./data/TvecsARUCO.csv")                                 # ARUCO DATA

ICP_Points = pd.read_table("./data/OutPositions5.txt", sep = " ")            # ICP trajectory


Data = Data[1:]
print(ICP_Points.head())


ax = plt.figure(figsize=(8,8)).add_subplot(projection='3d')

ax.set_ylabel('x [cm]', size = 10)
ax.set_xlabel('y [cm]')
ax.set_zlabel('z [cm]')

ax.scatter(Data["tx"], Data["ty"], Data["tz"])
ax.plot(Data["tx"], Data["ty"], zs=0)
#ax.plot(ICP_Points["y"], - ICP_Points["x"])                     # the trasformation to same coodinates is only here (ztheta(PI/2)  90°)

plt.show()