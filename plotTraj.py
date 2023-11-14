import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

Data = pd.read_csv("./data/TvecsARUCO.csv")
ICP_Points = pd.read_table("./data/OutPositions.txt", sep = " ")


Data = Data[1:]
print(ICP_Points.head())


ax = plt.figure(figsize=(10,10)).add_subplot(projection='3d')



ax.scatter(Data["tx"], Data["ty"], Data["tz"])
ax.plot(Data["tx"], Data["ty"], zs=0)
ax.plot(ICP_Points["y"], - ICP_Points["x"])

plt.show()