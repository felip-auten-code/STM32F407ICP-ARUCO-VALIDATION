import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os
import pandas as pd


tabLineErrors = [[]]

allerrors = np.fromfile("./data/errorIterations.txt", dtype='f', sep =',')
#allerrors.count(-9999)

print(np.count_nonzero(allerrors == -9999))

siz = np.count_nonzero(allerrors == -9999)


j=0
for i in range (0, len(allerrors)):
    #tabLineErrors[j].append(allerrors[i])
    if(allerrors[i] == -9999):
        tabLineErrors.append([])
        j+=1
    else:
        tabLineErrors[j].append(allerrors[i])

print(tabLineErrors)

fig1 = plt.figure(1)
plt.plot(range(0, len(tabLineErrors[7]), 1), tabLineErrors[7])
plt.xlabel("Iterações")
plt.ylabel("Erro RMSE [cm]")
plt.show()