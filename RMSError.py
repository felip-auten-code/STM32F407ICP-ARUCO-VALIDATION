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
## Explore iterations --> 20, 
call = 16
interval =3

plt.plot(range(0, len(tabLineErrors[call]), 1), tabLineErrors[call], 'x')
plt.plot(range(0, len(tabLineErrors[call]), 1), tabLineErrors[call], '--')
s = str(call*interval)
s1 = str(call*interval + interval)
title = "Chamada (" + s + ", " + s1 + ") ao algoritmo ICP"
plt.title( title )
plt.xlabel("Iterações")
plt.ylabel("Erro RMSE [cm]")
plt.show()