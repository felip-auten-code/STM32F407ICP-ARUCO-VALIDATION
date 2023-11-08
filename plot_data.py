import matplotlib.pyplot as plt
import matplotlib
import math as m
import numpy as np
import os


matplotlib.rcParams['font.family'] = ['sans-serif']
# read file

scan = np.zeros(360)
scan_end = np.zeros(360)
PI=3.1415
SCANS_TOTAL = np.zeros(360 * 135)

COORDINATES_X = np.zeros( 360 )
COORDINATES_Y = np.zeros( 360 )

with open('./data/scanLASTEST01.txt', mode='r') as f:
    ctt_line=0
    for line in f: 
        idx_scan=0
        idx_scan_end=0
        num=""
        for i in line:
            if ( str(i) != '[' and str(i) != ',' and str(i) != ' '):
                num += str(i)
            elif( str(i) == ',' ):
                #print(num)
                if(ctt_line == 185):
                    scan[idx_scan] = float(num)
                    idx_scan+=1
                elif(ctt_line == 187):
                    scan_end[idx_scan_end] = float(num) 
                    idx_scan_end+=1                
                num=""
            if(str(i) == ']'):
                ctt_line+=1
                
print(ctt_line)

def convert_scan_to_cartesian(scan):
    for i in range(0,360):
        rad = i * PI / 180
        COORDINATES_X[i] = scan[i] * m.cos(rad)
        COORDINATES_Y[i] = scan[i] * m.sin(rad) 

    return COORDINATES_X, COORDINATES_Y

def show_down_sampling(scan, mode):
    x, y = convert_scan_to_cartesian(scan)
    
    if mode == '5-mass':
        n_x = np.zeros(72) 
        n_y = np.zeros(72)
        ctt=0
        j=0
        for i in range(0,360):
            if(ctt<5):
                n_x[j]+= x[i]
                n_y[j]+= y[i]
                ctt+=1
            else:
                n_x[j]*=0.2
                n_y[j]*=0.2
                j+=1
                ctt=0
    
        fig, ax = plt.subplots(figsize=(10,7))
        ax.scatter(x, y, label='SAMPLE')
        ax.scatter(n_x, n_y, label="DOWNSAMPLE")
        ax.legend()
        plt.figure(2)
        #plt.show()
    if mode == 'uniform':
        n_x = np.zeros(120) 
        n_y = np.zeros(120)
        ctt=1
        j=0
        for i in range(0,360):
            if((i+1) % 3 == 0 ):
                n_x[j]+= x[i]                                   # alter here to be compatible with stm32f4 implementation
                n_y[j]+= y[i]
                ctt=0
                j+=1
            ctt+=1
            
        print(n_y)
        fig, ax = plt.subplots(1, 2, figsize=(20,5))
        ax[0].scatter(x, y, label='SAMPLE', s=20)
        ax[1].scatter(n_x, n_y, label="DOWNSAMPLE", s= 20, color='tab:orange')
        ax[0].legend()
        ax[1].legend()
        plt.figure(2)


if __name__ == '__main__':

    plt.style.use('seaborn')
    axis_x = np.arange(0,360,1) 
    print(len(scan))
    x, y = convert_scan_to_cartesian(scan)
    fig, ax = plt.subplots(figsize = (10, 7))
    ax.scatter(x, y, label='Escaneamento inicial $(k=0)$', s=10)
    x1, y1 = convert_scan_to_cartesian(scan_end)
    ax.scatter(x1,y1, color='tab:orange', label='Escaneamento final $(k=153)$', s=10)

    ax.set_title('POINT CLOUD DATA')
    ax.set_ylabel('Eixo Y [mm]')
    ax.set_xlabel('Eixo X [mm]')

    ax.legend()
    
    
    plt.figure(1)

    show_down_sampling(scan, 'uniform')
    plt.figure(3)
    
    ax2 = plt.axes()
    for i in range(0, len(x)):
        ax2.arrow(0,0, x[i], y[i])
    plt.xlim(-3000, 3500)
    plt.ylim(-3000, 3500)
    
    plt.figure(4)
    plt.stem(axis_x,scan_end )
    
    plt.show()
    

    #print(x, y)
