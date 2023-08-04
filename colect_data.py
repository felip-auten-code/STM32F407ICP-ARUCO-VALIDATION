## REQUIRTEMENTS ##
import os
from math import floor
from adafruit_rplidar import RPLidar
import serial

#SAMPLE_TIME = 10

port = input("Enter To Start: ")
lidar = RPLidar(None, "COM8", timeout=3)


def process_data(data, f):
    print(data)
    str_d = '['
    for e in data:
        str_d += str(e)
        str_d += ", "
    str_d += "]\n"
    f.write(str_d)


lidar.connect()
#f = open('./data/scan001.txt', mode = 'w')
#print(lidar.info())
scan_data = [0] * 360
try:
    #    print(lidar.get_info())
    with open('./data/scan0099.txt', mode = 'w') as f:
        for scan in lidar.iter_scans():
            for _, angle, distance in scan:
                scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data, f)

except KeyboardInterrupt:
    print("Stopping.")

lidar.stop()
lidar.disconnect()