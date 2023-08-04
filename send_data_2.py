import serial
import keyboard
import time

ser = serial.Serial(port="COM8", baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout = 0.2)

data_send =[]

with open('./data/scan002.txt', mode = 'r') as f:
    ctt=0
    for line in f:
        if(line[0] == '/'):
            break
        if(ctt % 10 == 0  ):
            #ser.write(line.encode("Ascii"))
            #print(line)
            data_send.append(line)
            #time.sleep(10)
            #print("open")
        ctt+=1
ctt=1
for i in data_send:
    print(i)
    ser.write(i.encode("Ascii"))
    if(ctt % 2 == 0):
        time.sleep(60)
    ctt+=1


ser.close()