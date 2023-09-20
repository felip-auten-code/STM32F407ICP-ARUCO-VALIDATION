import serial
import os
import time
import cv2
import io



 # Alwas edit witch COM PORT IS BEEING USED BY THE TTL(UART/USB) Converter
ser = serial.Serial(port="COM7", baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout = 300)
#sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

data_send =[]

with open('./data/scantestt.txt', mode = 'r') as f:
    ctt=0
    for line in f:
        if(line[0] == '/'):
            break
        #if(ctt % 10 == 0  ):
            #ser.write(line.encode("Ascii"))
            #print(line)
            #data_send.append(line)
            #time.sleep(10)
            #print("open")
        data_send.append(line)
        ctt+=1


list1 = [val for val in data_send for _ in (0, 1)]

ctt=0
#input("Press enter to start")
for i in list1[1:]:
    if(ctt % 2 == 0 ):
        
        #input("Press Enter to continue")    #wait until any key is pressed
        print("Scans will be sent!")
        
    ser.write(i.encode("Ascii"))
    print(i)
            
    ctt+=1
    
    if(ctt%2==0):
        wait_till_return = 1
        #input('wait till processed by MICRO')
        print('wait till processed by MICRO')
        current_line = ""
        current_line = ser.readline()
        #print(current_line)
        with open('./data/STM_ICP_DATA.txt', mode = 'a') as f:
            f.write(current_line.decode("Ascii"))
            print(current_line.decode("Ascii"))
            f.close()



ser.close()