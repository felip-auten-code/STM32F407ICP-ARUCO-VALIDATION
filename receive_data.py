import serial
import os
import time
import cv2



 # Alwas edit witch COM PORT IS BEEING USED BY THE TTL(UART/USB) Converter
ser = serial.Serial(port="COM4", baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout = 0.2)

data_send =[]
end = 1
var = ''
while(end):
    current_line = ser.readline()
    if(var != ''):
        with open('./data/STM_ICP_DATA.txt', mode = 'w') as f:
            ctt=0
            f.write(current_line)
            print(current_line)
            f.close()



ser.close()


#idx =0
#ctt2=0
#size = len(data_send)
#while(idx < size):
#    i = data_send[idx]
#    if(ctt2 % 2 == 0):
#        print("Press any key to send the 2 sequential Scans")
#        input("Press Enter to continue")    #wait until any key is pressed
#        
#        
#
#    ser.write(i.encode("Ascii"))
#    print(i)
#
#    if(ctt2 % 2 == 0):
#        #wait_till_return = 1
#        #input('wait till processed by MICRO')
#        print('wait till processed by MICRO')
#        current_line = ""
#        current_line = ser.readline()
#        #print(current_line)
#        with open('./data/STM_ICP_DATA.txt', mode = 'a') as f:
#            f.write(current_line.decode("Ascii"))
#            print(current_line.decode("Ascii"))
#            idx-=1
#            f.close()
#    ctt2+=1
#    idx+=1
#
#ser.close()