import serial
import keyboard

ser = serial.Serial(port="COM4", baudrate=115200, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout = 0.2)



with open('./data/scanT86mmR28g.txt', mode = 'r') as f:
    for line in f:
        if(line[0] == '/'):
            break
        ser.write(line.encode("Ascii"))
        print(line)

ser.close()