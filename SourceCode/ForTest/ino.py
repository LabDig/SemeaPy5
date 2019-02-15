import serial
import time
ino=serial.Serial("/dev/ttyS2",9600)


while True:
    a=ino.readline()
    print (a)

        

    
