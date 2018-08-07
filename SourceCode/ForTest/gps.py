#echo BB-UART4 > /sys....
import serial
import time
gps=serial.Serial("/dev/ttyS4",9600)
while True:
    print (gps.readline())
    time.sleep(1)
