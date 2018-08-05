#echo BB-UART4 > /sys....
import serial
gps=serial.Serial("/dev/ttyS4",9600)
print (gps.readline())
