#echo BB-UART4 > /sys....
import serial
import time
gps=serial.Serial("/dev/ttyS2",9600)


while True:
        
        nmea=gps.readline()
        
        nmea=nmea.decode('utf-8')
        
       
        print (nmea)
       

 

    
