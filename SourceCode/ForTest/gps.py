#echo BB-UART4 > /sys....
import serial
import time
gps=serial.Serial("/dev/ttyS4",9600)
nmea=''
t=time.time()
while True:
    while ('$GPRMC' in nmea) is False : 
        nmea=gps.readline()
        try:nmea=nmea.decode('utf-8')
        except:
            nmea=gps.readline()
            nmea=nmea.decode('utf-8')
    nmea_array=nmea.split(',')
    data=nmea_array[9]
    print (data)
    hora=str(int(nmea_array[1][0]+nmea_array[1][1])-2)+':'+nmea_array[1][2]+nmea_array[1][3]+':'+nmea_array[1][4]+nmea_array[1][5]
    nmea=''
    t=time.time()
    time.sleep(2.5)
        

    
