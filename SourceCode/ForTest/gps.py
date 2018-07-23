import serial
import Adafruit_BBIO.UART as UART
import time
UART.setup("UART1")

ser=serial.Serial('/dev/ttyS4',9600)
while True:
        nmea=ser.readline()
        nmea=nmea.decode("utf-8")
        print (nmea)
