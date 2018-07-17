import serial
import Adafruit_BBIO.UART as UART
import time
UART.setup("UART1")

ser=serial.Serial('/dev/ttyS4',9600)

print ("GPS Start")

while True:

	ser.flushInput()
	ser.flushOutput()
	while ser.inWaiting()==0:
		pass
	nmea=ser.readline()
	print (nmea)
