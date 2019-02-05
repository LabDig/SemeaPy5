import serial
import time

SERIAL_PORT = "/dev/ttyS1"
sim800l = serial.Serial(SERIAL_PORT, baudrate = 9600,timeout=0.05) #if not timeout, stop the readline
print ("Conf")
sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
time.sleep(1)
print ("clear")
sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r')) 
time.sleep(1)
print (sim800l.readall()) #clear buffer
m='nnn'
i=0
while i<50:
    t=time.time()
    i=i+1 
    link="http://andrecoelho.tech/SemeaView/send_mysql.php?LogID="+str(i)
    sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+'\r'))
    time.sleep(5)
    sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
    time.sleep(5)
    time.sleep(1)
    a=sim800l.readall()
    a=a.decode('utf-8')
    a=a.split("\r\n")
    print (i,round(time.time()-t,2),a)
print ("inish")
sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
