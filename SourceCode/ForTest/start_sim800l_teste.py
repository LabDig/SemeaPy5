import serial
import time
import start_sim800l

#start_sim800l.Start3G()

SERIAL_PORT = "/dev/ttyS1"
sim800l = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 0.1)
link="http://andrecoelho.tech/envia_mysql_hostinger.php?MachineID=K02&FieldID=F01&Lati=0.0&\Longi=0.0&XUtm=0.0&YUtm=0.0&Speed=11.1&OpCap=0.0&TimeOperation=0.0&Population=0&FertRatio=0&FertLevel=1&Area=1"
sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
time.sleep(1)
sim800l.write(str.encode('AT+HTTPREAD'+'\r'))
time.sleep(1)
a=sim800l.readall()
a=a.decode('utf-8')
a=a.split('\n')
for i in a:
    print (i)

