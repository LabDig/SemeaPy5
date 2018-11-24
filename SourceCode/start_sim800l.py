import serial
import time
SERIAL_PORT = "/dev/ttyS1"    # Raspberry Pi 3

sim800l = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 5)
sim800l.write(str.encode('AT\r'))
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+CPIN?\r'))
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPINIT'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"URL\",\"http://andrecoelho.tech/envia_mysql_hostinger.php?MachineID=K02&FieldID=F01&Lati=0.0&Longi=0.0&XUtm=0.0&YUtm=0.0&Speed=0.0&OpCap=0.0&TimeOperation=0.0&Population=0&FertRatio=0&FertLevel=1&Area=1"'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPACTION'+'\r'));
time.sleep(3)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPREAD'+'\r'));
time.sleep(3)
print(sim800l.readall())
