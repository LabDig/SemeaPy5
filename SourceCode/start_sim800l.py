import serial
import time
sim800l = serial.Serial ("/dev/ttyS1", 4800,timeout=0.1) # P9_24 P9_26
sim800l.write(str.encode('AT\r'))
print(sim800l.readall())
sim800l.write(str.encode('AT+COPS?\r'))
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPINIT'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"URL\",\"http://andrecoelho.tech"'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPACTION'+'\r'));
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPREAD'+'\r'));
print(sim800l.readall())
