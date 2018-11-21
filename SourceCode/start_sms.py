import serial
import time
sim800l = serial.Serial ("/dev/ttyS1", 4800,timeout=0.1) # P9_24 P9_26
sim800l.write(str.encode('AT\r'))
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+CPIN?\r'))
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPINIT'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPPARA=\"URL\",\"http://www.google.com.br"'+'\r'));
time.sleep(0.5)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPACTION'+'\r'));
time.sleep(0.5)
print(sim800l.readall())
sim800l.write(str.encode('AT+HTTPREAD'+'\r'));
time.sleep(0.5)
print(sim800l.readall())
