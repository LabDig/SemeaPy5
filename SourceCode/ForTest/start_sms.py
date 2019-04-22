import serial
import time
sim800l = serial.Serial ("/dev/ttyS1", 4800,timeout=0.1) # P9_24 P9_26
sim800l.write(str.encode('AT\r'))
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+CCMGF=1\r'))
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+CMGDA="DEL ALL"\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode('AT+CMGS="+5531997349892"\r'));
time.sleep(0.05)
print(sim800l.readall())
sim800l.write(str.encode("OLA"+chr(26)));
time.sleep(0.05)
print(sim800l.readall())
