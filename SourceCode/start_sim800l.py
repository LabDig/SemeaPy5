import serial
import time
SERIAL_PORT = "/dev/ttyS1"
sim800l = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 0.1)
def Start3G():
    sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
    time.sleep(0.1)
    sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
    time.sleep(0.1)
    sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
    time.sleep(0.1)
    sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
    time.sleep(0.1)
    sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
    time.sleep(0.1)
    sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
    time.sleep(0.1)

