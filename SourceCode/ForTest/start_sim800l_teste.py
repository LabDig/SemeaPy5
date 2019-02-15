import Adafruit_BBIO.GPIO as GPIO
import serial
import time
pinOnOffButton="P8_16"
GPIO.setup(pinOnOffButton,GPIO.IN)
SERIAL_PORT = "/dev/ttyS1"
sim800l = serial.Serial(SERIAL_PORT, baudrate = 9600,timeout=0.05) #if not timeout, stop the readline
i=0
ti=0
tf=0
error=0
while GPIO.input(pinOnOffButton):
    if i==2 :sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
    if i==4 : sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
    if i==6 : sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
    if i==8 : sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
    if i==10 :sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
    if i==12 :sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
    link='http://andrecoelho.tech/SemeaView/send_mysql.php?'
    str_data='LogID='+str(i)+'&Date='+str(i)+'&Time='+str(i)+'&MachineID='+'i'\
+'&FieldID='+'i'+'&Lati='+str(i)+'&Longi='+str(i)+'&XUtm='+str(i)+'&YUtm='+str(i)+'&Speed='+\
str(i)+'&OpCap='+str(i)+'&TimeOperation='+str(i)+'&Population='+str(i)+'&FertRatio='+\
str(i)+'&FertWgt='+str(i)+'&Area='+str(i)
    if i==18: sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+str_data+'\r'))
    if i==23:
        sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
        i=13
    rec=sim800l.readline()
    rec=rec.decode('utf-8')
    if '+SAPBR:' in rec: print (rec)
    if '+HTTPACTION:' in rec:
        tf=time.time()
        print ('tempo',round(tf-ti,1))
        ti=time.time()
        if "200" in rec :
            error=0
            print ('Enviado')
        if not "200" in rec :
            error=error+1
            print ('Erros',error)
    if error==3 :
        sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
        print ("Falha 3 vezes ao enviar. Resetar")
        error=0
        i=0
    i=i+1
    time.sleep(1)

print ("finish")
sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
print ("close")
