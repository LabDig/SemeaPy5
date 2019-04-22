import Adafruit_BBIO.GPIO as GPIO
import time
pinEncA_Roda="P8_7"
pinEncB_Roda="P8_9"
GPIO.setup(pinEncA_Roda, GPIO.IN)
GPIO.setup(pinEncB_Roda, GPIO.IN)

print ("Start")
file="res4.txt"
a=open(file,"w")
a.write('')
a.close()

while True:
    
    A1=GPIO.input(pinEncA_Roda) #encoder position (0 ou 1)
    a=open(file,"a")
    a.write(str(time.time())+','+str(A1)+'\n')
    a.close()
    time.sleep(10/1000)
