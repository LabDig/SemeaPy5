import Adafruit_BBIO.GPIO as GPIO
import time
pinEncA_Roda="P9_15"
pinEncB_Roda="P9_17"
GPIO.setup(pinEncA_Roda, GPIO.IN)

print ("Start")
A0=-1
i=0
ts=0
aux=0
while True:
    A1=GPIO.input(pinEncA_Roda) #encoder position (0 ou 1)
    if (A0==0 and A1==1): #if have up border
        i=i+1
    if (i==1): #if one up border is detectec start the time
         ts=time.time()
    if (i==3): #at complete three up border, calculate the velocity (one revolution is 15 up border)
        v=60*.2/(time.time()-ts)
        i=0
    if (A0==A1): #for dectect if encoder its stop, 
        aux=aux+1
    if (aux>100): #if encoder is stop much time, v=0
        aux=0
        v=0
    A0=A1 #update last status
    time.sleep(0.05)
    print ('v==(RPM)==',v)

GPIO.cleanup()
