import Adafruit_BBIO.GPIO as GPIO
import time
pinEncA_Roda="P8_8"
pinEncB_Roda="P9_10"
GPIO.setup(pinEncA_Roda, GPIO.IN)
import Adafruit_BBIO.PWM as PWM
PWM.start("P8_19",0,1000.0)
GPIO.setup("P8_12",GPIO.OUT)
GPIO.output("P8_12",GPIO.LOW)

print ("Start")
A0=-1
i=0
ts=0
aux=0
v=0
count=0
while (True):

    A1=GPIO.input(pinEncA_Roda) #encoder position (0 ou 1)
    if (A0==0 and A1==1): #if have up border
        i=i+1
    if (i==1): #if one up border is detectec start the time
         ts=time.time()
    if (i==20): #at complete three up border, calculate the velocity (one revolution is 20 up border)
        v=(60)/(time.time()-ts)
        print (v)
        i=0
        count=count+1
    A0=A1 #update last status
    time.sleep(10/1000)
    

    if count==0:
        GPIO.output("P8_12",GPIO.HIGH)
        PWM.set_duty_cycle("P8_19",100)
    elif count==50:
        GPIO.output("P8_12",GPIO.LOW)
    
GPIO.cleanup()
