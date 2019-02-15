import Adafruit_BBIO.GPIO as GPIO
import time
from statistics import mean
pinEncWhell="P9_29"
GPIO.setup(pinEncWhell, GPIO.IN)
last_st=GPIO.input(pinEncWhell)
atual_st= GPIO.input(pinEncWhell)
ton=0;
toff=0;
t1,t2,v0,v=0,0,0,0
import Adafruit_BBIO.PWM as PWM
PWM.start("P8_13",0,1000.0)
GPIO.setup("P8_10",GPIO.OUT)
GPIO.output("P8_10",GPIO.HIGH)

pinOnOffButton="P8_16"
GPIO.setup(pinOnOffButton, GPIO.IN)

while GPIO.input(pinOnOffButton):
    PWM.set_duty_cycle("P8_13",100)
    atual_st=GPIO.input(pinEncWhell)
    if last_st==0 and atual_st==1: #up
        t3=t1
        toff=time.time()
        t1=time.time()-ton
        print (t1,t2)
        '''
        if (t1>0.05 and t2>0.05 and t1/t2>0.85 and t1/t2<1.15) :
            v0=v
            v=(60/9.0)/((t1+t2)/2)
            print (round((v+v0)/2,1))
        '''
    if last_st==1 and atual_st==0: #down
        ton=time.time()
        t2=time.time()-toff
    last_st=atual_st
    time.sleep(10/1000) #1ms

GPIO.output("P8_10",GPIO.LOW)
PWM.set_duty_cycle("P8_13",0)
