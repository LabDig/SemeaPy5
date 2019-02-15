import Adafruit_BBIO.GPIO as GPIO
import time
from statistics import mean
pinEncWhell="P8_11"
GPIO.setup(pinEncWhell, GPIO.IN)
last_st=GPIO.input(pinEncWhell)
atual_st= GPIO.input(pinEncWhell)
ton=0;
toff=0;
t1,t2,v0,v=0,0,0,0
tz=0
while True:
    atual_st=GPIO.input(pinEncWhell)
    if last_st==0 and atual_st==1: #up
        tz=time.time()
        t3=t1
        toff=time.time()
        t1=time.time()-ton
        #if (t1>0.05 and t2>0.05 and t1<1000 and t2<1000):
        if (t1>0.05 and t2>0.05 and t1/t2>0.85 and t1/t2<1.15) :
            v0=v
            v=(2.0/9.0)/((t1+t2)/2)
            print (round((v+v0)/2,1))
    if last_st==1 and atual_st==0: #down
        ton=time.time()
        t2=time.time()-toff
    if time.time()-tz > 1.5:print (0)
    last_st=atual_st
    time.sleep(10/1000) #1ms

