import Adafruit_BBIO.GPIO as GPIO
import time
pinEncWhell="P8_11"
GPIO.setup(pinEncWhell, GPIO.IN)
last_st=-1
pulse=0
t_start=0
value=0
def Border():
    global pulse,t_start
    pulse=pulse+1
    if pulse==1: t_start=time.time()
    if pulse==5: #half revolution
        print (round (2.00/(time.time()-t_start),2))
        pulse=0

while True:
    atual_st= GPIO.input(pinEncWhell)
    if atual_st==1 and last_st==0: #border
        Border()
        value=0 #reset logic nivel
    value=value+1#count logic nivel read
    if value>50: #speed is zero
        print (0)
        value=0
    last_st=atual_st
    time.sleep(25/1000)

