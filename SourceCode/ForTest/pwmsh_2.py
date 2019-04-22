import Adafruit_BBIO.GPIO as GPIO
pinEnable_Seed="P8_12"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.HIGH)
T=1000000
import os
import time
a=open("/sys/class/pwm/pwmchip2/export","w")
a.write("0")
a.close()
print ("OK")
a=open("/sys/class/pwm/pwmchip2/pwm0/period","w")
a.write(str(T))
a.close()
print ("OK")
a=open("/sys/class/pwm/pwmchip2/pwm0/enable","w")
a.write("1")
a.close()
a=open("/sys/class/pwm/pwmchip2/pwm0/duty_cycle","w")
a.write(str(T))
a.close()
print ("OK")
i=0
while i<5:
    print (i)
    i=i+1
    time.sleep(1)
'''
a=open("/sys/class/pwm/pwmchip2/unexport","w")
a.write("0")
a.close()
GPIO.output(pinEnable_Seed,GPIO.HIGH)
'''
