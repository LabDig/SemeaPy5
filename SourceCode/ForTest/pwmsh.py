#echo PWM2 > /sys...
import Adafruit_BBIO.GPIO as GPIO
pinEnable_Seed="P8_12"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.HIGH)
T=1000000

import os
os.system("sh pwm.sh")
a=open("/sys/class/pwm/pwmchip2/pwm0/period","w")
a.write(str(T))
a.close()

a=open("/sys/class/pwm/pwmchip2/pwm0/duty_cycle","w")
a.write(str(int(0.8*T)))
a.close()

a=open("/sys/class/pwm/pwmchip2/pwm0/enable","w")
a.write("1")
a.close()
i=0
while i<5:
    print (i)
    i=i+1
os.system("sh pwm_exit.sh")
