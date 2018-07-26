import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
pinPWM_Seed="P9_16"
pinEnable_Seed="P9_18"
pinEnable_Seed2="P9_20"
PWM.start(pinPWM_Seed,0, 500.0) #pin, duty,frequencia
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.setup(pinEnable_Seed2, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.HIGH)
GPIO.output(pinEnable_Seed2,GPIO.HIGH)
print ("ok")

PWM.set_duty_cycle(pinPWM_Seed, 30.0)
time.sleep(100)    
GPIO.output(pinEnable_Seed,GPIO.LOW)
GPIO.output(pinEnable_Seed2,GPIO.LOW)  
