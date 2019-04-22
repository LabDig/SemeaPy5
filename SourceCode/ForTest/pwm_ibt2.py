import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
en="P8_10"
pwm="P8_13"
GPIO.setup(en,GPIO.OUT)
GPIO.output(en,GPIO.HIGH)
PWM.start(pwm,0,1000.0)
PWM.set_duty_cycle(pwm,100)
time.sleep(20)
GPIO.output(en,GPIO.LOW)
PWM.set_duty_cycle(pwm,0)
