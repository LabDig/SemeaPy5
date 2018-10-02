import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
GPIO.setup("P8_10",GPIO.OUT)
GPIO.output("P8_10",GPIO.HIGH)
PWM.start("P8_19",0,1000.0)
PWM.set_duty_cycle("P8_19",100)
time.sleep(10)
GPIO.output("P8_10",GPIO.LOW)
