import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
GPIO.setup("P8_12",GPIO.OUT)
GPIO.output("P8_12",GPIO.HIGH)
'''
PWM.start("P8_19",0,1000.0)
PWM.set_duty_cycle("P8_19",0)
'''
