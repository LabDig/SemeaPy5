import Adafruit_BBIO.GPIO as GPIO
pinOnOffButton="P9_12"
GPIO.setup(pinOnOffButton,GPIO.IN)
while True:
    if GPIO.input(pinOnOffButton):
        print ("on")

