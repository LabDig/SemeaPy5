import Adafruit_BBIO.GPIO as GPIO
pinOnOffButton="P8_15"
GPIO.setup(pinOnOffButton,GPIO.IN)
while True:
    if GPIO.input(pinOnOffButton):
        print ("on")

