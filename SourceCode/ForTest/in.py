import Adafruit_BBIO.GPIO as GPIO
import time
pinOnOffButton="P9_27"
pinOnOffButton2="P9_29"
pinOnOffButton3="P8_11"
pinOnOffButton4="P8_12"
GPIO.setup(pinOnOffButton,GPIO.IN)
GPIO.setup(pinOnOffButton2,GPIO.IN)
GPIO.setup(pinOnOffButton3,GPIO.IN)
GPIO.setup(pinOnOffButton4,GPIO.IN)
while True:
    print (GPIO.input(pinOnOffButton))
    print (GPIO.input(pinOnOffButton2))
    print (GPIO.input(pinOnOffButton3))
    print (GPIO.input(pinOnOffButton4))
    print ("......\n")
    time.sleep(2)

