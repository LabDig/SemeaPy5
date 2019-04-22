import Adafruit_BBIO.GPIO as GPIO
import time
channel="P9_12"
GPIO.setup(channel,GPIO.IN)
i=0

def my_call(channel):
    print ("up")

GPIO.add_event_detect(channel,GPIO.RISING,callback=my_call,bouncetime=100)

while True:
    print ('...')
    time.sleep(2)
'''
GPIO.setup(pinOnOffButton,GPIO.IN)

def a:
    print ("up")
    
GPIO.add_event_detect(pinOnOffButton,GPIO.RISING,callback=a,bouncetime=100)
'''
