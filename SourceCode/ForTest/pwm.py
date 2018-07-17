import Adafruit_BBIO.PWM as PWM

pinPWM_Seed="P9_14"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
print ("ok")

while True:
    PWM.set_duty_cycle(pinPWM_Seed, 50.0)

    
