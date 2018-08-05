#echo PWM2 > /sys...


import os
os.system("sh pwm.sh")
a=open("/sys/class/pwm/pwmchip2/pwm0/period","w")
a.write("1000000000")
a.close()

a=open("/sys/class/pwm/pwmchip2/pwm0/duty_cycle","w")
a.write("800000000")
a.close()

a=open("/sys/class/pwm/pwmchip2/pwm0/enable","w")
a.write("1")
a.close()

os.system("sh pwm_exit.sh")
