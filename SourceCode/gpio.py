import os

def Button():
	file=open("/sys/class/gpio/gpio49/value","r")
	if file.read()=="1\n": st_button=True
	else:st_button=False
	file.close()
	return st_button

def EnableSeed(st):
    if st=="HIGH":
        file=open("/sys/class/gpio/gpio44/value","w")
        file.write("1")
        file.close()
    elif st=="LOW":
        file=open("/sys/class/gpio/gpio44/value","w")
        file.write("0")
        file.close()

def PWMSeed(rot,T,max_rot_seed):
        dt=rot/max_rot_seed
        file=open("/sys/class/pwm/pwmchip2/pwm1/duty_cycle","w")
        file.write(str(int(dt*T)))
        file.close()

def EnableFert(st):
    if st=="HIGH":
        file=open("/sys/class/gpio/gpio45/value","w")
        file.write("1")
        file.close()
    elif st=="LOW":
        file=open("/sys/class/gpio/gpio45/value","w")
        file.write("0")
        file.close()

def PWMFert(rot,T,max_rot_fert):
        dt=rot/max_rot_fert
        file=open("/sys/class/pwm/pwmchip2/pwm0/duty_cycle","w")
        file.write(str(int(dt*T)))
        file.close()

