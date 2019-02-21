import os
T=1000000 #Period for PWM Signal in ns (Frequency 1 khz)

def StartGPS(gps):
        try:
                file=open("/sys/devices/platform/bone_capemgr/slots","w")
                file.write(gps)
                file.close()
        except:
                print ("Ja Configurado")
def StartPWM(pin,T,pinPWM_Fert,pinPWM_Seed):
        try:
                file=open("/sys/devices/platform/bone_capemgr/slots","w")
                file.write(pin)
                file.close()
                file=open("/sys/class/pwm/pwmchip2/export","w")
                file.write(str(pinPWM_Fert))
                file.close()
                file=open("/sys/class/pwm/pwmchip2/export","w")
                file.write(str(pinPWM_Seed))
                file.close()
                file=open("/sys/class/pwm/pwmchip2/pwm0/period","w")
                file.write(str(T))
                file.close()
                file=open("/sys/class/pwm/pwmchip2/pwm1/period","w")
                file.write(str(T))
                file.close()
                file=open("/sys/class/pwm/pwmchip2/pwm0/enable","w")
                file.write("1")
                file.close()
                file=open("/sys/class/pwm/pwmchip2/pwm1/enable","w")
                file.write("1")
                file.close()
        except:
                print ("Ja configurado")

def StartGPIO(pin):
        
        for i in range(7):
                print (i,pin[2*i],pin[2*i+1])
                try:
                        file=open("/sys/class/gpio/export","w")
                        file.write(str(pin[2*i])) #pin number
                        file.write("\n")
                        file.close()

                        file=open("/sys/class/gpio/gpio"+str(pin[2*i])+"/direction","w")
                        file.write(pin[(2*i)+1]) #direction
                        file.close()
                        file.write("\n")
                except:
                        print ("Ja configurado")

def Button(pin):
        try:
                file=open("/sys/class/gpio/gpio"+str(pin)+"/value","r")
                if file.read()=="1\n": st_button=True
                else:st_button=False
                file.close()
                return st_button
        except:
                st_button=False
                return st_button

def EnableSeed(st):
    if st=="HIGH":
        try:
                file=open("/sys/class/gpio/gpio44/value","w")
                file.write("1")
                file.close()
        except:
                print ("Erro in EnableSeed")

    elif st=="LOW":
        try:
                file=open("/sys/class/gpio/gpio44/value","w")
                file.write("0")
                file.close()
        except:
                print ("Erro in EnableSeed")

def PWMSeed(dt):
        try:
                file=open("/sys/class/pwm/pwmchip2/pwm1/duty_cycle","w")
                file.write(str(int(dt*T)))
                file.close()
        except:
                print ("Erro in PWM Seed")
'''
def EnableFert(st):
        
        if st=="HIGH":
                try:
                        file=open("/sys/class/gpio/gpio45/value","w")
                        file.write("1")
                        file.close()
                except:
                        print ("Erro in EnableFert")
        elif st=="LOW":
                try:
                        file=open("/sys/class/gpio/gpio45/value","w")
                        file.write("0")
                        file.close()
                except:
                        print ("Erro in EnableFert")

def PWMFert(rot,T,max_rot_fert):
        dt=rot/max_rot_fert
        try:
                file=open("/sys/class/pwm/pwmchip2/pwm0/duty_cycle","w")
                file.write(str(int(dt*T)))
                file.close()
        except:
                print ("Erro in PWM Fert")
'''
def Stop(pin):
        for i in range(7):
                try:
                        file=open("/sys/class/gpio/unexport","w")
                        file.write(str(pin[i]))
                        file.close()
                except:
                        print ("Erro ao sair")
        try:
                file=open("/sys/class/pwm/pwmchip2/unexportt","w")
                file.write("0")
                file.close()
                file=open("/sys/class/pwm/pwmchip2/unexportt","w")
                file.write("1")
                file.close()
        except:
                print ("Erro ao sair")
