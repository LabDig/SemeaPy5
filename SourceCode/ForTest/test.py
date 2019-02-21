pin=47
dir="in"

file=open("/sys/class/gpio/export","w")
file.write(str(pin)) #pin number
file.close()

file=open("/sys/class/gpio/gpio"+str(pin)+"/direction","w")
file.write(dir) #direction
file.close()
