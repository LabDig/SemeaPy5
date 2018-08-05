import os
os.system("sh shell.sh")
a=open("/sys/class/gpio/gpio2/value","r")
while True:
        if a.read()=="1\n":
                print ("on")

os.system("sh exit.sh")
