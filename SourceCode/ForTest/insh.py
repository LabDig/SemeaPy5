import os
os.system("sh shell.sh")
while True:
	a=open("/sys/class/gpio/gpio49/value","r")
	st=a.read()
	print (st)	
	if (st=="1\n"):
		print("on")
	else:
		print ("off")
	a.close()
os.system("sh exit.sh")
