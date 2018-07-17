#for test
# 4095 === 1.8 V
# 2048 === 0.9 V === 15 kg
#  1  ==== x =  7 g

import Adafruit_BBIO.ADC as ADC
ADC.setup()
import time
import numpy as np
from sklearn import linear_model # pip install sklearn  apt-get install python3-scipy

#Calibration
massa = np.array([ 700, 4200, 8100, 15900 ])
voltage = np.array([ 0.07,0.26,0.54,0.59])

voltage=voltage.reshape(1,-1)
model = linear_model.LinearRegression()
model.fit(voltage.T, massa)


max_samples = 50 # window for movel average filter

value_array=np.array(ADC.read("P9_33")) #first value
file="res.txt"
res=open(file,"a")
res.write("Cabecalho")
res.write("\n")
res.close()

ti=time.time()
while True:
    value=ADC.read("P9_33")

    value_array = np.append(value_array, value)

    avg_value = np.mean(value_array)

    if len(value_array) == max_samples:
        value_array = np.delete(value_array, 0) # delete the first value


    massa=avg_value*model.coef_[0] + model.intercept_ #massa (g) =x (v) * a +b
    print (time.time()-ti,value,avg_value,massa)
    res=open(file,"a")
    res.write(str(time.time()-ti))
    res.write(",")
    res.write(str(value))
    res.write(",")
    res.write(str(avg_value))
    res.write("\n")
    res.close()
    time.sleep(0.5)
