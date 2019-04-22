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
massa = np.array([ 0,3000,4200,6900,8600,10800,13100,11200,8900,7200,5000,2300,0 ])
voltage = np.array([ 0.08,0.23,0.29,0.46,0.55,0.73,0.86,0.75,0.60,0.47,0.34,0.18,0.08])

voltage=voltage.reshape(1,-1)
model = linear_model.LinearRegression()
model.fit(voltage.T, massa)


max_samples = 50  # window for movel average filter
value_array=np.array(ADC.read("P9_33")) #first value
file="re2s.txt"
res=open(file,"a")
res.write("Cabecalho")
res.write("\n")
res.close()

ti=time.time()
while True:
    value=ADC.read("P9_33")

    value_array = np.append(value_array, value)

    avg_value = np.mean(value_array)
    print (avg_value)

    if len(value_array) == max_samples:
        value_array = np.delete(value_array, 0) # delete the first value


    massa=avg_value*model.coef_[0] + model.intercept_ #massa (g) =x (v) * a +b
    print (massa-450)
    res=open(file,"a")
    res.write(str(time.time()-ti))
    res.write(",")
    res.write(str(value))
    res.write(",")
    res.write(str(avg_value))
    res.write("\n")
    res.close()
    time.sleep(1)
