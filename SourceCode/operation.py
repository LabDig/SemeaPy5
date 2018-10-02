import math
import utm
import time
from sklearn import linear_model # pip install sklearn  apt-get install python3-scipy
import numpy as np

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
from Adafruit_BBIO.Encoder import RotaryEncoder,eQEP0,eQEP2 # 0 == Seed # 2 == Roda 


#Encoder Velocidade Deslocamento
#use set_freq.py in Python2 to set frequency
EncRoda=RotaryEncoder(eQEP2)
EncRoda.enable()
#Encoder Dosador Semente
EncSeed=RotaryEncoder(eQEP0)
EncSeed.enable()


atual_st_seed,last_st_seed,aux_i_seed,time_start_seedm=-99,-99,0,0
atual_st_wheel,last_st_wheel,time_start_wheel=-99,-99,0
real_rot_seed,real_rot_wheel=0,0
lat, long,pdop,status=0,0,0,0
seed_cor=0
weight_array=[]

def Fert(v,rate,spacing):
    fertbym=rate*spacing/10000.0
    factor=0.026  #kg of fertilizer in one revolution of screw
    return round(60*fertbym*v/factor,1),round(fertbym,1)

def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    return round(60*3.3*seeds*v/holes,1),round(seeds,1)

# Find the Neart point in the map
def FindNeig(x_atual,y_atual,x_map,y_map,pop_map):
    minDist=9999999
    k=len(x_map)
    for i in range(k):
        deltaX=math.fabs(x_atual-x_map[i])
        deltaY=math.fabs(y_atual-y_map[i])
        distance=math.sqrt(deltaX*deltaX+deltaY*deltaY)
        if distance<minDist:
            minDist=distance
            idminDist=i
            
    return pop_map[idminDist],x_map[idminDist],y_map[idminDist]

def ReadMapFile(data):
    x,y,z=[],[],[]
    for i in range(1,len(data)-1): #remove header
            Row=data[i].split(',')
            x.append(float(Row[0])) 
            y.append(float(Row[1]))
            z.append(float(Row[2]))
    return x,y,z

def ReadGPS(nmea):
    global lat,long,pdop,status
    try:
        nmea=nmea.decode("utf-8")
        nmea_array=nmea.split(',')
        size=len(nmea_array)
        if nmea_array[0]=='$GPRMC':
            status=nmea_array[2]  # check status
            if status=='A' and size==13:
                latMin=float(nmea_array[3][2:])/60   #
                lat=((float(nmea_array[3][0:2])+latMin)) #
                lonMin=float(nmea_array[5][3:])/60   # 
                lon=((float(nmea_array[5][0:3])+lonMin)) #
                latHem=nmea_array[4]  # N or S
                lonHem=nmea_array[6]  # W or E
                if lonHem=='W': lon=-lon
                if latHem=='S': lat=-lat
                utm_conv=utm.from_latlon(lat,lon)
                lat=float(utm_conv[0])
                long=float(utm_conv[1])
        if nmea_array[0]=='$GPGSA'and status=='A' and size==18:
            pdop=float(nmea_array[-3]) # pdop
        if status=='V':
            pdop=999.99
    except:
        pass

    return lat,long,pdop,status

def SeedSpeed():
    global atual_st_seed,last_st_seed,aux_i_seed,real_rot_seed,time_start_seed
    
    atual_st_seed=EncSeed.position
    
    #if have up border
    if (last_st_seed==0 and atual_st_seed==-1): aux_i_seed=aux_i_seed+1
    #if one up border is detectec start the time
    if (aux_i_seed==1): time_start_seed=time.time()
    #at complete 20 up border, calculate the velocity (one revolution is 20 up border)
    if (aux_i_seed==20):
        real_rot_seed= (1)/(time.time()-time_start_seed)
        aux_i_seed=0
    last_st_seed=atual_st_seed #update last status

    return round(real_rot_seed,1)

def WheelSpeed():
    global atual_st_wheel,last_st_wheel,real_rot_wheel,time_start_wheel

    atual_st_wheel=EncRoda.position
    
    if (atual_st_wheel==2): time_start_wheel=time.time()

    if (atual_st_wheel==60): 
        real_rot_wheel= (0.5)/(time.time()-time_start_wheel)
        Encroda.zero()

    #for dectect if encoder its stop, 
    if (time.time()-time_start_wheel > 0.5): real_rot_wheel=0
    last_st_wheel=atual_st_wheel #update last status

    return round(real_rot_wheel,1)

def ReadWeight(pin):
    global weight_array

    #Calibration
    massa = np.array([ 0,3000,4200,6900,8600,10800,13100,11200,8900,7200,5000,2300,0 ])
    voltage = np.array([ 0.08,0.23,0.29,0.46,0.55,0.73,0.86,0.75,0.60,0.47,0.34,0.18,0.08])
    voltage=voltage.reshape(1,-1)
    model = linear_model.LinearRegression()
    model.fit(voltage.T, massa)
    value=ADC.read(pin)
    weight_array=np.append(weight_array,value)
    avg_value = np.mean(weight_array)

    # delete the first value of arry
    if len(weight_array) == 50: weight_array = np.delete(weight_array, 0) 

    massa=avg_value*model.coef_[0] + model.intercept_ #massa (g) =x (v) * a +b

    return round(massa,1)


def ControlSpeedSeed(pinEnable_Seed,pinPWM_Seed,rot_seed,real_rot_seed):
    global seed_cor

    dt=(1.6535*rot_seed+9.5872)+seed_cor
    if dt>100.0 : dt=100.0
    if dt<25 : dt=25.0
    PWM.set_duty_cycle(pinPWM_Seed,dt)
    GPIO.output(pinEnable_Seed,GPIO.HIGH)
    seed_cor=(rot_seed-real_rot_seed)/10 #seed_cor+

    
def ControlSpeedFert(pinEnable_Fert,pinPWM_Fert,rot_fert):

    PWM.set_duty_cycle(pinPWM_Fert,60)
    GPIO.output(pinEnable_Fert,GPIO.HIGH)
    
