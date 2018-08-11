import math
import utm
import time
from sklearn import linear_model # pip install sklearn  apt-get install python3-scipy
import numpy as np
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
atual_st_seed,last_st_seed,aux_i_seed,time_start_seedm=-1,-1,0,0
atual_st_wheel,last_st_wheel,aux_i_wheel,aux_j_wheel,time_start_wheel=-1,-1,0,0,0
real_rot_seed,real_rot_wheel=0,0
lat, long,pdop,status=0,0,0,0
weight_array=[]

def Fert(v,rate,spacing):
    fertbym=rate*spacing/10000.0
    factor=0.026  #kg of fertilizer in one revolution of screw
    return 60*fertbym*v/factor,fertbym

def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    return 60*3.3*seeds*v/holes,seeds

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

def SeedSpeed(pin):
        global atual_st_seed,last_st_seed,aux_i_seed,real_rot_seed,time_start_seed
        
        #file=open("/sys/class/gpio/gpio68/value","r")
        if GPIO.input(pin):
        #if file.read()=="1\n":
            atual_st_seed=1
        else:
            atual_st_seed=0
        #file.close()
        if (last_st_seed==0 and atual_st_seed==1): #if have up border
            aux_i_seed=aux_i_seed+1
        if (aux_i_seed==1): #if one up border is detectec start the time
             time_start_seed=time.time()
        if (aux_i_seed==20): #at complete  up border, calculate the velocity (one revolution is 20 up border)
            real_rot_seed=(60)/(time.time()-time_start_seed)
            aux_i_seed=0
        last_st_seed=atual_st_seed #update last status
        return real_rot_seed

def WheelSpeed(pin):
        global atual_st_wheel,last_st_wheel,aux_i_wheel,aux_j_wheel,real_rot_wheel,time_start_wheel
        #file=open("/sys/class/gpio/gpio66/value","r")
        #if file.read()=="1\n":
        if GPIO.input(pin):
            atual_st_wheel=1
        else:
            atual_st_wheel=0
        #file.close()

        if (last_st_wheel==0 and atual_st_wheel==1): #if have up border
            aux_i_wheel=aux_i_wheel+1
        if (aux_i_wheel==1): #if one up border is detectec start the time
             time_start_wheel=time.time()
        if (aux_i_wheel==5): #at complete three up border, calculate the velocity (one revolution is 20 up border)
            real_rot_wheel=0.5/(time.time()-time_start_wheel)
            aux_i_wheel=0
        if (last_st_wheel==atual_st_wheel): #for dectect if encoder its stop, 
            aux_j_wheel=aux_j_wheel+1
        if (aux_j_wheel>200): #if encoder is stop much time, v=0
            aux_j_wheel=0
            real_rot_wheel=0
        last_st_wheel=atual_st_wheel #update last status
        return real_rot_wheel

def ReadWeight(pin):
    global weight_array
    #Calibration
    massa = np.array([ 0,3000,4200,6900,8600,10800,13100,11200,8900,7200,5000,2300,0 ])
    voltage = np.array([ 0.08,0.23,0.29,0.46,0.55,0.73,0.86,0.75,0.60,0.47,0.34,0.18,0.08])
    voltage=voltage.reshape(1,-1)
    model = linear_model.LinearRegression()
    model.fit(voltage.T, massa)
 

    ti=time.time()
    #file=open("/sys/bus/iio/devices/iio:device0/in_voltage4_raw","r") #AIN4 P9.33
    #value=int(file.readline())
    #file.close()
    value=ADC.read(pin)
    
    weight_array = np.append(weight_array, value)

    avg_value = np.mean(weight_array)

    if len(weight_array) == 50:
        weight_array = np.delete(weight_array, 0) # delete the first value


    massa=avg_value*model.coef_[0] + model.intercept_ #massa (g) =x (v) * a +b
    return massa
