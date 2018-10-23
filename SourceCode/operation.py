import math
import utm
import time
import numpy as np
#
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
#
atual_st_seed,last_st_seed,aux_i_seed,time_start_seedm=-99,-99,0,0
atual_st_wheel,last_st_wheel,time_start_wheel=-99,-99,0
real_rot_seed,real_rot_wheel=0,0
lat,long,lat_utm,long_utm,pdop,status=0,0,0,0,0,0
seed_cor=0
weight_array=[]
#
def Fert(v,rate,spacing):
    fertybym=rate*spacing/10000.0
    return round(fertybym,3),round(fertybym*v,3)
#
def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    return round(3.3*seeds*v/holes,2),round(seeds,1)
#
# Find the neart point in the map
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
#
def ReadMapFile(data):
    x,y,z=[],[],[]
    for i in range(1,len(data)-1):
            Row=data[i].split(',')
            x.append(float(Row[0])) 
            y.append(float(Row[1]))
            z.append(float(Row[2]))
    return x,y,z
#
def ReadGPS(nmea):
    global lat,long,lat_utm,long_utm,pdop,status
    try:
        nmea=nmea.decode("utf-8")
        nmea_array=nmea.split(',')
        size=len(nmea_array)
        if nmea_array[0]=='$GPRMC':
            status=nmea_array[2]  # check status
            if status=='A' and size==13:
                latMin=float(nmea_array[3][2:])/60   
                lat=((float(nmea_array[3][0:2])+latMin)) 
                lonMin=float(nmea_array[5][3:])/60   
                long=((float(nmea_array[5][0:3])+lonMin)) 
                latHem=nmea_array[4]  # N or S
                lonHem=nmea_array[6]  # W or E
                if lonHem=='W': long=-long
                if latHem=='S': lat=-lat
                utm_conv=utm.from_latlon(lat,long)
                lat_utm=float(utm_conv[0])
                long_utm=float(utm_conv[1])
        if nmea_array[0]=='$GPGSA'and status=='A' and size==18:
            pdop=float(nmea_array[-3])
        if status=='V':
            pdop=999.99
            lat,long,lat_utm,long_utm=0,0,0,0
    except: pass
    return lat_utm,long_utm,lat,long,pdop,status
#
def SeedSpeed():
    global atual_st_seed,last_st_seed,aux_i_seed,real_rot_seed,time_start_seed
    atual_st_seed=EncSeed.position
    print(atual_st_seed)
    #if have up border
    if (last_st_seed==0 and atual_st_seed==-1): aux_i_seed=aux_i_seed+1
    #if one up border is detectec start the time
    if (aux_i_seed==1): time_start_seed=time.time()
    #at complete 20 up border, calculate the velocity (one revolution is 20 up border)
    if (aux_i_seed==20):
        real_rot_seed= (1)/(time.time()-time_start_seed)
        aux_i_seed=0
        
    last_st_seed=atual_st_seed #update last status
    return round(real_rot_seed,2)
#
def WheelSpeed():
    global atual_st_wheel,last_st_wheel,real_rot_wheel,time_start_wheel
    atual_st_wheel=EncRoda.position
    if (atual_st_wheel<0): time_start_wheel=time.time()
    if (atual_st_wheel<-60):
        real_rot_wheel= (2.0)/(time.time()-time_start_wheel)
        EncRoda.zero()
    #for dectect if encoder its stop, 
    if (time.time()-time_start_wheel > 0.5): real_rot_wheel=0
    last_st_wheel=atual_st_wheel #update last status
    return round(real_rot_wheel,2)
#
def ReadWeight(pin,cal_a,cal_b):
    global weight_array
    value=1.8*ADC.read(pin)
    weight_array=np.append(weight_array,value)
    avg_value = np.mean(weight_array)
    # delete the first value of arry
    if len(weight_array) == 50: weight_array = np.delete(weight_array, 0) 
    return round(avg_value*cal_a + cal_b ,3)
#
def ControlSpeedSeed(pinEnable_Seed,pinPWM_Seed,rot_seed,real_rot_seed):
    global seed_cor
    dt=100*rot_seed#+seed_cor #experimental calibration Equation
    if dt>100.0 : dt=100.0
    if dt<0 : dt=0
    PWM.set_duty_cycle(pinPWM_Seed,dt)
    seed_cor=(rot_seed-real_rot_seed)/10 #seed_cor+
    if dt>20: GPIO.output(pinEnable_Seed,GPIO.HIGH)  #motor dont'work in low speed
    else:GPIO.output(pinEnable_Seed,GPIO.LOW)
#    
def ControlSpeedFert(pinEnable_Fert,pinPWM_Fert,fertbys):
    dt=(1300*fertbys) # Experimental Calibration Equation
    if dt>100.0 : dt=100.0
    if dt<0 : dt=0
    PWM.set_duty_cycle(pinPWM_Fert,dt)
    if dt>10: GPIO.output(pinEnable_Fert,GPIO.HIGH) #motor dont'work in low speed
    else:GPIO.output(pinEnable_Fert,GPIO.LOW)
