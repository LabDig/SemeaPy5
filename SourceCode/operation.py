import math
import utm
import time
import numpy as np
import random as rd
#
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
#from Adafruit_BBIO.Encoder import RotaryEncoder,eQEP0 #0== Seed # 
#Encoder Velocidade Deslocamento
#Encoder Dosador Semente
#EncSeed=RotaryEncoder(eQEP0)
#EncSeed.enable()
#Encoder Whell
pinEncWhell="P8_11"
GPIO.setup(pinEncWhell, GPIO.IN)
atual_st_seed,last_st_seed,aux_i_seed,time_start_seed=-99,-99,0,0
atual_st_wheel,last_st_wheel,time_start_wheel,st_start_wheel,aux_i_wheel,time_reset_speed=-1,-1,0,False,0,0
real_rot_seed,real_rot_wheel=0.0,0.0
lat,long,lat_utm,long_utm,pdop,status=0,0,0,0,0,0
dt_corr=0
sum_time=0
st=False
avg_speed=0
seed_spped_array=[]
aux_reset,sum_wgt,value=0,0,0
start_t_seed=False
#
def Fert(v,rate,spacing):
    fertybym=rate*spacing/10000.0
    return round(fertybym,3),round(fertybym*v,3)
#
def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    return round(3.3*seeds*v/holes,3),round(seeds,1)
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
'''
def SeedSpeed(change_duty_cicle):
    global atual_st_seed,last_st_seed,aux_i_seed,real_rot_seed,\
           time_start_seed,sum_time,st,seed_spped_array
    atual_st_seed=abs(EncSeed.position)
    #
    #testar se o tempo 10 ms é suficiente
    #
    #if have up border
    if (last_st_seed==0 and atual_st_seed==1):
        if st:
            seed_spped_array=np.append(seed_spped_array,time.time()-time_start_seed)
            real_rot_seed=(1/20)/np.mean(seed_spped_array)
            print (round(real_rot_seed,3))
        time_start_seed=time.time()
        st=True #only after second border in start software
    
        
    if len(seed_spped_array) == 50: seed_spped_array = np.delete(seed_spped_array, 0)
    last_st_seed=atual_st_seed #update last status
 
    return round(real_rot_seed,2)
    
'''

    
    
def SeedSpeed(change_duty_cicle):
    global atual_st_seed,last_st_seed,aux_i_seed,real_rot_seed,time_start_seed,start_t_seed,seed_spped_array,avg_speed
    atual_st_seed=0#abs(EncSeed.position)
    #if have up border
    if (last_st_seed==0 and atual_st_seed==1):
        aux_i_seed=aux_i_seed+1
    #if one up border is detectec start the time
        
    last_st_seed=atual_st_seed #update last status
    if change_duty_cicle :
        seed_spped_array=[] #clear the array, for not smothing the speed variation
    seed_spped_array=np.append(seed_spped_array,real_rot_seed)
    # delete the first value of arry
    
    if len(seed_spped_array) == 50: seed_spped_array = np.delete(seed_spped_array, 0)

    if len(seed_spped_array)>0:
        avg_speed=np.mean(seed_spped_array)
    return round(avg_speed,3)
#
def WheelSpeed():
    global atual_st_wheel,last_st_wheel,time_start_wheel,st_start_wheel,aux_i_wheel,real_rot_wheel,time_reset_speed
    atual_st_wheel=GPIO.input(pinEncWhell)
    #if have up border
    if (last_st_wheel==0 and atual_st_wheel==1):
        aux_i_wheel=aux_i_wheel+1
        time_reset_speed=time.time()
    #if one up border is detectec start the time
    if (aux_i_wheel==1 and st_start_wheel is False):
        time_start_wheel=time.time()
        st_start_wheel=True
    #at complete 20 up border, calculate the velocity (one revolution is 20 up border)
    if (aux_i_wheel==10):
        real_rot_wheel= (1)/(time.time()-time_start_wheel)
        aux_i_wheel=0
        st_start_wheel=False
    last_st_wheel=atual_st_wheel #update last statu
    #set speed to 0, if the machine stop
    if (time.time()-time_reset_speed>0.4): real_rot_wheel=0.0 #if speed< 0.25 m/s ==> speed=0
    return round(real_rot_wheel,3)
#
def ReadWeight(pin,cal_a,cal_b):
    
    global aux_reset,sum_wgt,value
    aux_reset=aux_reset+1
    sum_wgt=sum_wgt+1.8*ADC.read(pin)
    if aux_reset==100:
        value=(sum_wgt/100.0)*cal_a+cal_b
        aux_reset=0
        sum_wgt=0
    return round(value,3)
#
def ControlSpeedSeed(st,pinEnable_Seed,pinPWM_Seed,calc_rot,real_rot,a,b):
    global dt_corr
    kp=5.0
    if (calc_rot-real_rot)>0.01 and real_rot!=0.0:dt_corr=dt_corr+kp*(calc_rot-real_rot)
    elif (calc_rot-real_rot)<-0.01 and  real_rot!=0.0:dt_corr=dt_corr+kp*(calc_rot-real_rot)
    else: dt_corr=dt_corr
    if st: dt_corr=0
    dt_seed=(a*calc_rot+b)+dt_corr
    if dt_seed>100.0 :
        dt_seed=100.0
    if dt_seed<30.0 :
        dt_seed=0.0 #because the motor dont work in low speed
    PWM.set_duty_cycle(pinPWM_Seed,dt_seed)
    GPIO.output(pinEnable_Seed,GPIO.HIGH)
    return dt_seed
     
def ControlSpeedFert(pinEnable_Fert,pinPWM_Fert,fertbys,wgt,last_wgt,v,t,change_rt):

    m_exit_cal=fertbys*t  #the ideal is 5 s or 5 m
    m_exit_real=wgt-last_wgt  #fertilizer mass the real exit

    if change_rt is False: #only for fert_rt constant
        varm= m_exit_cal-m_exit_real #based in this, ajust the dt
    else: varm=0
    
    dt=(1300*fertbys) # Experimental Calibration Equation
    if dt>100.0 : dt=100.0
    if dt<0.0: dt=0.0
    PWM.set_duty_cycle(pinPWM_Fert,dt)
    if dt>10: GPIO.output(pinEnable_Fert,GPIO.HIGH) #motor dont'work in low speed
    else:GPIO.output(pinEnable_Fert,GPIO.LOW)

def Encder():
    print ("up")
