import math
import utm
import time
import numpy as np
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
#pins
pinLoadCell="P9_33"
pinPWM_Seed="P8_13"
pinEnable_Seed="P8_10"
pinPWM_Fert="P8_19"
pinEnable_Fert="P8_9"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
PWM.start(pinPWM_Fert,0, 1000.0) #pin, duty,frequencia

# Calc Fert Ratio
def Fert(v,rate,spacing):
    fertybym=rate*spacing/10000.0
    return round(fertybym,3),round(fertybym*v,3)

#Calc Seed Ratio. Work only for spped >0.5 m/s
def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    if v>0.3 : return round(3.3*seeds*v/holes,2),round(seeds,1)
    else : return 0.0,round(seeds,1)

#Check if a point it is inside or outside to polygon
def ray_tracing(x,y,poly):
    n = len(poly)
    inside = False
    p2x = 0.0
    p2y = 0.0
    xints = 0.0
    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside

# Find the neart point in the map (for fert and seed)



def FindNeig(x_atual,y_atual,poly_map,values):

    for i in range(len(poly_map)):
            if(ray_tracing(x_atual,y_atual,poly_map[i])==True):#valor do atributo #if point inside any polygon
                atribute=values[i]
                break 
            else : atribute =0
    return atribute

# Split the gprm nmea sente

def ReadGPS(nmea1,nmea2):
    if '$GPGSA' in nmea1[0]: pdop=float(nmea1[-3])
    if '$GPGSA' in nmea2[0]: pdop=float(nmea2[-3])
    if '$GPRMC' in nmea1[0]: date,ttime,lat_utm,long_utm,lat,long,status=SplitNMEA(nmea1)
    if '$GPRMC' in nmea2[0]: date,ttime,lat_utm,long_utm,lat,long,status=SplitNMEA(nmea2)
    return date,ttime,lat_utm,long_utm,lat,long,status,pdop

def SplitNMEA(nmea_array):
        size=len(nmea_array)
        status=nmea_array[2]  # check status
        if status=='A' and size==13:
            date=nmea_array[9][0]+nmea_array[9][1]+'/'+nmea_array[9][2]+nmea_array[9][3]+'/'+nmea_array[9][4]+nmea_array[9][5]
            ttime=nmea_array[1][0]+nmea_array[1][1]+':'+nmea_array[1][2]+nmea_array[1][3]+':'+nmea_array[1][4]+nmea_array[1][5]
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
        if status=='V':  lat,long,lat_utm,long_utm,date,ttime=0,0,0,0,'',''
        return date,ttime,lat_utm,long_utm,lat,long,status


# Read the weigth of fert tank
mass_filter,mass=[],0 #global variables
def ReadWeight(cal_a,cal_b):
        
    global mass_filter,mass
    mass_filter=np.append(mass_filter,cal_a*(1.8*ADC.read(pinLoadCell))+cal_b)
    if len(mass_filter)==10:mass_filter=np.delete(mass_filter,0)
    if len(mass_filter)>0: mass=round(np.mean(mass_filter),1)
    return mass

# Control the seed speed

dt_corr=0 #global variable
def ControlSpeedSeed(st,calc_rot,real_rot,a,b):
    global dt_corr
    kp=1.5
    if (calc_rot-real_rot)>0.02 and real_rot!=0.0:
        dt_corr=dt_corr+kp*(calc_rot-real_rot)
    elif (calc_rot-real_rot)<-0.02 and  real_rot!=0.0:
        dt_corr=dt_corr+kp*(calc_rot-real_rot)
    else: dt_corr=dt_corr
    if st is True or calc_rot<0.05: #reset dt_cor
        dt_corr=0
    dt_seed=(a*calc_rot+b)+dt_corr
    if dt_seed>100.0 :dt_seed=100.0
    if dt_seed<40.0 :dt_seed=0.0#because the motor dont work in low speed
    PWM.set_duty_cycle(pinPWM_Seed,dt_seed)
    GPIO.output(pinEnable_Seed,GPIO.HIGH)
    return dt_seed

# Control the seed speed     
def ControlSpeedFert(a,b,fertbys):
    dt=a*fertbys+b # Experimental Calibration Equation
    if dt>100.0 : dt=100.0
    if dt<40.0: dt=0.0 #motor dont'work in low speed
    PWM.set_duty_cycle(pinPWM_Fert,dt)
    GPIO.output(pinEnable_Fert,GPIO.HIGH)
    return dt
    
    
