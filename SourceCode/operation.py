#Auxilar script file to SemeaPy5.
#Containing functions used for developed controller for seed variable rate aplication
import math
import utm # to install python3 -m pip install utm
import time
import numpy as np
#Import Adrafruit Module for use GPIO and PWM Pins of Beablebone Black 
#This software was developed and Test using the oficialLinux Debian 8.6 for Beablebone Black
#Dowloaded in https://beagleboard.org/latest-images
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
#Declarate pins of BeagleBone Black
#PWM and Enable send signal to BTS 7960 controller motor
pinPWM_Seed="P8_13" #
pinEnable_Seed="P8_10"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty cycle and frequency
#
#
#Calc Sowing Ratio (number of seed in one meter) and Angular Velocity of Eletric Motor (in rad/s). The seed metering device work only in planter speed greater than 0.3 m/s
# v is planter speed (m/s)
# pop is the plant density (plants/ha)
# holes is number of holes in perforeted disk
# germ it is the seed germination percentage
#3.3 it is the speed ration of gears used in seed metering devices
#2.0 * pi it is used to angular velocity in rad/s
def Seeder(v,pop,row,holes,germ):
    seeds=pop*row/(10000*germ/100)
    if v>0.3 : return round(2.0*math.pi*3.3*seeds*v/holes,2),round(seeds,1)
    else : return 0.0,round(seeds,1)
#
#Check if the actual planter position (determined by UBLOX NEO 6-M module) it's inside of a management zone
# x and y is the planter position in utm coordinante (m)
# poly it is the a array with management zone vertices
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
#
# Find if the actual planter postion (x_actual and y_actual) it is inside a management zones
#poly_map have arrays. Each array contain the vertices of a management zones.
#value it is a array with atributes contained in shapefile
# atribute it is the plant density for each management zone
def FindZone(x_atual,y_atual,poly_map,values):
    for i in range(len(poly_map)): # for each management zone
			#check if the actual point it is inside of management zone i
            if(ray_tracing(x_atual,y_atual,poly_map[i])==True):
                atribute=values[i] # the plant seed of actual management zone it is defined 
                break 
            else : atribute =0 # if the planter it is outside of all management zones. Plnat density it is 0
    return atribute
# Check if NMEA 1 AND 2 it is GPRMC or GPGSA
# The other NMEA it is not send by UBLOX NEO 6M
# pdop it is the PDOP
# Date and ttime it is the date and UTC time informed by GNSS
# lat_utm and long_utm it is utm coordinate, obtained by convert Latitude and Longitude
# status it is the status of ubloxNEO-6M - Sattelite conect. Active or Void. 
def ReadGPS(nmea1,nmea2):
    if '$GPGSA' in nmea1[0]: pdop=float(nmea1[-3])
    if '$GPGSA' in nmea2[0]: pdop=float(nmea2[-3])
    if '$GPRMC' in nmea1[0]: date,ttime,lat_utm,long_utm,lat,long,status=SplitNMEA(nmea1)
    if '$GPRMC' in nmea2[0]: date,ttime,lat_utm,long_utm,lat,long,status=SplitNMEA(nmea2)
    return date,ttime,lat_utm,long_utm,lat,long,status,pdop
#
# Split the GPRMC NMEA SENTENCES  send by UBLOX NEO 6M GNSS module
def SplitNMEA(nmea_array):
        size=len(nmea_array) # size of array
        status=nmea_array[2]  # check if conection with sattelites
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
#
#
# Calculate duty cycle of PWM signal to Control the angular velocity of electric motor
# Calc_rot it is the calculated angular velocity
# a and b it is the parameter of linear model (dc=b+a∙ω_c)
def ControlSpeedSeed(calc_rot,a,b):
	#
    dt_seed=(a*calc_rot+b)
    if dt_seed>100.0 :dt_seed=100.0
    if dt_seed<30.0 :dt_seed=0.0#the minimum duty cycle it is 30%
    PWM.set_duty_cycle(pinPWM_Seed,dt_seed)
    GPIO.output(pinEnable_Seed,GPIO.HIGH)
    return dt_seed 
