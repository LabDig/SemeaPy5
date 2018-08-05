import math
import utm
import time
from sklearn import linear_model # pip install sklearn  apt-get install python3-scipy
import numpy as np

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
    x,y,z=[]
    for i in range(1,len(content)-1): #remove header
            Row=content[i].split(',')
            x.append(float(Row[0])) 
            y.append(float(Row[1]))
            z.append(float(Row[2]))
    return x,y,z

def GPSRead(nmea):
    try:
        nmea=nmea.decode("utf-8")
        nmea_array=nmea.split(',')
        size=len(nmea_array)
        if nmea_array[0]=='$GPRMC':
            status=nmea_array[2]  # check status
            if self.status=='A' and size==13:
                latMin=float(nmea_array[3][2:])/60   #
                lat=((float(nmea_array[3][0:2])+latMin)) #
                lonMin=float(nmea_array[5][3:])/60   # 
                lon=((float(nmea_array[5][0:3])+lonMin)) #
                latHem=nmea_array[4]  # N or S
                lonHem=nmea_array[6]  # W or E
                if lonHem=='W': lon=-lon
                if latHem=='S': lat=-lat
                utm_conv=utm.from_latlon(lat,lon)
                lat_atual=utm_conv[0]
                long_atual=utm_conv[1]
        if nmea_array[0]=='$GPGSA'and status=='A' and size==18:
            pdop=nmea_array[-3] # pdop
        if status=='V':
            pdop=999.99
    except:
        pass
    return lat,long,pdop,status

def Speed_Seed(atual_st_seed):
       
        if (last_st_seed==0 and atual_st_seed==1): #if have up border
            aux_i_seed=saux_i_seed+1
        if (aux_i_seed==1): #if one up border is detectec start the time
             time_start_seed=time.time()
        if (aux_i_seed==3): #at complete three up border, calculate the velocity (one revolution is 15 up border)
            real_rot=60*.2/(time.time()-ts)
            aux_i_seed=0
        if (last_st_seed==atual_st_seed): #for dectect if encoder its stop, 
            aux_j_seed=aux_j_seed+1
        if (aux_j_seed>100): #if encoder is stop much time, v=0
            aux_j_seed=0
            real_rot=0
        last_st_seed=atual_st_seed #update last status
        return real_rot

def ReadWeight(value):
    
    #Calibration
    massa = np.array([ 0,3000,4200,6900,8600,10800,13100,11200,8900,7200,5000,2300,0 ])
    voltage = np.array([ 0.08,0.23,0.29,0.46,0.55,0.73,0.86,0.75,0.60,0.47,0.34,0.18,0.08])
    voltage=voltage.reshape(1,-1)
    model = linear_model.LinearRegression()
    model.fit(voltage.T, massa)
    max_samples = 50  # window for movel average filter
    value_array=[]

    ti=time.time()
    value=ADC.read("P9_33")

    value_array = np.append(value_array, value)

    avg_value = np.mean(value_array)

    if len(value_array) == max_samples:
        value_array = np.delete(value_array, 0) # delete the first value


    massa=avg_value*model.coef_[0] + model.intercept_ #massa (g) =x (v) * a +b
    return massa
