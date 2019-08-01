import requests
#import matplotlib.pyplot as plt
#import numpy as np
import time
import os

direc=os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(direc,"M0_F0_original1.csv"),"r",encoding="latin-1") as f:
    dataset=f.read().splitlines()
f.close()

print ('inicio')   
for i in range (1,1000): #para se obter 250 dados enviados
                
                row=dataset[i].split(';') 
                log_id=row[0]
                date=row[2]
                ttime=row[3]
                machineID=row[4]
                fieldID=row[5]
                lat=row[8]
                long=row[9]
                lat_utm=row[6]
                long_utm=row[7]
                mach_speed=row[10]
                popseed=row[12]
                fert_rt=row[13]
                fert_wgt=row[14]
                opcap=row[15]
                time_operation=row[16]
                area=row[17]
                
                st=time.time()
                link='http://andrecoelho.tech/SemeaView/send_mysql.php?' #http://andrecoelho.tech/SemeaView/conectado.php
                str_data='LogID='+str(log_id)+'&Date='+str(date)+'&Time='+str(ttime)+'&MachineID='+machineID\
            +'&FieldID='+fieldID+'&Lati='+str(lat)+'&Longi='+str(long)+'&XUtm='+str(lat_utm)+'&YUtm='+str(long_utm)+'&Speed='+\
            str(mach_speed)+'&OpCap='+str(opcap)+'&TimeOperation='+str(time_operation)+'&Population='+str(popseed)+'&FertRatio='+\
            str(fert_rt)+'&FertWgt='+str(fert_wgt)+'&Area='+str(area)
                
                r=requests.get(link+str_data)
                t=time.time()-st
                #escreve no arquivo de resposta
                str_write=str(i)+','+str(round(t,2))+','+str(r.status_code)+'\n'
                with open('request_rj45.txt',"a",encoding="latin-1") as f:
                    f.write(str_write)
                f.close()
                print (i,t,r.status_code)

print ('fim')
