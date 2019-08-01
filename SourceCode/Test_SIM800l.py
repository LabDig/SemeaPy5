#Teste SIM800L - Ensaio
#
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import serial
import time
import re

#3G
sim800l = serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26

#Funcao
def Read():
    rec=sim800l.readline()
    rec=rec.decode('utf-8')
    return rec

'''
    if '+SAPBR: 1,1' in rec:
        return rec,rec
    if '+CSQ:' in rec:
        return rec,int(re.findall(r'\d+',rec)[0])
    if '+HTTPACTION:' in rec:
        return rec,int(re.findall(r'\d+',rec)[1])
    else :
        return rec,''
'''
def Send(cmd_at):
    rec=''
    sim800l.write(str.encode(cmd_at+'\r'))
    while not 'OK' in rec: #espera responder ao comando
        rec=Read()
        if 'ERROR' in rec:break #necessario pular
        if '+SAPBR: 1,1' in rec:
            print ('sa',rec)
            return rec
        if '+CSQ:' in rec:
            print ('cs',rec)
            return int(re.findall(r'\d+',rec)[0])
        if '+HTTPACTION:' in rec:
            print ('hp',rec)
            return int(re.findall(r'\d+',rec)[1])

    

#print ("Start...")
#configure
Send('ATE0')
Send('AT+SAPBR=3,1,\"Contype\",\"GPRS\"')
Send('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"')
ip=Send('AT+SAPBR=1,1')
print ('ip',ip)
Send('AT+SAPBR=2,1')
Send('AT+HTTPINIT')
#print ('configurado')
Send('AT+HTTPPARA=\"CID\",1')
#time.sleep(10)
i=0
while i<3:
    st=time.time()
    signal=Send('AT+CSQ')
    print ('s',signal)
    link='http://andrecoelho.tech/SemeaView/send_mysql.php?' #http://andrecoelho.tech/SemeaView/conectado.php
    str_data='LogID='+str(i)
    Send('AT+HTTPPARA=\"URL\",'+link+str_data)
    #print ('Enviado')
    time.sleep(10)
    st=Send('AT+HTTPACTION=0')
    print ('st',st)
    time.sleep(10)
    #Send('AT+HTTPREAD')
    #print (round(time.time()-st,2),signal,status)
    i=i+1
    
Send('AT+SAPBR=0,1')
print ('fim')
    





'''
i=0
while i<10:
    Send('AT+CSQ')
    i=i+1
'''
#Send('AT+SAPBR=0,1')
#print ('fim')



'''


sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
time.sleep(t_config)
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))           
time.sleep(t_config)

#read until configure 3G
st_start=False
rec=''
print ("Aguardando Configurar 3G...")
while not '+SAPBR:1,1' in rec and st_start is False:
        print ('entrei',rec,st_start)
        rec=sim800l.readline()
        rec=rec.decode('utf-8')
        st_start=True
        print (rec)

print ('Pronto')

i=0
while i<100:
    rec=sim800l.readline()
    rec=rec.decode('utf-8')
    i=i+1
    sim800l.write(str.encode('AT+'+'CSQ\r'))
    time.sleep(1)
    if "+CSQ:" in rec:
        print (rec)
    print (rec)
    time.sleep(1)
    


# 3G Send Data
dt=int(2000/self.time_control) #  5*dt/time_control must  10 s
if self.aux==dt and  "ON" in self.st_remote:
    sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
if self.aux==4*dt and  "ON" in self.st_remote:
    sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
    self.ql_remote_status.setPlainText("Starting..")
if self.aux==6*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
if self.aux==8*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
if self.aux==10*dt and  "ON" in self.st_remote:
    sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
    self.ql_remote_status.setPlainText("HTTP INIT")
if self.aux==12*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
link='http://andrecoelho.tech/SemeaView/send_mysql.php?'
str_data='LogID='+str(self.log_id)+'&Date='+str(self.date)+'&Time='+str(self.time)+'&MachineID='+self.machineID\
+'&FieldID='+self.fieldID+'&Lati='+str(self.lat)+'&Longi='+str(self.long)+'&XUtm='+str(self.lat_utm)+'&YUtm='+str(self.long_utm)+'&Speed='+\
str(self.mach_speed)+'&OpCap='+str(self.opcap)+'&TimeOperation='+str(self.time_operation)+'&Population='+str(self.popseed)+'&FertRatio='+\
str(self.fert_rt)+'&FertWgt='+str(self.fert_wgt)+'&Area='+str(self.area)
if self.aux==14*dt and  "ON" in self.st_remote and self.start_remote==QMessageBox.Yes:
    sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+str_data+'\r'))
if self.aux==17*dt and  "ON" in self.st_remote and self.start_remote==QMessageBox.Yes:
    sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
    self.aux=12*dt
rec=sim800l.readline()
rec=rec.decode('utf-8')
if '+SAPBR:' in rec: self.start_remote=QMessageBox.question(self,'3G',rec)
if '+HTTPACTION:' in rec:
    if "200" in rec :
        self.error=0
        self.rm_st='ON'
        self.ql_remote_status.setPlainText("Sucess")
    if not "200" in rec :
        self.error=self.error+1
        self.rm_st='ERROR'
        self.ql_remote_status.setPlainText("Error")
if "OFF" in self.st_remote:
        self.rm_st='OFF'
        self.aux=0
        self.ql_remote_status.setPlainText("Disable")
        sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
if self.error==5:
        self.error=0
        self.rm_st="RESET"
        sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
        self.aux=0 #reset 3G
        QMessageBox.information(self,'3G','Error')
self.aux=self.aux+1
'''
   

