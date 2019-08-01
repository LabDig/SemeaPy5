#!/usr/bin/python3
# -*- coding: utf-8 -*-
import serial
import time
import re
#Le arquivo de dados
with open("M0_F0_original1.csv","r",encoding="latin-1") as f:
    dataset=f.read().splitlines()
f.close()


#Cria Arquivo de Saida
with open("M0_F0_remoto.txt","a",encoding="latin-1") as f:
    f.write('Teste\n')
    f.write('I,ID,Tempo Loop,Sleep,Date,Time,Signal,Status\n')
f.close()

#3G
sim800l= serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26


#Leitura
def Read():
    rec=sim800l.readline()
    return rec.decode('utf-8')

#Envia AT e processa resposta
def Send(cmd_at):
    global t_limite
    rec=''
    data=''
    sim800l.write(str.encode(cmd_at+'\r'))
    while not cmd_at in rec:rec=Read() #enquanto não le o echo
    if cmd_at in rec: #se recebeu o echo
        if 'AT+HTTPACTION' in rec: # Se for o Comando 'AT+HTTPACTION', deve esperar o servidor responder
            t=time.time()
            while not '+HTTPACTION:' in rec:
                rec=Read()
                t2=time.time()-t
                if '+HTTPACTION:' in rec and t2<t_limite: #se nao demorar
                    data=re.findall(r'\d+',rec)[1]
                    break
                elif t2>=t_limite: #se demorar pula
                    data='TIMEOUT'
                    break
        else: #Para os outros comandos, deve aguardar o OK
             while not 'OK' in rec:
                rec=Read()
                if 'ERROR' in rec:break #necessario pular para HTTPINT
                if '+SAPBR: 1,1' in rec:data=rec #para ler o ip
                if '+CSQ:' in rec: data=re.findall(r'\d+',rec)[0] #le intensidade sinal
    return data





for t_step in range (1,11): #varia o tempo entre 1 e 10 s

    print ('time step',t_step)
    Send('ATE1')
    Send('AT+SAPBR=3,1,\"Contype\",\"GPRS\"')
    Send('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"')
    Send('AT+SAPBR=1,1')
    ip=Send('AT+SAPBR=2,1')
    Send('AT+HTTPINIT')
    Send('AT+HTTPPARA=\"CID\",1')

    t_limite=t_step-0.5 #tempo limtite para aguardar o servidor responder
    with open("M0_F0_remoto.txt","a",encoding="latin-1") as f:
            f.write("TimeStep"+str(t_step)+"\n")
            f.write("IP"+ip+"\n")
    f.close()

    for i in range (1,3): #para se obter 200 dados enviados
        
        row=dataset[t_step*i].split(';') 
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
        signal=Send('AT+CSQ')
        #link='http://andrecoelho.tech/SemeaView/send_mysql.php?' #
        link='http://andrecoelho.tech/SemeaView/conectado.php'
        str_data='LogID='+str(log_id)+'&Date='+str(date)+'&Time='+str(ttime)+'&MachineID='+machineID\
    +'&FieldID='+fieldID+'&Lati='+str(lat)+'&Longi='+str(long)+'&XUtm='+str(lat_utm)+'&YUtm='+str(long_utm)+'&Speed='+\
    str(mach_speed)+'&OpCap='+str(opcap)+'&TimeOperation='+str(time_operation)+'&Population='+str(popseed)+'&FertRatio='+\
    str(fert_rt)+'&FertWgt='+str(fert_wgt)+'&Area='+str(area)
        
        Send('AT+HTTPPARA=\"URL\",'+link)
        status=Send('AT+HTTPACTION=0')
        print (status)

        t_loop=time.time()-st
        try:
            t_sleep=t_step-(time.time()-st)
            time.sleep(t_sleep) #programa fica esperando dar o time step, para padronizar o tempo
        except :pass
        
        #escreve no arquivo de resposta
        str_write=str(i)+','+log_id+','+str(round(t_loop,2))+','+str(round(time.time()-st,2))+','+date+','+ttime+','+signal+','+status+'\n'
        with open("M0_F0_remoto.txt","a",encoding="latin-1") as f:
            f.write(str_write)
        f.close()

    Send('AT+SAPBR=0,1')    
    print ('troca tempo')

print ('fim')
