#Teste SIM800L - Ensaio
#
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import serial
import time
import re


#3G
sim800l = serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26

#Leitura
def Read():
    rec=sim800l.readline()
    return rec.decode('utf-8')

#Envia AT e processa resposta
def Send(cmd_at):
    print ('recebi cmd_at',cmd_at)
    rec=''
    data=''
    sim800l.write(str.encode(cmd_at+'\r'))
    while not cmd_at in rec:
        print ('aguarda eco para',cmd_at)
        rec=Read() #enquanto não le o echo
    if cmd_at in rec: #se recebeu o echo
        print ('recebi o echo para',cmd_at)
        if 'AT+HTTPACTION' in rec: # Se for o Comando 'AT+HTTPACTION', deve esperar o servidor responder
            print ('recebi at+hpptaction',rec)
            while not '+HTTPACTION:' in rec:
                rec=Read()
                print ('esperando +httpaction',rec)
                if '+HTTPACTION:' in rec:
                    print ('recebi +hppt action',rec)
                    data=int(re.findall(r'\d+',rec)[1])
                    break
        else: #Para os outros comandos, deve aguardar o OK
             while not 'OK' in rec:
                print ('esperando por ok in',rec,'do comando',cmd_at)
                rec=Read()
                if 'ERROR' in rec:break #necessario pular para HTTPINT
                if '+SAPBR: 1,1' in rec:data=rec #para ler o ip
                if '+CSQ:' in rec: data=int(re.findall(r'\d+',rec)[0]) #le intensidade sinal
    print ('fim do send')
    return data

Send('ATE1')
Send('AT+SAPBR=3,1,\"Contype\",\"GPRS\"')
Send('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"')
Send('AT+SAPBR=1,1')
ip=Send('AT+SAPBR=2,1')
#print (ip)
Send('AT+HTTPINIT')
Send('AT+HTTPPARA=\"CID\",1')


i=0
while i<2:
    i=i+1
    st=time.time()
    print ('...Looopppp.................',i)
    signal=Send('AT+CSQ')
    link='http://andrecoelho.tech/SemeaView/send_mysql.php?' #http://andrecoelho.tech/SemeaView/conectado.php
    str_data='LogID='+str(i)
    Send('AT+HTTPPARA=\"URL\",'+link+str_data)
    #time.sleep(10)
    print ('at enviado')
    status=Send('AT+HTTPACTION=0')
    #time.sleep(10)
    print (round(time.time()-st,2),signal,status)
    print ('....................')

Send('AT+SAPBR=0,1')    
print ('fim')
