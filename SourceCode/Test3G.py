# -*- coding: utf-8 -*-
#!/usr/bin/python3
#Programa Base para desenvolvimento de interfaces em PyQt5
#Autor : Andre Luiz de Freitas Coelho
#Versão : 1.0
#
#
import sys
from PyQt5.QtWidgets import QDialog, QApplication
from ggg import Ui_Dialog
import serial
import time
import re
import os

direc=os.path.dirname(os.path.abspath(__file__))

#Le arquivo de dados
with open(os.path.join(direc,"M0_F0_original1.csv"),"r",encoding="latin-1") as f:
    dataset=f.read().splitlines()
f.close()

file_out=os.path.join(direc,"M0_F0_remoto.txt")
#Cria Arquivo de Saida
with open(file_out,"a",encoding="latin-1") as f:
    f.write('Teste\n')
    f.write('I,ID,Tempo Loop,Sleep,Date,Time,Signal,Status\n')
f.close()


#3G
sim800l= serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26

class AppWindow(QDialog): 
    def __init__(self):
        super().__init__()
        self.ui = Ui_Dialog() 
        self.ui.setupUi(self)



        self.ui.bt_start.clicked.connect(self.Start)
        self.ui.label.setText('Inicio')


        #Leitura
    def Read(self):
        rec=sim800l.readline()
        return rec.decode('utf-8')

#Envia AT e processa resposta
    def Send(self,cmd_at):
        rec=''
        data=''
        sim800l.write(str.encode(cmd_at+'\r'))
        while not cmd_at in rec:rec=self.Read() #enquanto não le o echo
        if cmd_at in rec: #se recebeu o echo
            if 'AT+HTTPACTION' in rec: # Se for o Comando 'AT+HTTPACTION', deve esperar o servidor responder
                while not '+HTTPACTION:' in rec:
                    rec=self.Read()
                    if '+HTTPACTION:' in rec:
                        data=re.findall(r'\d+',rec)[1]
                        break
            else: #Para os outros comandos, deve aguardar o OK
                 while not 'OK' in rec:
                    rec=self.Read()
                    if 'ERROR' in rec:break #necessario pular para HTTPINT
                    if '+SAPBR: 1,1' in rec:data=rec #para ler o ip
                    if '+CSQ:' in rec: data=re.findall(r'\d+',rec)[0] #le intensidade sinal
        return data

    def Start(self):
        print('Inicio')
        for t_step in range (10,0,-1): #varia o tempo entre 1 e 10 s
            print ('time',t_step)
            self.Send('ATE1')
            self.Send('AT+SAPBR=3,1,\"Contype\",\"GPRS\"')
            self.Send('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"')
            self.Send('AT+SAPBR=1,1')
            ip=self.Send('AT+SAPBR=2,1')
            print (ip)
            self.Send('AT+HTTPINIT')
            self.Send('AT+HTTPPARA=\"CID\",1')


            with open(file_out,"a",encoding="latin-1") as f:
                    f.write("TimeStep"+str(t_step)+"\n")
                    f.write("IP"+ip+"\n")
            f.close()

            for i in range (1,251): #para se obter 250 dados enviados
                
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
                signal=self.Send('AT+CSQ')
                link='http://andrecoelho.tech/SemeaView/send_mysql.php?' #http://andrecoelho.tech/SemeaView/conectado.php
                str_data='LogID='+str(i)+'&Date='+str(t_step)+'&Time='+str(ttime)+'&MachineID='+machineID\
            +'&FieldID='+fieldID+'&Lati='+str(lat)+'&Longi='+str(long)+'&XUtm='+str(lat_utm)+'&YUtm='+str(long_utm)+'&Speed='+\
            str(mach_speed)+'&OpCap='+str(opcap)+'&TimeOperation='+str(time_operation)+'&Population='+str(popseed)+'&FertRatio='+\
            str(fert_rt)+'&FertWgt='+str(fert_wgt)+'&Area='+str(area)
                
                self.Send('AT+HTTPPARA=\"URL\",'+link+str_data)
                status=self.Send('AT+HTTPACTION=0')
                print (status)

                t_loop=time.time()-st
                try:
                    t_sleep=t_step-(time.time()-st)
                    time.sleep(t_sleep) #programa fica esperando dar o time step, para padronizar o tempo
                except :
                    status=status+'-TIMEOUT'
                    pass
                
                #escreve no arquivo de resposta
                str_write=str(i)+','+log_id+','+str(round(t_loop,2))+','+str(round(time.time()-st,2))+','+date+','+ttime+','+signal+','+status+'\n'
                with open(file_out,"a",encoding="latin-1") as f:
                    f.write(str_write)
                f.close()
            

            self.Send('AT+SAPBR=0,1')
            print ('troca tempo')
        self.ui.label.setText('Fim')
        print ('fim')

#Executa a interface
app = QApplication(sys.argv)
w = AppWindow()
w.show()
sys.exit(app.exec_())
