# -*- coding: utf-8 -*-
#!/usr/bin/python3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtGui,QtCore
import math
import utm
from time import time

# import python files
from seeder_ui import Ui_TabWidget
import operation
import serial
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
from  time import time



pinOnOffButton="P9_12"
'''
GPIO.setup(pinOnOffButton, GPIO.IN) 

#GPS
GPS = serial.Serial ("/dev/ttyO4", 9600) # P9_11 P9_13
# pin
pinRel3="P8_9" #cinza
pinRel4="P8_10" #marrom
pinBotao="P8_16" # marrom
pinEncMotor="P8_13" #verde
pinEncRoda="P8_14" #vermelho
servo_pin="P8_19" # amarelo
#reles
GPIO.setup(pinRel3, GPIO.OUT) 
GPIO.output(pinRel3,GPIO.HIGH)
GPIO.setup(pinRel4, GPIO.OUT) # rele 04
GPIO.output(pinRel4,GPIO.HIGH)
# saida PWM
duty_min = 3.4
duty_max = 13.0 
duty_span = duty_max - duty_min
#PWM.start(servo_pin, duty_min, 50.0)
#Botao habilita semeadora
#GPIO.setup(pinBotao, GPIO.IN) 
#Encoder Roda
#GPIO.setup(pinEncRoda, GPIO.IN)  
#Encoder Motor
#GPIO.setup(pinEncMotor, GPIO.IN) 

'''

# software start
class Semea(QtWidgets.QTabWidget,Ui_TabWidget):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)

        #tab changes
        self.bt_to_conf.clicked.connect(lambda:self.setCurrentIndex(1))
        self.bt_back_main.clicked.connect(lambda:self.setCurrentIndex(0))
        self.bt_to_aux.clicked.connect(lambda:self.setCurrentIndex(2))
        self.bt_back_conf.clicked.connect(lambda:self.setCurrentIndex(1))
        

        #timers
        self.control_timer = QtCore.QTimer()
        self.monitoring_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()

        self.control_timer.timeout.connect(self.ControlFunction)
        self.monitoring_timer.timeout.connect(self.MonitoringFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)

        #OnOff Button Start Stop Control, Monitoring and Encoder
        pinOnOffButton="ON"
        if pinOnOffButton=="ON": #GPIO.input(pinOnOffButton):
            self.control_timer.start(500)
            self.monitoring_timer.start(3000)
            self.encoder_timer.start(10)

        else:
            self.control_timer.stop()
            self.monitoring_timer.stop()
            self.encoder_timer.stop()

            self.ql_speed.setPlainText("")
            self.ql_pdop.setPlainText("")
            self.ql_pol.setPlainText("")
            self.ql_n.setPlainText("")
            self.ql_p.setPlainText("")
            self.ql_k.setPlainText("")
            self.ql_area.setPlainText("")
            self.ql_opcap.setPlainText("")


    def ControlFunction(self):
        # Update Line Edit in MAIN TAB
        self.ql_speed.setPlainText("0.0")
        self.ql_pdop.setPlainText("0.0")
        self.ql_pol.setPlainText("0.0")
        self.ql_n.setPlainText("0.0")
        self.ql_p.setPlainText("0.0")
        self.ql_k.setPlainText("0.0")
        self.ql_area.setPlainText("0.0")
        self.ql_opcap.setPlainText("0.0")
        

    def MonitoringFunction(self):
        pass

    def EncoderFunction(self):
        pass

        '''

        self.timer.start(1000)
        self.timer_encoder.start(5)
        self.has_map=False
        self.st_button=False
        self.status="V" # do gps
        self.lat=0.0
        self.long=0.0
        self.pdop_gps=0.0
        self.dutty_pwm=0.0
        self.vel_roda=0.0
        self.vel_disco=0.0
        self.fator=0.0
        self.p_ant_roda=-1
        self.p_ant_disco=-1
        self.count_roda=0
        self.count_disco=0
        self.t1_d=0
        self.t1_r=0
        #modo
        self.modo=self.modo.currentText()
        self.furos=int(self.furos.currentText())
        self.germinacao=float(self.germ.value())
        #botoes
        self.carrega.clicked.connect(self.CarregaMapa)
        self.sair.clicked.connect(self.Sair)
        #grapghs view
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.pen=QtGui.QPen(QtCore.Qt.red)
        self.brush=QtGui.QBrush(QtCore.Qt.red)
        self.Bpen=QtGui.QPen(QtCore.Qt.blue)
        self.Bbrush=QtGui.QBrush(QtCore.Qt.blue)

    def Sair(self):
        GPIO.output(pinRel3,GPIO.HIGH)
        GPIO.output(pinRel4,GPIO.HIGH)
        GPIO.cleanup() # clean all GPIO ports
        
        PWM.set_duty_cycle(servo_pin, 0.0)
        PWM.stop(servo_pin)
        PWM.cleanup()
        self.close()

    def CarregaMapa(self):
        #ler mapa
        fname = QFileDialog.getOpenFileName(self, 'Select File', '/home/debian')
        self.content=None # clear variable for security
        self.x,self.y,self.populacao,self.espacamento=[],[],[],[]
        with open(fname[0], "r",encoding='latin-1') as f: content = f.read().splitlines()
        f.close
        nlen=len(content)
        for i in range(1,nlen-1): #remove header
            Row=content[i].split(',')
            self.x.append(float(Row[0])) # separe y data in a vector
            self.y.append(float(Row[1])) # separe y data in a vector
            self.populacao.append(float(Row[2])) # separe z data in a vector
            self.espacamento.append(float(Row[3]))
      #plot in GraphicsView
            self.scene.addRect(float(Row[0]),float(Row[1]),1,1,self.pen,self.brush)
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
        if len(self.x)>0: self.has_map=True
 
    def LigaDosador(self):
        GPIO.output(pinRel3,GPIO.LOW)
        GPIO.output(pinRel4,GPIO.LOW)

    def DesligaDosador(self):
        GPIO.output(pinRel3,GPIO.HIGH)
        GPIO.output(pinRel4,GPIO.HIGH)

    def Encoder(self):
        #ler encoder roda
        
        if GPIO.input(pinEncRoda):p_atual_roda=1
        else: p_atual_roda=0
        if self.count_roda==0: self.t1_r=time()
        if self.p_ant_roda==1 and p_atual_roda==0:
            self.count_roda=self.count_roda+1
        if self.count_roda==15: # 1/3 volta
            self.count_roda=0
            self.vel_roda=2.00/(time()-self.t1_r)
            #print (self.vel_roda)
        self.p_ant_roda=p_atual_roda

        # encoder disco
        if GPIO.input(pinEncMotor):p_atual_disco=1
        else: p_atual_disco=0
        if self.count_disco==0: self.t1_d=time()
        if self.p_ant_disco==1 and p_atual_disco==0:
            self.count_disco=self.count_disco+1
        if self.count_disco==15: # 1 volta
            self.count_disco=0
            self.vel_disco=1*60/(time()-self.t1_d) 
        self.p_ant_disco=p_atual_disco
        
        
    def Update(self):  #Funcção de atualização continua
        #self.LerGPS()
        #self.vel_roda=1.0
        self.vel_sensor.setPlainText(str(round(self.vel_roda,2)))
        # ler botao habilitar
        #if GPIO.input(pinBotao): self.st_button=True
        #else: self.st_button=False
        if self.status=='A' and self.has_map is True and self.modo=="MAP" and self.st_button is True and self.vel_roda>0.1:
                self.st_semeadora.setPlainText("Em operacao")
                self.vel_sensor.setPlainText(str(round(self.vel_roda,2)))
                # procura ponto no mapa com posicao mais proxima da atual
                # retorna a populacao e o espacamento
                populacao,espacamento,xx,yy=operation.FindNeig(self.x,self.y,self.populacao,self.espacamento,self.lat,self.long)
                # add ponto atual e ponto vizinho proximo
                self.scene.addRect(xx,yy,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(self.lat,self.long,0.5,0.5,self.Bpen,self.Bbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                #calcula a rotacao
                rotacao,sementes=seeder.Seeder(self.vel_roda,populacao,espacamento,self.furos,self.germinacao)
                self.line_rot.setPlainText("SP: "+str(round(rotacao,2))+"  R: "+str(round(self.vel_disco,2)))
                self.line_plantas.setPlainText(str(round(sementes,2)))
                #ajuste fino da velocidade
                #if (rotacao-self.vel_disco<-0.5):self.fator=self.fator-0.1
                #if (rotacao-self.vel_disco>0.5):self.fator=self.fator+0.1
                #calcula o dutycicle 0=0 RPM   100=15RPM  dc=x rpm
                #self.dc=self.fator+100*rotacao/15 #fator para ajuste entre rotacao atual e desejada
                self.LigaDosador()

        if (self.modo=="MAP" and (self.status=='V' or self.has_map is False)) or self.st_button is False or self.vel_roda<0.1:
                self.DesligaDosador()
                self.st_semeadora.setPlainText("Paused")
                self.line_rot.setPlainText("0")
                self.line_plantas.setPlainText("0")

        if self.modo=="MANUAL" and self.st_button is True and self.vel_roda>0.1:
                self.st_semeadora.setPlainText("In operation")
                populacao=float(self.pol.currentText())
                espacamento=float(self.espac.currentText())
                rotacao,sementes=operation.Seeder(self.vel_roda,populacao,espacamento,self.furos,self.germinacao)
                self.line_rot.setPlainText("C: "+str(round(rotacao,2))+"  R: "+str(round(self.vel_disco,2)))
                self.line_plantas.setPlainText(str(round(sementes,2)))
                #ajuste fino da velocidade
                if (rotacao-self.vel_disco<-0.5):self.fator=self.fator-0.1
                if (rotacao-self.vel_disco>0.5):self.fator=self.fator+0.1
                #controla a velocidade
                dc=self.fator+0.22086*rotacao+1.5057
                #dc=duty_min+4*2.4
                #PWM.set_duty_cycle(servo_pin,dc )
                self.LigaDosador()

    def LerGPS(self):
        nmea=GPS.readline()
        try:
            nmea=nmea.decode("utf-8")
            nmea_array=nmea.split(',')
            size=len(nmea_array)
            if nmea_array[0]=='$GPRMC':
                self.status=nmea_array[2]  # olha status
                if self.status=='A' and size==13:
                    latMin=float(nmea_array[3][2:])/60   # converte em decimo de graus
                    lat=((float(nmea_array[3][0:2])+latMin)) #
                    lonMin=float(nmea_array[5][3:])/60   # converte em decimo de graus
                    lon=((float(nmea_array[5][0:3])+lonMin)) #
                    latHem=nmea_array[4]  # N or S
                    lonHem=nmea_array[6]  # W or E
                    if lonHem=='W': lon=-lon
                    if latHem=='S': lat=-lat
                    utm_conv=utm.from_latlon(lat,lon)
                    self.lat=utm_conv[0]
                    self.long=utm_conv[1]
                    #print (self.lat,self.long)
            if nmea_array[0]=='$GPGSA'and self.status=='A' and size==18:
                self.pdop_gps=nmea_array[-3] # pdop
        except:
            pass
        #atualiza aba status
        self.pdop.setPlainText(str(self.pdop_gps))
'''        
#Run the app:
if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    ex = Semea()
    ex.show()
    sys.exit(app.exec_())

