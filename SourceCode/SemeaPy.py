#22 de julho
# -*- coding: utf-8 -*-

#!/usr/bin/python3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import QDateTime,Qt
import math
import utm
from time import time
import serial
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC


# import python files
from seeder_ui import Ui_SEMEA #gui
import operation #calculations

#OnOff
pinOnOffButton="P9_12"
GPIO.setup(pinOnOffButton, GPIO.IN) 
#GPS
os.system("sh start.sh") #enable UART PIN Script
ser = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13
#PWM Seed
rotmax_seed=60 #maximum rotation of motor (in RPM)
pinPWM_Seed="P9_14"
PWM.start(pinPWM_Seed,0, 500.0) #pin, duty,frequencia
pinEnable_Seed="P9_18"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
#PWM Fertilizer
rotmax_fert=80
pinPWM_Fert="P9_16"
PWM.start(pinPWM_Fert,0, 500.0) #pin, duty,frequencia
pinEnable_Fert="P9_20"
GPIO.setup(pinEnable_Fert, GPIO.OUT)
GPIO.output(pinEnable_Fert,GPIO.LOW)
#Encoder Velocidade Deslocamento
pinEncA_Roda="P9_15"
pinEncB_Roda="P9_17"
GPIO.setup(pinEncA_Roda, GPIO.IN)
GPIO.setup(pinEncB_Roda, GPIO.IN)
#Encoder Dosador Semente
pinEncA_Seed="P9_21"
pinEncB_Seed="P9_23"
GPIO.setup(pinEncA_Seed, GPIO.IN)
GPIO.setup(pinEncB_Seed, GPIO.IN) 
#Celula de Carga
ADC.setup()
pinLoadCell="P9_33"

# software start
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)

        #global variables
        self.speed=0.0 #speed of machine
        self.spd_seed=0.0 #angular velocity of seed distributor
        self.pdop=0.0
        self.popseed=0.0 #population 
        self.fert_rt=0.0
        self.fert_wgt=0.0
        self.area=0.0
        self.opcap=0.0
        self.time_operation=0
        self.last_st_seed=-1
        self.aux_i_seed=0
        self.time_start_seed=0
        self.aux_j_seed=0
        self.real_rot_seed=0
        self.last_st_mach=-1
        self.aux_i_mach=0
        self.time_start_mach=0
        self.aux_j_mach=0
        self.fertbym=0
        self.seedbym=0
        self.lat=0
        self.long=0
        self.pdop=0
        self.status=0 #of gps
        self.row_spacing=0
        self.disk_hole=0
        self.seed_germ=0
        self.st_has_logfile=False #the user define a loggile?
        self.logfile_name=""
        self.lat_map_fert,self.long_map_fert,self.map_fert=[],[],[]
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[]
        self.aux_logger=0 #auxiliar for save datas each 5 * timestep of control
        self.seed_mode="OFF"
        self.fert_mode="OFF"

        #tab changes
        self.bt_to_conf.clicked.connect(lambda:self.setCurrentIndex(1))
        self.bt_back_main.clicked.connect(lambda:self.setCurrentIndex(0))
        self.bt_to_aux.clicked.connect(lambda:self.setCurrentIndex(2))
        self.bt_back_conf.clicked.connect(lambda:self.setCurrentIndex(1))

        #exit button
        self.exit.clicked.connect(self.Close)
        
        #timers
        self.control_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)
 
        #start timer for control
        self.control_timer.start(1000)
        #button for logfile
        self.bt_define_logfile.clicked.connect(self.DefineLogFile)
        self.bt_reload_logfile.clicked.connect(self.ReloadLogFile)

        #button for map
        self.bt_load_seed.clicked.connect(self.LoadSeedFile)
        self.bt_load_fert.clicked.connect(self.LoadFertFile)

        #grapghs view
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.Rpen=QtGui.QPen(QtCore.Qt.red)
        self.Rbrush=QtGui.QBrush(QtCore.Qt.red)
        self.Bpen=QtGui.QPen(QtCore.Qt.blue)
        self.Bbrush=QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen=QtGui.QPen(QtCore.Qt.green)
        self.Gbrush=QtGui.QBrush(QtCore.Qt.green)
        self.Kpen=QtGui.QPen(QtCore.Qt.black)
        self.Kbrush=QtGui.QBrush(QtCore.Qt.black)


    def DefineLogFile(self):
        self.logfile_name=QFileDialog.getSaveFileName(self,"Save","","*.txt")[0]
        self.st_has_logfile=True
        f=open(self.logfile_name,'w')
        f.write("Data/Hora,Lat-Atual(m),Long-Atual(m),Speed (m/s),PDOP, \
          PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),OpCap(ha/h),Area(ha),Time Operation (h),Row Spacing(m), Holes,SeedByM,FertByM, Seed Mode, Fert Mode")
        f.write("\n")
        f.close()
                
    def ReloadLogFile(self):
        self.logfile_name=QFileDialog.getOpenFileName(self,"Open Log File",".","*.txt")[0]
        self.st_has_logfile=True

    def LoadSeedFile(self):
        seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File",".","*.txt")[0]
        content=None # clear variable for security
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[]
        with open(seedfile_name, "r",encoding='latin-1') as f:
            content = f.read().splitlines()
        f.close
        for i in range(1,len(content)-1): #remove header
            Row=content[i].split(',')
            self.lat_map_seed.append(float(Row[0])) 
            self.long_map_seed.append(float(Row[1]))
            self.pop_map_seed.append(float(Row[2]))
            #plot in GraphicsView
            self.scene.addRect(float(Row[0]),float(Row[1]),1,1,self.Rpen,self.Rbrush)
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)

    def LoadFertFile(self):
        fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File",".","*.txt")[0]
        self.lat_map_fert,self.long_map_fert,self.pop_map_fert=[],[],[]
        with open(fertfile_name, "r",encoding='latin-1') as f:
            content = f.read().splitlines()
        f.close
        for i in range(1,len(content)-1): #remove header
            Row=content[i].split(',')
            self.lat_map_fert.append(float(Row[0])) 
            self.long_map_fert.append(float(Row[1]))
            self.map_fert.append(float(Row[2]))
            self.scene.addRect(float(Row[0]),float(Row[1]),1,1,self.Bpen,self.Bbrush)
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)

      
    def Close(self):
        # add warning for confirm close software
        self.control_timer.stop()
        self.encoder_timer.stop()
        GPIO.cleanup() #clear pin configuration
        PWM.cleanup()
        self.close() #close software

        
    def ControlFunction(self):  #Main Loop of Software
        
        if  GPIO.input(pinOnOffButton): #If button is on
            # Start Encoder Function
            self.encoder_timer.start(50)
            
            # Read Germination
            self.seed_germ=int(self.sl_germ.value())
            self.ql_germ.setPlainText(str(self.seed_germ))

            #Read Hole Disk
            self.disk_hole=int(self.list_holes.currentText())

            #Read GPS (if MODE=FIX, only for log)
            #self.GPSRead()

            #Read Spacing
            self.row_spacing=float(self.sl_row_spacing.value())/100.0
            self.ql_row_spacing.setPlainText(str(self.row_spacing))

            #Read Fert Weight
            self.fert_wgt=float(("{0:.4f}").format(ADC.read(pinLoadCell)))

            #Read Simulated Speed (By test only)
            if self.cb_motion_simulate.isChecked(): 
                self.speed=float(self.sl_sim_speed.value())/50.0 
                self.ql_sim_speed.setPlainText(str(self.speed))

            ###Seeder Distributor###
                
            if self.list_seed.currentText()=="FIX": #Fix seed distribuition rate
                self.seed_mode="FIX"
                self.popseed=int(self.sl_popseed.value())*5000 
                self.ql_set_pop.setPlainText(str(self.popseed))

                if self.pdop<5: #plot in graph if have gps signal
                    self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                else:
                    self.lat_atual=-999
                    self.long_atual=-999
                    
                #calculate angular velocity of dc motor (in RPM)
                rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                GPIO.output(pinEnable_Seed,GPIO.HIGH)

                #ADD CORRECTION

                dtc_seed=100*rot_seed/rotmax_seed
                PWM.set_duty_cycle(pinPWM_Seed, dtc_seed)
                
      
            if self.list_seed.currentText()=="MAP": #Map for seed distribuition rate
                self.seed_mode="MAP"
                if len(self.lat_map_seed) >0: # if user loaded the map
                    self.ql_set_pop.setPlainText("OK MAP")

                    if self.pdop<5.0: # if gps signal is good

                        # find in the map the point nearst to atual point. The return it's the population and the map point used
                        self.popseed,lat_used,long_used=operation.FindNeig(self.lat_atual,self.long_atual,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
                        # add atual and used point
                        self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                        self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
                        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                    
                        #calculate angular velocity of dc motor (in RPM)
                        rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                        GPIO.output(pinEnable_Seed,GPIO.HIGH)

                        #ADD CORRECTION
                        
                        dtc_seed=100*rot_seed/rotmax_seed
                        PWM.set_duty_cycle(pinPWM_Seed, dtc_seed)                        
                else:
                    GPIO.output(pinEnable_Seed,GPIO.LOW)
                    self.popseed=-999
                    self.ql_set_pop.setPlainText("NO MAP OR GPS")
             
            if self.list_seed.currentText()=="OFF":
                self.seed_mode="OFF"
                GPIO.output(pinEnable_Seed,GPIO.LOW)
                self.popseed="OFF"
                self.ql_set_pop.setPlainText("OFF")
                self.lat_atual=-999
                self.long_atual=-999


            ####Fertilizer Distribution###
            if self.list_fert.currentText()=="FIX": #Fix fert distribuition rate
                self.fert_mode="FIX"
                self.fert_rt=int(self.sl_fert.value())*6 
                self.ql_set_fert.setPlainText(str(self.fert_rt))

                if self.pdop<5: #plot in graph if have gps signal
                    self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                else:
                    self.lat_atual=-999
                    self.long_atual=-999
                    
                #calculate angular velocity of dc motor (in RPM)
                rot_fert,self.fertbym=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
                GPIO.output(pinEnable_Fert,GPIO.HIGH)

                #ADD CALIBRATION
                
                dtc_fert=100*rot_fert/rotmax_fert
                PWM.set_duty_cycle(pinPWM_Fert, dtc_fert)
  
            if self.list_fert.currentText()=="MAP": #Map for seed distribuition rate
                self.fert_mode="MAP"
                if len(self.lat_map_fert) >0:
                    self.ql_set_fert.setPlainText("OK MAP")

                    if self.pdop<5.0: # if gps signal is good

                        # find in the map the point nearst to atual point. The return it's the population and the map point used
                        self.fert_rt,lat_used,long_used=operation.FindNeig(self.lat_atual,self.long_atual,self.lat_map_fert,self.long_map_fert,self.map_fert)
                        # add atual and used point
                        self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                        self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
                        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                    
                        #calculate angular velocity of dc motor (in RPM)
                        rot_fert,self.fertbym=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
                        GPIO.output(pinEnable_Fert,GPIO.HIGH)

                        #ADD CALIBRATION
                        
                        dtc_fert=100*rot_fert/rotmax_fert
                        PWM.set_duty_cycle(pinPWM_Fert, dtc_fert)

                else:
                    GPIO.output(pinEnable_Fert,GPIO.LOW)
                    self.popsfert_rteed=-999
                    self.ql_set_fert.setPlainText("NO MAP OR GPS")
                    self.lat_atual=-999
                    self.long_atual=-999

            if self.list_fert.currentText()=="OFF":
                self.fert_mode="OFF"
                GPIO.output(pinEnable_Fert,GPIO.LOW)
                self.fert_rt="OFF"
                self.fert_wgt=0.0
                self.ql_set_fert.setPlainText("OFF")


            #calculate operational capability and area
            self.area=self.area+(self.row_spacing*self.speed*1/10000) #in ha
            self.area=float(("{0:.4f}").format(self.area))
            self.time_operation=self.time_operation+(1/60)   # im minutes
            self.time_operation=float(("{0:.4f}").format(self.time_operation))
            self.opcap=self.area/(self.time_operation/60) # in ha/h
            self.opcap=float(("{0:.4f}").format(self.opcap))

            #Logger
            if self.speed>0.1: # save datas if machine is operation
                self.aux_logger=self.aux_logger+1
                if self.aux_logger==5: #save datas each 5 s
                    self.LoggerFunction()
                    self.aux_logger=0

             
        else: # If button is off
            self.encoder_timer.stop()
            self.speed="BUTTON OFF"
            self.pdop="OFF"
            self.popseed="OFF"
            self.fert_rt="OFF"
            self.fert_wgt="OFF"
            self.area="OFF"
            self.opcap="OFF"

        # Update LineEdit in main tab
        self.ql_speed.setPlainText(str(self.speed))
        self.ql_pdop.setPlainText(str(self.pdop))
        self.ql_seed.setPlainText(str(self.popseed))
        self.ql_fert_rt.setPlainText(str(self.fert_rt))
        self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
        self.ql_area.setPlainText(str(self.area))
        self.ql_opcap.setPlainText(str(self.opcap))
        

    def LoggerFunction(self):
        if self.st_has_logfile is False:
            self.logfile_name="res.txt"
        string="Saving in: "+str(self.logfile_name)
        self.ql_logfile.setPlainText(string)

        if self.cb_remote.isChecked():
            self.ql_remote_status.setPlainText("Enable")

            #add send buffer by bluetooth

            
        else:
            self.ql_remote_status.setPlainText("Disable")

        f=open(self.logfile_name,'a')
        f.write(QDateTime.currentDateTime().toString(Qt.ISODate))
        f.write(",")
        f.write(str(self.lat_atual))
        f.write(",")
        f.write(str(self.long_atual))
        f.write(",")
        f.write(str(self.speed))
        f.write(",")
        f.write(str(self.pdop))
        f.write(",")
        f.write(str(self.popseed))
        f.write(",")
        f.write(str(self.fert_rt))
        f.write(",")
        f.write(str(self.fert_wgt))
        f.write(",")
        f.write(str(self.opcap))
        f.write(",")
        f.write(str(self.area))
        f.write(",")
        f.write(str(self.time_operation))
        f.write(",")
        f.write(str(self.row_spacing))
        f.write(",")
        f.write(str(self.disk_hole))
        f.write(",")
        f.write(str(self.seedbym))
        f.write(",")
        f.write(str(self.fertbym))
        f.write(",")
        f.write(str(self.seed_mode))
        f.write(",")
        f.write(str(self.fert_mode))
        f.write(",")
        f.write("\n")
        f.close()
    
    def EncoderFunction(self):
        atual_st_seed=GPIO.input(pinEncA_Seed) #encoder position (0 ou 1)
        if (self.last_st_seed==0 and atual_st_seed==1): #if have up border
            self.aux_i_seed=self.aux_i_seed+1
        if (self.aux_i_seed==1): #if one up border is detectec start the time
             self.time_start_seed=time.time()
        if (self.aux_i_seed==3): #at complete three up border, calculate the velocity (one revolution is 15 up border)
            self.real_rot_seed=60*.2/(time.time()-ts)
            self.aux_i_seed=0
        if (self.last_st_seed==atual_st_seed): #for dectect if encoder its stop, 
            self.aux_j_seed=self.aux_j_seed+1
        if (self.aux_j_seed>100): #if encoder is stop much time, v=0
            self.aux_j_seed=0
            self.real_rot_seed=0
        self.last_st_seed=atual_st_seed #update last status
        
        atual_st_mach=GPIO.input(pinEncA_Roda) #encoder position (0 ou 1)
        if (self.last_st_mach==0 and atual_st_mach==1): #if have up border
            self.aux_i_mach=self.aux_i_mach+1
        if (self.aux_i_mach==1): #if one up border is detectec start the time
             self.time_start_mach=time.time()
        if (self.aux_i_mach==3): #at complete three up border, calculate the velocity (one revolution is 15 up border)
            self.speed=2.0*.2/(time.time()-ts)
            self.aux_i_mach=0
        if (self.last_st_mach==atual_st_mach): #for dectect if encoder its stop, 
            self.aux_j_mach=self.aux_j_mach+1
        if (self.aux_j_mach>100): #if encoder is stop much time, v=0
            self.aux_j_mach=0
            self.speed=0
        self.last_st_mach=atual_st_mach #update last status
 
    #Read NMEA Sentences
    def GPSRead(self):
        nmea=ser.readline()
        try:
            nmea=nmea.decode("utf-8")
            nmea_array=nmea.split(',')
            size=len(nmea_array)
            if nmea_array[0]=='$GPRMC':
                self.status=nmea_array[2]  # check status
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
                    self.lat_atual=utm_conv[0]
                    self.long_atual=utm_conv[1]
            if nmea_array[0]=='$GPGSA'and self.status=='A' and size==18:
                self.pdop=nmea_array[-3] # pdop
            if self.status=='V':
                self.pdop=999.99
        except:
            pass


#Run the app:
if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    ex = Semea()
    ex.show()
    sys.exit(app.exec_())

