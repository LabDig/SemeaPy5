#06 de agosto
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
# import python files
from seeder_ui import Ui_SEMEA #gui
import operation #calculations


#Because conflict between Debian 8.6, 4D 7" LCD-CAPE, Adafruit and Python 3.6
# Adafruit library is not used
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
#OnOff
pinOnOffButton="P8_15"
GPIO.setup(pinOnOffButton, GPIO.IN) 
#PWM Seed
rotmax_seed=60 #maximum rotation of motor (in RPM)
pinPWM_Seed="P8_19"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_12"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
#PWM Fertilizer
rotmax_fert=80
pinPWM_Fert="P8_13"
PWM.start(pinPWM_Fert,0, 1000.0) #pin, duty,frequencia
pinEnable_Fert="P8_11"
GPIO.setup(pinEnable_Fert, GPIO.OUT)
GPIO.output(pinEnable_Fert,GPIO.LOW)
#Encoder Velocidade Deslocamento
pinEncA_Wheel="P8_7"
pinEncB_Wheel="P8_9"
GPIO.setup(pinEncA_Wheel, GPIO.IN)
GPIO.setup(pinEncB_Wheel, GPIO.IN)
#Encoder Dosador Semente
pinEncA_Seed="P8_8"
pinEncB_Seed="P8_10"
GPIO.setup(pinEncA_Seed, GPIO.IN)
GPIO.setup(pinEncB_Seed, GPIO.IN) 
#Celula de Carga
ADC.setup()
pinLoadCell="P9_33"
#GPS
ser = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13


'''
#import gpio #enable and read gpio
gps="BB-UART4"
ibt2="BB-PWM2"
pinOnOffButton=47 #P8_15
pinPWM_Fert=0 #pwm0 in pwmchip2 P8_19
pinPWM_Seed=1 #pwm0 in pwmchip2 P8_13
pinEncA_Wheel=66 #"P8_7"
pinEncB_Wheel=69 #"P8_9"
pinEncA_Seed=67 #"P8_8"
pinEncB_Seed=68 #"P8_10"
pinEnable_Seed=44 #"P8_12"
pinEnable_Fert=45 #"P8_11"

#Config and Start GPIO's
os.system("sh start.sh")
#GPS
ser = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13
#PWM
T=1000000 #Period for PWM Signal in ns (Frequency 1 khz)
max_rot_seed=80
max_rot_fert=120

'''
# software start
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        
        #global variables
        self.speed,self.spd_seed,self.pdop,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap,self.time_operation=0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
        self.last_st_seed,self.aux_i_seed,self.time_start_seed,self.aux_j_seed,self.real_rot_seed,self.last_st_mach, self.aux_i_mach=-1,0,0,0,-1,0,0
        self.time_start_mach,self.aux_j_mach,self.fertbym,self.seedbym,self.lat,self.long,self.pdop,self.status=0,0,0,0,0,0,0,0
        self.row_spacing,self.disk_hole,self.seed_germ,self.st_has_logfile,self.logfile_name,self.aux_logger=0,0,0,False,"",0
        self.lat_map_fert,self.long_map_fert,self.map_fert,self.lat_atual,self.long_atual=[],[],[],0,0
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[]
        self.seed_mode,self.fert_mode="OFF","OFF"
        self.rot_seed,self.rot_fert,self.seed_cor=0.0,0.0,0
  
        #exit button
        self.exit.clicked.connect(self.Close)
        
        #timers
        self.control_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)
        self.control_timer.start(1000)
        #button for logfile
        self.bt_define_logfile.clicked.connect(self.DefineLogFile)
        self.bt_reload_logfile.clicked.connect(self.ReloadLogFile)
        #button for map
        self.bt_load_seed.clicked.connect(self.LoadSeedFile)
        self.bt_load_fert.clicked.connect(self.LoadFertFile)
        #buton for tare
        self.bt_tare.clicked.connect(lambda: print ("Tare"))
        #buton ok cal
        self.bt_ok_cal.clicked.connect(lambda: print ("OK cal"))
        self.lb_cal.setText("Tank + 5kg")
        #buton calibrate
        self.bt_calibrate.clicked.connect(lambda: print ("Calibrate"))
        
        #grapghs view
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)

    def DefineLogFile(self):
        self.logfile_name=QFileDialog.getSaveFileName(self,"Save","","*.txt")[0]
        self.st_has_logfile=True
        f=open(self.logfile_name,'w')
        f.write("Data/Hora,Lat-Atual(m),Long-Atual(m),Speed (m/s),PDOP, \
          PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),OpCap(ha/h),Area(ha),Time Operation (h),Row Spacing(m), Holes,SeedByM,Real Rot Seed, FertByM, Seed Mode, Fert Mode")
        f.write("\n")
        f.close()
                
    def ReloadLogFile(self):
        self.logfile_name=QFileDialog.getOpenFileName(self,"Open Log File",".","*.txt")[0]
        self.st_has_logfile=True

    def LoadSeedFile(self):
        seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File",".","*.txt")[0]
        content=None # clear variable for security
        with open(seedfile_name, "r",encoding='latin-1') as f: content = f.read().splitlines()
        f.close
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed=operation.ReadMapFile(content)
        for i in range(len(self.lat_map_seed)):self.scene.addRect(self.lat_map_seed[i],self.long_map_seed[i],1,1,self.Rpen,self.Rbrush)
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)

    def LoadFertFile(self):
        fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File",".","*.txt")[0]
        content=None # clear variable for security
        with open(fertfile_name, "r",encoding='latin-1') as f:  content = f.read().splitlines()
        f.close
        self.lat_map_fert,self.long_map_fert,self.pop_map_fert=operation.ReadMapFile(content)
        for i in range(len(sel.lat_map_fert)):self.scene.addRect(self.lat_map_fert[i],self.long_map_fert[i],1,1,self.Bpen,self.Bbrush)
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)

    def ControlFunction(self):  #Main Loop of Software
        if GPIO.input(pinOnOffButton):
        #if gpio.Button(pinOnOffButton) is True: #If button is on
            self.lb_status.setText("Button ON")
            self.encoder_timer.start(10) # Start Encoder Function
            self.seed_germ=int(self.sl_germ.value())             # Read Germination
            self.ql_germ.setPlainText(str(self.seed_germ))
            self.disk_hole=int(self.list_holes.currentText())             #Read Hole Disk
            self.lat_atual,self.long_atual,self.pdop,self.status=operation.ReadGPS(ser.readline())   #Read GPS (if MODE=FIX, only for log)
            self.row_spacing=float(self.sl_row_spacing.value())/100.0             #Read Spacing
            self.ql_row_spacing.setPlainText(str(self.row_spacing))
            #self.fert_wgt=operation.ReadWeight(pinLoadCell)             #Read Fert Weight

            #Read Simulated Speed (By test only)
            if self.cb_motion_simulate.isChecked(): 
                self.speed=float(self.sl_sim_speed.value())/50.0 
                self.ql_sim_speed.setPlainText(str(self.speed))

            ###Seeder Distributor###

            if self.list_seed.currentText()=="FIX": #Fix seed distribuition rate
                self.seed_mode="FIX"
                self.popseed=int(self.sl_popseed.value())*5000 
                self.ql_set_pop.setPlainText(str(self.popseed))
                if self.pdop<3.0 and self.lat_atual!=0: #plot in graph if have gps signal
                    self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                else: self.lat_atual,self.long_atual=-999,-999

                #calculate angular velocity of dc motor (in RPM)
                self.rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                dt=(100*self.rot_seed/60.0)+self.seed_cor
                if dt>100.0 : dt=100.0
                if dt<20 : dt=20.0
                PWM.set_duty_cycle(pinPWM_Seed,dt)
                GPIO.output(pinEnable_Seed,GPIO.HIGH)
                self.seed_cor=self.seed_cor+(self.rot_seed-self.real_rot_seed)/20
                self.lb_status.setText("Rot Seed:"+str(self.real_rot_seed)+"...."+str(self.rot_seed))
                #gpio.EnableSeed("HIGH")
                #gpio.PWMSeed(self.rot_seed,T,max_rot_seed)
 
            if self.list_seed.currentText()=="MAP": #Map for seed distribuition rate
                self.seed_mode="MAP"
                if len(self.lat_map_seed) >0 and self.pdop<3.0 and self.lat_atual!=0: # if user loaded the map and gps is good
                    self.ql_set_pop.setPlainText("OK MAP/GPS")
                    # find in the map the point nearst to atual point. The return it's the population and the map point used
                    self.popseed,lat_used,long_used=operation.FindNeig(self.lat_atual,self.long_atual,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
                    # add atual and used point
                    self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                    self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)

                    #calculate angular velocity of dc motor (in RPM)
                    self.rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                    dt=(100*self.rot_seed/60.0)+self.seed_cor
                    if dt>100.0 : dt=100.0
                    if dt<20 : dt=20.0
                    PWM.set_duty_cycle(pinPWM_Seed,dt)
                    GPIO.output(pinEnable_Seed,GPIO.HIGH)
                    self.seed_cor=self.seed_cor+(self.rot_seed-self.real_rot_seed)/20
                    self.lb_status.setText("Rot Seed:"+str(self.real_rot_seed)+"...."+str(self.rot_seed))
                else:
                    self.rot_seed=0.0
                    self.ql_set_pop.setPlainText("NO MAP/GPS")
                    GPIO.output(pinEnable_Seed,GPIO.LOW)
                    PWM.set_duty_cycle(pinPWM_Seed,0)
             
            if self.list_seed.currentText()=="OFF" or self.speed==0:
                self.seed_mode="OFF"
                self.rot_seed=0.0
                GPIO.output(pinEnable_Seed,GPIO.LOW)
                PWM.set_duty_cycle(pinPWM_Seed,0)
            
            ''' 
            ####Fertilizer Distribution###
            if self.list_fert.currentText()=="FIX": #Fix fert distribuition rate
                self.fert_mode="FIX"
                self.fert_rt=int(self.sl_fert.value())*6 
                self.ql_set_fert.setPlainText(str(self.fert_rt))
                if self.pdop<5: #plot in graph if have gps signal
                    self.scene.addRect(self.lat_atual,self.long_atual,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                else: self.lat_atual,self.long_atual=-999,-999
                #calculate angular velocity of dc motor (in RPM)
                self.rot_fert,self.fertbym=operation.Fert(self.speed,self.fert_rt,self.row_spacing)

                GPIO.output(pinEnable_Fert,GPIO.HIGH)
                PWM.set_duty_cycle(pinPWM_Fert,100.0)
  
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
                        self.rot_fert,self.fertbym=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
                else:
                    self.rot_fert=0.0
                    self.popsfert_rteed=-999
                    self.ql_set_fert.setPlainText("NO MAP OR GPS")
                    GPIO.output(pinEnable_Fert,GPIO.LOW)

            if self.list_fert.currentText()=="OFF":
                self.rot_fert=0.0
                self.fert_mode,self.fert_rt="OFF","OFF"
                self.fert_wgt=0.0
                self.ql_set_fert.setPlainText("OFF")
                GPIO.output(pinEnable_Fert,GPIO.LOW)

 
            #gpio.EnableFert("HIGH")
            #gpio.PWMFert(self.rot_fert,T,max_rot_fert)
            '''

  
            #calculate operational capability and area
            '''
            self.area=self.area+(self.row_spacing*self.speed*1/10000) #in ha
            self.area=float(("{0:.4f}").format(self.area))
            self.time_operation=self.time_operation+(1/60)   # im minutes
            self.time_operation=float(("{0:.4f}").format(self.time_operation))
            self.opcap=self.area/(self.time_operation/60) # in ha/h
            self.opcap=float(("{0:.4f}").format(self.opcap))
            '''
            #Logger
            if self.speed>0.1: # save datas if machine is operation
                self.aux_logger=self.aux_logger+1
                if self.aux_logger==5: #save datas each 5 loops
                    self.LoggerFunction()
                    self.aux_logger=0
             
        else: # If button is off
            GPIO.output(pinEnable_Seed,GPIO.LOW)
            GPIO.output(pinEnable_Fert,GPIO.LOW)
            #gpio.EnableSeed("LOW")
            #gpio.EnableFert("LOW")
            self.lb_status.setText("Button OFF")
            self.encoder_timer.stop()
            self.speed,self.pdop,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap="OFF","OFF","OFF","OFF","OFF","OFF","OFF"

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

        data_string=QDateTime.currentDateTime().toString(Qt.ISODate)+","+str(self.lat_atual)+","+str(self.long_atual)+","+str(self.speed)+","+str(self.speed)+","+\
                     str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+","+str(self.area)+","+str(self.time_operation)+","+\
                     str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seedbym)+","+str(self.real_rot_seed)+","+str(self.fertbym)+","+str(self.seed_mode)+","+str(self.fert_mode)

        f=open(self.logfile_name,'a')
        f.write(data_string)
        f.write("\n")
        f.close()
    
    def EncoderFunction(self):
        self.real_rot_seed=operation.SeedSpeed(pinEncA_Seed)
        self.speed=operation.WheelSpeed(pinEncA_Wheel)
 
    def Close(self):
        GPIO.cleanup()
        PWM.cleanup()
        #os.system("sh stop.sh")
        self.control_timer.stop()
        self.encoder_timer.stop()
        self.close() #close software

#Run the app:
if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    ex = Semea()
    ex.show()
    sys.exit(app.exec_())

