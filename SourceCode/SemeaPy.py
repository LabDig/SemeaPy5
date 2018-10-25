#21 de outubro
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import QDateTime,Qt
import math
import time
import serial
# import python files
from seeder_ui import Ui_SEMEA #gui
import operation #calculations
from scipy import stats
import numpy as np
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
from Adafruit_BBIO.Encoder import RotaryEncoder,eQEP0,eQEP2 # 0 == Seed # 2 == Roda 
#
#Button OnOff  (Enable)
pinOnOffButton="P8_16"
GPIO.setup(pinOnOffButton, GPIO.IN)
#UButton p Dynamic
pinUpDyn="P9_15"
GPIO.setup(pinUpDyn, GPIO.IN)
#PWM Seed
pinPWM_Seed="P8_13"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_10"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
#PWM Fertilizer
pinPWM_Fert="P8_19"
PWM.start(pinPWM_Fert,0, 1000.0) #pin, duty,frequencia
pinEnable_Fert="P8_9"
GPIO.setup(pinEnable_Fert, GPIO.OUT)
GPIO.output(pinEnable_Fert,GPIO.LOW)
#Celula de Carga
ADC.setup()
pinLoadCell="P9_33"
#GPS
gps = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13
#3G
sim800l = serial.Serial ("/dev/ttyS1", 4800) # P9_24 P9_26
#
#
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        #global variables
        self.speed,self.pdop,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap,self.time_operation=0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
        self.fertbym,self.seedbym,self.lat_utm,self.long_utm,self.lat,self.long,self.pdop,self.status=0,0,0,0,0,0.0,0,0
        self.row_spacing,self.disk_hole,self.seed_germ,self.logfile_name=0,0,0,""
        self.lat_map_fert,self.long_map_fert,self.map_fertrt,self.lat_utm,self.long_utm=[],[],[],0,0
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[]
        self.seed_mode,self.fert_mode="OFF","OFF"
        self.rot_seed,self.real_rot_seed,self.rot_fert=0.0,0.0,0.0
        self.wgt_voltage_cal=np.zeros(4)
        self.inst_opcap,self.inst_area,self.inst_time=0,0,0
        self.dt_seed_cal,self.dt_fert_cal=0,0
        self.zoom=1
        self.last_pinUpDyn_st=False
        self.change_popseed=False
        self.last_popseed,self.last_fert_rt=0,0
        self.last_wgt=0
        #timers
        self.control_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()
        self.gps_timer=QtCore.QTimer()
        self.log_timer=QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)
        self.gps_timer.timeout.connect(self.GPSFunction)
        self.log_timer.timeout.connect(self.LogFunction)
        self.time_control=1 #in s
        self.control_timer.start(self.time_control*1000) #
        self.encoder_timer.start(10) # Start Encoder Function
        self.gps_timer.start(2500) # Start GPS Function
        self.log_timer.start(5000) # Start Log Function
        #buttons
        #Main
        #exit button
        self.exit.clicked.connect(self.Close)
        self.bt_fit.clicked.connect(self.FitMap)
        self.m_view.clicked.connect(self.ZoomOut)
        self.p_view.clicked.connect(self.ZoomIn)
        #Conf 01
        #button for map
        self.bt_load_seed.clicked.connect(self.LoadSeedFile)
        self.bt_load_fert.clicked.connect(self.LoadFertFile)
        #button plus and minus
        self.m_pop.clicked.connect(self.DecPop)
        self.p_pop.clicked.connect(self.IncPop)
        self.m_germ.clicked.connect(self.DecGer)
        self.p_germ.clicked.connect(self.IncGer)
        self.m_fert.clicked.connect(self.DecFert)
        self.p_fert.clicked.connect(self.IncFert)
        self.m_row.clicked.connect(self.DecRow)
        self.p_row.clicked.connect(self.IncRow)
        #Conf 02
        #button for logfile
        self.bt_define_logfile.clicked.connect(self.DefineLogFile)
        #buton save cal
        self.bt_save_cal.clicked.connect(self.SaveCal)
        self.lb_cal.setText("Tank + 5kg")
        #buton calibrate
        self.bt_calibrate.clicked.connect(self.Calibration)
        #keyboard button
        self.kbd.clicked.connect(lambda:os.system('xvkbd'))
        self.kbd1.clicked.connect(lambda:os.system('xvkbd'))
        #Cal
        #button plus and minus
        self.m_dt_seed.clicked.connect(self.DecSeedCal)
        self.p_dt_seed.clicked.connect(self.IncSeedCal)
        self.m_dt_fert.clicked.connect(self.DecFertCal)
        self.p_dt_fert.clicked.connect(self.IncFertCal)
        self.m_dyn_seed.clicked.connect(self.DecPopDyn)
        self.p_dyn_seed.clicked.connect(self.IncPopDyn)
        self.m_dyn_fert.clicked.connect(self.DecFertDyn)
        self.p_dyn_fert.clicked.connect(self.IncFertDyn)
        #
        #grapghs view configuration
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)
        #read configuration, if the application close
        self.dir=os.path.dirname(os.path.abspath(__file__))  
        self.conffile_name=os.path.join(self.dir,"conf.txt")
        with open(self.conffile_name, "r",encoding='latin-1') as f:
            self.st_chk=(f.readline())
            self.seedfile_name=f.readline()
            self.st_chk_2=(f.readline())
            self.popseed=float(f.readline())
            self.seed_germ=int(f.readline())
            self.st_chk_3=(f.readline())
            self.fertfile_name=f.readline()
            self.st_chk_4=(f.readline())
            self.fert_rt=float(f.readline())
            self.row_spacing=float(f.readline())
            self.st_cb_remote=f.readline()
            self.logfile_name=f.readline()
            self.machineID=f.readline()
            self.fieldID=f.readline()
            self.cal_a=float(f.readline())
            self.cal_b=float(f.readline())
        f.close()
        #set configuration at open
        self.seedfile_name=self.seedfile_name.rstrip()
        self.fertfile_name=self.fertfile_name.rstrip()
        self.logfile_name=self.logfile_name.rstrip()
        self.machineID=self.machineID.rstrip()
        self.fieldID=self.fieldID.rstrip()
        self.cb_seed_map.setCheckState ("True" in self.st_chk )
        self.cb_seed_fix.setCheckState ("True" in self.st_chk_2)
        self.cb_fert_map.setCheckState ("True" in self.st_chk_3)
        self.cb_fert_fix.setCheckState ("True" in self.st_chk_4)
        self.ql_machine_id.setPlainText(str(self.machineID))
        self.ql_field_id.setPlainText(str(self.fieldID))
        self.ql_row_spacing.setPlainText(str(self.row_spacing))
        self.ql_germ.setPlainText(str(self.seed_germ))
        self.ql_set_fert.setPlainText(str(self.fert_rt))
        self.ql_set_pop.setPlainText(str(self.popseed))
        # Reload Maps
        self.cb_seed_map.stateChanged.connect(self.LoadSeedMap)
        if self.cb_seed_map.isChecked():
            try: self.LoadSeedMap()
            except:
                self.lb_status.setText("No Fert Map")
                pass
        else: self.seedfile_name=""
        self.cb_fert_map.stateChanged.connect(self.LoadFertMap)
        if self.cb_fert_map.isChecked():
            try: self.LoadFertMap()
            except:
                self.lb_status.setText("No Fert Map")
                pass
        else: self.fertfile_name="" 
#####
####Functions
####
#Main
    def Close(self): #save configuration and close the software
        with open(self.conffile_name, "w",encoding='latin-1') as f:
            f.write(str(self.cb_seed_map.isChecked())+"\n")
            f.write(self.seedfile_name+"\n")
            f.write(str(self.cb_seed_fix.isChecked())+"\n")
            f.write(str(self.popseed)+"\n")
            f.write(str(self.seed_germ)+"\n")
            f.write(str(self.cb_fert_map.isChecked())+"\n")    
            f.write(self.fertfile_name+"\n")
            f.write(str(self.cb_fert_fix.isChecked())+"\n")
            f.write(str(self.fert_rt)+"\n")
            f.write(str(self.row_spacing)+"\n")
            f.write(str(self.cb_remote.isChecked())+"\n")
            f.write(self.logfile_name+"\n")
            f.write(self.machineID+"\n")
            f.write(self.fieldID+"\n")
            f.write(str(self.cal_a)+"\n")
            f.write(str(self.cal_b)+"\n")
        f.close()
        GPIO.cleanup()
        PWM.cleanup()
        self.control_timer.stop()
        self.encoder_timer.stop()
        self.gps_timer.stop()
        self.log_timer.stop()
        self.close()

    def FitMap(self): self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio) # fit the map
    def ZoomOut(self):
        self.zoom=self.zoom/2
        self.gv.scale(self.zoom,self.zoom)
    def ZoomIn(self):
        self.zoom=self.zoom*2
        self.gv.scale(self.zoom,self.zoom)
###
##Config 01
###
    def LoadSeedMap(self):
        if self.cb_seed_map.isChecked():
            if ".txt" in self.seedfile_name:
                content=None # clear variable for security
                with open(self.seedfile_name, "r",encoding='latin-1') as f: content = f.read().splitlines()
                f.close()
                self.lat_map_seed,self.long_map_seed,self.pop_map_seed=operation.ReadMapFile(content)
                for i in range(len(self.lat_map_seed)):self.scene.addRect(self.lat_map_seed[i],self.long_map_seed[i],1,1,self.Rpen,self.Rbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                del content
            else:
                print ("shape")

    def LoadFertMap(self):
        if self.cb_fert_map.isChecked():
            if ".txt" in self.fertfile_name:
                content=None # clear variable for security
                with open(self.fertfile_name, "r",encoding='latin-1') as f:  content = f.read().splitlines()
                f.close()
                self.lat_map_fert,self.long_map_fert,self.map_fertrt=operation.ReadMapFile(content)
                del content
                for i in range(len(self.lat_map_fert)):self.scene.addRect(self.lat_map_fert[i],self.long_map_fert[i],1,1,self.Bpen,self.Bbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            else:
                print ("shape")
                
    def DecPop(self):
        self.popseed=self.popseed-2500
        self.ql_set_pop.setPlainText(str(self.popseed))
    def IncPop(self):
        self.popseed=self.popseed+2500
        self.ql_set_pop.setPlainText(str(self.popseed))
    def DecGer(self):
        self.seed_germ=self.seed_germ-1
        self.ql_germ.setPlainText(str(self.seed_germ))
    def IncGer(self):
        self.seed_germ=self.seed_germ+1
        self.ql_germ.setPlainText(str(self.seed_germ))
    def DecFert(self):
        self.fert_rt=self.fert_rt-10
        self.ql_set_fert.setPlainText(str(self.fert_rt))
    def IncFert(self):
        self.fert_rt=self.fert_rt+10
        self.ql_set_fert.setPlainText(str(self.fert_rt))
    def DecRow(self):
        self.row_spacing=self.row_spacing-0.05
        self.ql_row_spacing.setPlainText(str(self.row_spacing))
    def IncRow(self):
        self.row_spacing=self.row_spacing+0.05
        self.ql_row_spacing.setPlainText(str(self.row_spacing))
    def LoadSeedFile(self): self.seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File",".","*.txt")[0]
    def LoadFertFile(self): self.fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File",".","*.txt")[0]
###
##Config 02
###    
    def DefineLogFile(self):
        self.logfile_name=QFileDialog.getSaveFileName(self,"Define Logfile","","*.txt")[0]
        f=open(self.logfile_name,'w')
        f.write("Data/Hora,MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),PDOP,GPS Status,PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),\
Instantanea OpCap(ha/h),Area(ha),Row Spacing(m),Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode\n")
        f.close()

    def SaveCal(self):
        sum_wgt=0
        for i in range(0,20):
            value=round(1.8*ADC.read(pinLoadCell),2)
            sum_wgt=sum_wgt+value
            time.sleep(0.5)
        self.wgt_voltage_cal[self.list_cal_wgt.currentIndex()]=round(sum_wgt/10,5)
        self.qd_voltage_cal.setPlainText(str(round(sum_wgt/10,5)))

    def Calibration(self):
            mass = np.array([6.4,11.4,16.4,21.4])
            try: self.cal_a,self.cal_b,r_value,p_value,std_error=stats.linregress(self.wgt_voltage_cal,mass)
            except:pass
            self.cal_a=round(self.cal_a,4)
            self.cal_b=round(self.cal_b,4)
###
##Cal Only
###
    def DecSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal-10
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>99.99:self.dt_seed_cal=99.99
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
            
        
    def IncSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal+10
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>99.9:self.dt_seed_cal=99.99
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
 
    def DecFertCal(self):
        if self.cb_speed_fert.isChecked():
            self.dt_fert_cal=self.dt_fert_cal-10
            self.ql_dt_fert.setPlainText(str(self.dt_fert_cal))
            PWM.set_duty_cycle(pinPWM_Fert,self.dt_seed_cal)
            GPIO.output(pinEnable_Fert,GPIO.HIGH)

    def IncFertCal(self):
        if self.cb_speed_fert.isChecked():
            self.dt_fert_cal=self.dt_fert_cal-10
            self.ql_dt_fert.setPlainText(str(self.dt_fert_cal))
            PWM.set_duty_cycle(pinPWM_Fert,self.dt_fert_cal)
            GPIO.output(pinEnable_Fert,GPIO.HIGH)

    def DecPopDyn(self):
        if self.cb_dyn_seed.isChecked():
            self.popseed=self.popseed-10000
            self.ql_seed_dyn.setPlainText(str(self.popseed))

    def IncPopDyn(self):
        if self.cb_dyn_seed.isChecked():
            self.popseed=self.popseed+10000
            self.ql_seed_dyn.setPlainText(str(self.popseed))

    def DecFertDyn(self):
        if self.cb_dyn_fert.isChecked():
            self.fert_rt=self.fert_rt-10000
            self.ql_fert_dyn.setPlainText(str(self.fert_rt))

    def IncFertDyn(self):
        if self.cb_dyn_fert.isChecked():
            self.fert_rt=self.fert_rt+10000
            self.ql_fert_dyn.setPlainText(str(self.fert_rt))
            
            
##TimeOutFunctions
    def GPSFunction(self):
        if GPIO.input(pinOnOffButton):
            try:
                self.lat_utm,self.long_utm,self.lat,self.long,self.pdop,self.status=operation.ReadGPS(gps.readline())
            except:
                self.ql_remote_status.setPlainText("GPS Error")
                pass
                

    def EncoderFunction(self):
        if GPIO.input(pinOnOffButton):
            self.real_rot_seed=operation.SeedSpeed(self.change_popseed)
            self.speed=operation.WheelSpeed()

    def LogFunction(self):
        if GPIO.input(pinOnOffButton):
            string="Saving in: "+str(self.logfile_name)
            self.ql_logfile.setPlainText(string)

            data_string=QDateTime.currentDateTime().toString(Qt.ISODate)+","+str(self.lat_utm)+","+str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+\
str(self.speed)+","+str(self.pdop)+","+str(self.status)+","+ str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+","+str(self.area)\
+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+","+str(self.fertbym)+"," +str(self.seed_mode)+","+str(self.fert_mode)
            f=open(self.logfile_name,'a')
            f.write(data_string)
            f.write("\n")
            f.close()

            #Remote
            if self.cb_remote.isChecked():
                self.ql_remote_status.setPlainText("Enable")
                try:
                    sim800l.write ('AT+HTTPPARA="URL","http://'+'\r\n');
                    sim800l.write('AT+HTTPACTION=0'+'\r\n');
                    sim800l.write('AT+HTTPREAD'+'\r\n');
                except:
                    self.ql_remote_status.setPlainText("3G Error")
                    pass
            else:
                self.ql_remote_status.setPlainText("Disable")
        else:
            self.ql_logfile.setPlainText("Not Saving")
###
###
    def ControlFunction(self):  

        if GPIO.input(pinOnOffButton):
            self.disk_hole=int(self.list_holes.currentText()) #Read Hole Disk
            self.fert_wgt=operation.ReadWeight(pinLoadCell,self.cal_a,self.cal_b) #Read Fert Weight
            #Read Simulated Speed (By test only)
            if self.cb_motion_simulate.isChecked(): 
                self.speed=float(self.ql_sim_speed.toPlainText()) 
            ###
            ###Seeder Distributor###
            ###
            if self.cb_seed_map.isChecked() and len(self.lat_map_seed)>1 and self.status=='A': #Variable Seed base on map and have map and have signal gps
                self.seed_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.popseed,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
                
            elif self.cb_seed_fix.isChecked(): #Fix seed distribuition rate
                self.seed_mode="FIX"
                self.scene.clear()
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            
            elif self.cb_seed_fix.isChecked() and self.cb_seed_map.isChecked(): #if two mode is checked
                self.cb_seed_fix.setCheckState (False)
                self.cb_seed_map.setCheckState (False)
 
            else:  
                self.seed_mode="OFF"
                self.popseed=0
            #check if population change, for use in speed mean filter
            if self.popseed!=self.last_popseed:
                self.change_popseed=True
            else :self.change_popseed=True
            self.last_popseed=self.popseed
            
            #Calcute and Control Speed Motor
            if self.cb_speed_cal.isChecked() is False: #if test function is not active
                self.rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                operation.ControlSpeedSeed(pinEnable_Seed,pinPWM_Seed,self.rot_seed,self.real_rot_seed)
            #
            ####Fertilizer Distribution###
            #
            if self.cb_fert_map.isChecked() and len(self.lat_map_fert)>1 and self.status=='A': #Variable Fert base on map and have map and have signal gps
                self.fert_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.fert_rt,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_fert,self.long_map_fert,self.map_fertrt)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
                
            elif self.cb_fert_fix.isChecked(): #Fix seed distribuition rate
                self.fert_mode="FIX"
                self.scene.clear()
                 #Write value
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            
            elif self.cb_fert_fix.isChecked() and self.cb_fert_map.isChecked(): #if two mode is checked
                self.cb_fert_fix.setCheckState (False)
                self.cb_fert_map.setCheckState (False)

            else:
                self.fert_mode="OFF"
                self.fert_rt=0

            #check if population change
            if self.fert_rt!=self.last_fert_rt:
                self.change_fertrt=True
            else :self.change_fertrt=False
            self.last_fert_rt=self.fert_rt 
            
            #Calcute and Control Speed Motor
            self.fertbym,self.fertbys=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
            operation.ControlSpeedFert(pinEnable_Fert,pinPWM_Fert,self.fertbys,self.fert_wgt,self.last_wgt,self.speed,self.time_control,self.last_fert_rt)

            #update the fertilizer wgt
            self.last_wgt=self.fert_wgt
            #  
            #calculate operational capability and area
            self.inst_area=round(self.row_spacing*self.speed*self.time_control/10000.0,2)
            self.area=round(self.area+self.inst_area,1) #in ha
            self.inst_time=round(self.time_control/3600.0,5)
            self.time_operation=round(self.time_operation+self.inst_time,5)   # in h
            self.opcap=round(self.area/(self.time_operation),2) # in ha/h
            self.inst_opcap=self.inst_area/self.inst_time
            # Update LineEdit in main tab
            self.ql_speed.setPlainText(str(self.speed))
            self.ql_pdop.setPlainText(str(self.pdop))
            self.ql_seed.setPlainText(str(self.popseed))
            self.ql_fert_rt.setPlainText(str(self.fert_rt))
            self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
            self.ql_area.setPlainText(str(self.area))
            self.ql_opcap.setPlainText("M:"+str(self.opcap)+"..I:"+str(self.inst_opcap))
            self.lb_status.setText("Habilitado. Fert: "+self.fert_mode+" Seed:"+self.seed_mode)
            #in Cal Tab
            self.ql_speed_seed.setPlainText(str(self.real_rot_seed))
            
            # Incread Population and Fert Ratio by button for dynamic test
            if GPIO.input(pinUpDyn) and self.last_pinUpDyn_st is False: # if bottun is pressed
                self.IncPopDyn()
                self.IncFertDyn()
            self.last_pinUpDyn_st=GPIO.input(pinUpDyn)
        #
        else: # If button is off
            GPIO.output(pinEnable_Seed,GPIO.LOW)
            GPIO.output(pinEnable_Fert,GPIO.LOW)
            self.lb_status.setText("Desabilitado")
#Run the app:
if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    ex = Semea()
    ex.show()
    sys.exit(app.exec_())

