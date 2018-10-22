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
import utm
from time import time
import serial
# import python files
from seeder_ui import Ui_SEMEA #gui
import operation #calculations
from scipy import stats
import numpy as np
'''
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
from Adafruit_BBIO.Encoder import RotaryEncoder,eQEP0,eQEP2 # 0 == Seed # 2 == Roda 
'''
#OnOff
pinOnOffButton="P8_16"
#GPIO.setup(pinOnOffButton, GPIO.IN) 
#PWM Seed
pinPWM_Seed="P8_13"
#PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_10"
#GPIO.setup(pinEnable_Seed, GPIO.OUT)
#GPIO.output(pinEnable_Seed,GPIO.LOW)
#PWM Fertilizer
pinPWM_Fert="P8_19"
#PWM.start(pinPWM_Fert,0, 1000.0) #pin, duty,frequencia
pinEnable_Fert="P8_9"
#GPIO.setup(pinEnable_Fert, GPIO.OUT)
#GPIO.output(pinEnable_Fert,GPIO.LOW)
#Celula de Carga
#ADC.setup()
pinLoadCell="P9_33"
#GPS
#gps = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13
#3G
#sim800l = serial.Serial ("/dev/ttyS1", 4800) # P9_24 P9_26
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
        self.lat_map_fert,self.long_map_fert,self.map_fert,self.lat_utm,self.long_utm=[],[],[],0,0
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[]
        self.seed_mode,self.fert_mode="OFF","OFF"
        self.rot_seed,self.real_rot_seed,self.rot_fert=0.0,0.0,0.0
        self.wgt_voltage_cal=np.zeros(4)
        #exit button
        self.exit.clicked.connect(self.Close)
        #timers
        self.control_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()
        self.gps_timer=QtCore.QTimer()
        self.log_timer=QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)
        self.gps_timer.timeout.connect(self.GPSFunction)
        self.log_timer.timeout.connect(self.LogFunction)
        self.time_control=0.5 #in s
        self.control_timer.start(self.time_control*1000) #
        #button for logfile
        self.bt_define_logfile.clicked.connect(self.DefineLogFile)
        #button for map
        self.bt_load_seed.clicked.connect(self.LoadSeedFile)
        self.bt_load_fert.clicked.connect(self.LoadFertFile)
        #buton save cal
        self.bt_save_cal.clicked.connect(self.SaveCal)
        self.lb_cal.setText("Tank + 5kg")
        #buton calibrate
        self.bt_calibrate.clicked.connect(self.Calibration)
        #grapghs view
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)
        #button plus and minus
        self.m_pop.clicked.connect(self.DecPop)
        self.p_pop.clicked.connect(self.IncPop)
        self.m_germ.clicked.connect(self.DecGer)
        self.p_germ.clicked.connect(self.IncGer)
        self.m_fert.clicked.connect(self.DecFert)
        self.p_fert.clicked.connect(self.IncFert)
        self.m_row.clicked.connect(self.DecRow)
        self.p_row.clicked.connect(self.IncRow)
        #keyboard button
        self.kbd.clicked.connect(lambda:os.system('florence'))
        self.kbd1.clicked.connect(lambda:os.system('florence'))
        #read file configuration
        with open("conf.txt", "r",encoding='latin-1') as f:
            self.st_chk=(f.readline())
            self.seedfile_name=f.readline()
            self.st_chk_2=(f.readline())
            self.popseed=int(f.readline())
            self.seed_germ=int(f.readline())
            self.st_chk_3=(f.readline())
            self.fertfile_name=f.readline()
            self.st_chk_4=(f.readline())
            self.fert_rt=int(f.readline())
            self.row_spacing=float(f.readline())
            self.st_cb_remote=f.readline()
            self.logfile_name=f.readline()
            self.machineID=f.readline()
            self.fieldID=f.readline()
            self.cal_a=float(f.readline())
            self.cal_b=float(f.readline())
        f.close()
        #set config
        self.seedfile_name=self.seedfile_name.rstrip()
        self.fertfile_name=self.fertfile_name.rstrip()
        self.logfile_name=self.logfile_name.rstrip()
        self.machineID=self.machineID.rstrip()
        self.fieldID=self.fieldID.rstrip()
        self.checkBox.setCheckState ("True" in self.st_chk )
        self.checkBox_2.setCheckState ("True" in self.st_chk_2)
        self.checkBox_3.setCheckState ("True" in self.st_chk_3)
        self.checkBox_4.setCheckState ("True" in self.st_chk_4)
        self.ql_machine_id.setPlainText(str(self.machineID))
        self.ql_field_id.setPlainText(str(self.fieldID))

        self.checkBox.stateChanged.connect(self.LoadSeedMap)
        if self.checkBox.isChecked(): self.LoadSeedMap
        self.checkBox_2.stateChanged.connect(self.LoadFertMap)
        if self.checkBox_3.isChecked():self.LoadFertMap

    def LoadSeedMap(self):
        if self.checkBox.isChecked():
            content=None # clear variable for security
            with open(self.seedfile_name, "r",encoding='latin-1') as f: content = f.read().splitlines()
            f.close()
            self.lat_map_seed,self.long_map_seed,self.pop_map_seed=operation.ReadMapFile(content)
            for i in range(len(self.lat_map_seed)):self.scene.addRect(self.lat_map_seed[i],self.long_map_seed[i],1,1,self.Rpen,self.Rbrush)
            self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            del content

    def LoadFertMap(self):
        if self.checkBox_2.isChecked():
            content=None # clear variable for security
            with open(self.fertfile_name, "r",encoding='latin-1') as f:  content = f.read().splitlines()
            f.close()
            self.lat_map_fert,self.long_map_fert,self.pop_map_fert=operation.ReadMapFile(content)
            del content
            for i in range(len(self.lat_map_fert)):self.scene.addRect(self.lat_map_fert[i],self.long_map_fert[i],1,1,self.Bpen,self.Bbrush)
            self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            
    #
    def DecPop(self):
        self.popseed=self.popseed-2500
    def IncPop(self):
        self.popseed=self.popseed+2500
    def DecGer(self):
        self.seed_germ=self.seed_germ-1
    def IncGer(self):
        self.seed_germ=self.seed_germ+1
    def DecFert(self):
        self.fert_rt=self.fert_rt-10
    def IncFert(self):
        self.fert_rt=self.fert_rt+10
    def DecRow(self):
        self.row_spacing=self.row_spacing-0.05
    def IncRow(self):
        self.row_spacing=self.row_spacing+0.05
    
    def DefineLogFile(self):
        self.logfile_name=QFileDialog.getSaveFileName(self,"Define Logfile","","*.txt")[0]
        f=open(self.logfile_name,'w')
        f.write("Data/Hora,MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),PDOP,GPS Status, \
          PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),  Instantanea OpCap(ha/h),Area(ha),Row Spacing(m), \
          Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode")
        f.write("\n")
        f.close()
    #
    def LoadSeedFile(self):
        self.seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File",".","*.txt")[0]

    def LoadFertFile(self):
        self.fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File",".","*.txt")[0]
       
    # short funcions
    def GPSFunction(self):
        #self.lat_utm,self.long_utm,self.lat,self.long,self.pdop,self.status=operation.ReadGPS(gps.readline())
        print ("Read GPS")
    #
    def SaveCal(self):
        sum_wgt=0
        for i in range(0,10):
            value=round(1.8*ADC.read(pinLoadCell),2)
            sum_wgt=sum_wgt+value
            self.qd_voltage_signal.setText(str(value))
            time.sleep(0.5)
        self.wgt_voltage_cal[self.list_cal_wgt.currentIndex()]=sum_wgt/10
    #
    def Calibration(self):
            mass = np.array([6.4,11.4,16.4,21.4]) #ph
            self.cal_a,self.cal_b,r_value,p_value,std_error=stats.linregress(self.wgt_voltage,mass)
            self.cal_a=round(self.cal_a,4)
            self.cal_b=round(self.cal_b,4)
    #
    def ControlFunction(self):  #Main Loop of Software
    
    #
        if True: #GPIO.input(pinOnOffButton):
            self.lb_status.setText("Habilitado")
            self.encoder_timer.start(10) # Start Encoder Function
            self.gps_timer.start(2500) # Start GPS Function
            self.log_timer.start(5000) # Start GPS Function
            self.ql_germ.setPlainText(str(self.seed_germ)) # Write seed_germination
            self.disk_hole=int(self.list_holes.currentText()) #Read Hole Disk
            self.ql_row_spacing.setPlainText(str(self.row_spacing)) #Write Spacing
            self.fert_wgt=operation.ReadWeight(pinLoadCell,self.cal_a,self.cal_b) #Read Fert Weight
            #
            self.machideID=self.ql_machine_id.toPlainText()
            self.fieldID=self.ql_field_id.toPlainText()
            #    
            #Read Simulated Speed (By test only)
            if self.cb_motion_simulate.isChecked(): 
                self.speed=float(self.ql_sim_speed.toPlainText()) 
            ###
            ###Seeder Distributor###
            ###
            if self.checkBox.isChecked() and len(self.lat_map_seed)>1 and self.status=='A': #Variable Seed base on map and have map and have signal gps
                self.seed_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.popseed,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)

                
            elif self.checkBox_2.isChecked(): #Fix seed distribuition rate
                self.seed_mode="FIX"
                self.ql_set_pop.setPlainText(str(self.popseed)) #Write value
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            
            elif self.checkBox.isChecked() and self.checkBox_2.isChecked(): #if two mode is checked
                self.checkBox.setCheckState (False)

            else:
                self.seed_mode="OFF"
                self.popseed=0
                #GPIO.output(pinEnable_Seed,GPIO.LOW)
            #
            self.rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
            operation.ControlSpeedSeed(pinEnable_Seed,pinPWM_Seed,self.rot_seed,self.real_rot_seed)
            #
            ####Fertilizer Distribution###
            #
            if self.checkBox_3.isChecked() and len(self.lat_map_fert)>1 and self.status=='A': #Variable Fert base on map and have map and have signal gps
                self.fert_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.fertrt,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_fert,self.long_map_fert,self.pop_map_fert)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
                
            elif self.checkBox_4.isChecked(): #Fix seed distribuition rate
                self.fert_mode="FIX"
                self.ql_set_fert.setPlainText(str(self.fert_rt)) #Write value
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            
            elif self.checkBox_3.isChecked() and self.checkBox_4.isChecked(): #if two mode is checked
                self.checkBox_3.setCheckState (False)

            else:
                self.fert_mode="OFF"
                self.fert_rt=0
                #GPIO.output(pinEnable_Fert,GPIO.LOW)
            #
            self.rot_fert,self.fertbym=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
            operation.ControlSpeedFert(pinEnable_Fert,pinPWM_Fert,self.rot_fert)
            #  
            #calculate operational capability and area
            self.area=round(self.area+(self.row_spacing*self.speed*self.time_control/10000.0),1) #in ha
            self.time_operation=round(self.time_operation+(self.time_control/3600.0),5)   # in h
            self.opcap=round(self.area/(self.time_operation),2) # in ha/h

        else: # If button is off
            #GPIO.output(pinEnable_Seed,GPIO.LOW)
            #GPIO.output(pinEnable_Fert,GPIO.LOW)
            self.lb_status.setText("Desabilitado")
            self.encoder_timer.stop()
            self.gps_timer.stop()
            self.log_timer.stop()
            self.speed,self.pdop,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap=0,0,0,0,0,0,0

        # Update LineEdit in main tab
        self.ql_speed.setPlainText(str(self.speed))
        self.ql_pdop.setPlainText(str(self.pdop))
        self.ql_seed.setPlainText(str(self.popseed))
        self.ql_fert_rt.setPlainText(str(self.fert_rt))
        self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
        self.ql_area.setPlainText(str(self.area))
        self.ql_opcap.setPlainText(str(self.opcap))
        
    def LogFunction(self):
        string="Saving in: "+str(self.logfile_name)
        self.ql_logfile.setPlainText(string)
        if self.cb_remote.isChecked():
            self.ql_remote_status.setPlainText("Enable")
            sim800l.write ('AT+HTTPPARA="URL","http://ecosolucoes.net/andre/salvaarduino.php?ph="+String(ph)+"&turb="+String(turbidez)+"&k_comp="+\
                         String(!digitalRead(bt_k_comp))+"&rt_comp="+String(!digitalRead(bt_rt_comp))+"&k_filt="+String(!digitalRead(bt_k_filt))+"&k_elev2="\
                         +String(!digitalRead(bt_k_elev2))+"&k_elev1="+String(!digitalRead(bt_k_elev1))+"&k_lamp="+String(!digitalRead(bt_k_lamp))+"&k_agit="+\
                         String(!digitalRead(bt_k_agit))+"&rt_agit="+String(!digitalRead(bt_rt_agit))'+'\r\n');
            sim800l.write('AT+HTTPACTION=0'+'\r\n');
            sim800l.write('AT+HTTPREAD'+'\r\n');
        else:
            self.ql_remote_status.setPlainText("Disable")

        #Data/Hora,MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),PDOP,GPS Status, \
        #PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),  Instantanea OpCap(ha/h),Area(ha),Row Spacing(m), \
        #Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode

        data_string=QDateTime.currentDateTime().toString(Qt.ISODate)+","+str(self.lat_utm)+","+str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+\
            str(self.speed)+","+str(self.pdop)+","+str(self.status)+","+ str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+\
            ","+str(self.area)+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+","+str(self.fertbym)+","\
            +str(self.seed_mode)+","+str(self.fert_mode)
        f=open(self.logfile_name,'a')
        f.write(data_string)
        f.write("\n")
        f.close()
    
    def EncoderFunction(self):
        self.real_rot_seed=operation.SeedSpeed()
        self.speed=operation.WheelSpeed()
 
    def Close(self):
        #save configuration
        with open("conf.txt", "w",encoding='latin-1') as f:
            f.write(str(self.checkBox.isChecked())+"\n")
            f.write(self.seedfile_name+"\n")
            f.write(str(self.checkBox_2.isChecked())+"\n")
            f.write(str(self.popseed)+"\n")
            f.write(str(self.seed_germ)+"\n")
            f.write(str(self.checkBox_3.isChecked())+"\n")    
            f.write(self.fertfile_name+"\n")
            f.write(str(self.checkBox_4.isChecked())+"\n")
            f.write(str(self.fert_rt)+"\n")
            f.write(str(self.row_spacing)+"\n")
            f.write(str(self.cb_remote.isChecked())+"\n")
            f.write(self.logfile_name+"\n")
            f.write(self.machineID+"\n")
            f.write(self.fieldID+"\n")
            f.write(str(self.cal_a)+"\n")
            f.write(str(self.cal_b)+"\n")
        f.close()
        #GPIO.cleanup()
        #PWM.cleanup()
        self.control_timer.stop()
        self.encoder_timer.stop()
        self.gps_timer.stop()
        self.log_timer.stop()
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

