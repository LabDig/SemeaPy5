#13 de janeiro
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5 import QtGui,QtCore
import time
from scipy import stats
import numpy as np
# import python files
from seeder_ui import Ui_SEMEA #gui
import operation #calculations
import serial
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
#Pins Configuration
#Button OnOff  (Enable)
pinOnOffButton="P8_16"
GPIO.setup(pinOnOffButton, GPIO.IN)
#UButton p Dynamic
pinUpDyn="P9_12"
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
#LoadCell
ADC.setup()
pinLoadCell="P9_33"
#GPS
gps = serial.Serial ("/dev/ttyS4", 9600) # P9_11 P9_13
#3G Monitoring 
sim800l = serial.Serial("/dev/ttyS1", baudrate = 9600, timeout = 0.1) # P9_24 P9_26
sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
time.sleep(0.1)
sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
time.sleep(0.1)
sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
time.sleep(0.1)
sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
time.sleep(0.1)
sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
time.sleep(0.1)
sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
time.sleep(0.1)
#
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        #global variables declaration
        self.speed,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap,self.time_operation=0.0,'',0.0,0.0,0.0,0.0,0.0 #operation variables
        self.inst_opcap,self.inst_area,self.inst_time=0,0,0 #operation variables
        self.hora,self.data,self.nmea,self.lat_utm,self.long_utm,self.lat,self.long,self.pdop,self.status='','','',0,0,0,0.0,0,0 #gps variables
        self.row_spacing,self.disk_hole,self.seed_germ,self.logfile_name=0,0,0,"" #operation variables
        self.lat_map_fert,self.long_map_fert,self.map_fertrt=[],[],[] #map  fert
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[] #map seed
        self.seed_mode,self.fert_mode,self.fertbym,self.seedbym="OFF","OFF",0,0, #operation modes
        self.rot_seed,self.real_rot_seed=0.0,0.0 #seed speed
        self.wgt_voltage_cal=np.zeros(4) #calibrate load cell
        self.dt_seed_cal,self.dt_fert_cal=0,0 #variables for test 
        self.change_popseed=False #variables for dynamic test and check if population changet
        self.last_popseed,self.last_fert_rt=0,0 #for ckeck if population and fert ratio change
        self.last_wgt,self.dt_seed,self.last_dt_seed_cal=0,-1,-1  #for ckeck if population and fert ratio change
        self.n_machine_id,self.n_field_id=1,1 #number auxiliar for setting machine and field id
        #timers configuration
        self.control_timer = QtCore.QTimer()
        self.encoder_timer = QtCore.QTimer()
        self.gps_timer=QtCore.QTimer()
        self.log_timer=QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.encoder_timer.timeout.connect(self.EncoderFunction)
        self.gps_timer.timeout.connect(self.GPSFunction)
        self.log_timer.timeout.connect(self.LogFunction)
        self.time_control=1.0 #in s
        self.control_timer.start(self.time_control*1000) #Start Control Function
        self.encoder_timer.start(25) # Start Encoder Function
        self.gps_timer.start(2500) # Start GPS Function
        self.log_timer.start(10000) # Start Log Function
        #GUI Buttons Configuration
        #Main
        self.exit.clicked.connect(self.Close)  #exit button
        self.bt_fit.clicked.connect(self.FitMap) #fit view button
        self.m_view.clicked.connect(self.ZoomOut) # zoom out button
        self.p_view.clicked.connect(self.ZoomIn) #zoom in button
        #grapghs view configuration
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)
        self.zoom=1
        #Setup Operation
        self.bt_load_seed.clicked.connect(self.LoadSeedFile) # load seed map
        self.bt_load_fert.clicked.connect(self.LoadFertFile) # load fert map
        self.m_pop.clicked.connect(self.DecPop)  #buttons plus and minus values for variavel operations
        self.p_pop.clicked.connect(self.IncPop)
        self.m_germ.clicked.connect(self.DecGer)
        self.p_germ.clicked.connect(self.IncGer)
        self.m_fert.clicked.connect(self.DecFert)
        self.p_fert.clicked.connect(self.IncFert)
        self.m_row.clicked.connect(self.DecRow)
        self.p_row.clicked.connect(self.IncRow)
        #Setupt Monitoring and Others
        self.bt_st_id.clicked.connect(self.DefineID) # Define Id's for machine and field
        self.bt_save_cal.clicked.connect(self.SaveCal) # Savel Caliration Point
        self.bt_calibrate.clicked.connect(self.Calibration) #buton calibrate
        self.m_machine_id.clicked.connect(self.DecMacID)  #buttons plus and minus for set 
        self.p_machine_id.clicked.connect(self.IncMacID) # name of Field and Machine
        self.m_field_id.clicked.connect(self.DecFilID)
        self.p_field_id.clicked.connect(self.IncFilID)
        #For test Tab
        self.m_dt_seed.clicked.connect(self.DecSeedCal)  #button plus and minus
        self.p_dt_seed.clicked.connect(self.IncSeedCal)  # values for variavel operations
        self.m_dt_fert.clicked.connect(self.DecFertCal) 
        self.p_dt_fert.clicked.connect(self.IncFertCal)
        self.m_dyn_seed.clicked.connect(self.DecPopDyn)
        self.p_dyn_seed.clicked.connect(self.IncPopDyn)
        self.m_dyn_fert.clicked.connect(self.DecFertDyn)
        self.p_dyn_fert.clicked.connect(self.IncFertDyn)
        #
        #Calcule calibration equation for seed motor
        rot = np.array([0.25,0.37,0.49,0.63,0.75,0.87,0.98]) # angular speed meansured
        duty=np.array([40.0,50.0,60.0,70.0,80.0,90.0,100.0]) # duty cicle for PWM
        self.cal_a_seed,self.cal_b_seed,r_value,p_value,std_error=stats.linregress(rot,duty) # x (rot),y (duty) #duty =a*rot+b
        #Open the software with configuration of last use
        self.dir=os.path.dirname(os.path.abspath(__file__))  
        self.conffile_name=os.path.join(self.dir,"conf.txt")
        with open(self.conffile_name, "r",encoding='latin-1') as f:
            self.st_chk=(f.readline()) #map seed mode
            self.seedfile_name=f.readline()
            self.st_chk_2=(f.readline()) #fix seed mode
            self.popseed=float(f.readline())
            self.seed_germ=int(f.readline())
            self.st_chk_3=(f.readline()) #map fert mode
            self.fertfile_name=f.readline()
            self.st_chk_4=(f.readline()) #fix map mode
            self.fert_rt=float(f.readline())
            self.row_spacing=float(f.readline())
            self.st_cb_remote=f.readline()
            self.logfile_name=f.readline() #log file name
            self.machineID=f.readline() 
            self.fieldID=f.readline()
            self.cal_a=float(f.readline()) #calibration for load cell
            self.cal_b=float(f.readline()) #calibration for load cell
            self.area=float(f.readline()) #
            self.time_operation=float(f.readline())
        f.close()
        #Setup the operation, with configuration of last use
        self.seedfile_name=self.seedfile_name.rstrip()
        self.fertfile_name=self.fertfile_name.rstrip()
        self.logfile_name=self.logfile_name.rstrip()
        self.machineID=self.machineID.rstrip()
        self.fieldID=self.fieldID.rstrip()
        self.cb_seed_map.setCheckState ("True" in self.st_chk )
        self.cb_seed_fix.setCheckState ("True" in self.st_chk_2)
        self.cb_fert_map.setCheckState ("True" in self.st_chk_3)
        self.cb_fert_fix.setCheckState ("True" in self.st_chk_4)
        self.cb_remote.setCheckState ("True" in self.st_cb_remote)
        self.ql_machine_id.setPlainText(str(self.machineID))
        self.ql_field_id.setPlainText(str(self.fieldID))
        self.ql_row_spacing.setPlainText(str(self.row_spacing))
        self.ql_germ.setPlainText(str(self.seed_germ))
        self.ql_set_fert.setPlainText(str(self.fert_rt))
        self.ql_set_pop.setPlainText(str(self.popseed))
        # load and Reload the Fert and Seed Map, when map mode is active
        self.cb_seed_map.stateChanged.connect(self.LoadSeedMap)
        if self.cb_seed_map.isChecked():
            self.LoadSeedMap()
        else: self.seedfile_name=""
        self.cb_fert_map.stateChanged.connect(self.LoadFertMap)
        if self.cb_fert_map.isChecked():
           self.LoadFertMap()
        else: self.fertfile_name=""
        #Event Detect Dynamic Test Button
        GPIO.add_event_detect(channel,GPIO.RISING,callback=self.IncPopFert,bouncetime=100)
####
#Functions
# Im main Tab
    def Close(self): #save configuration inf file, clear GPIO, stop timer and close the software
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
            f.write(str(self.area)+"\n")
            f.write(str(self.time_operation)+"\n")
        f.close()
        GPIO.cleanup()
        PWM.cleanup()
        self.control_timer.stop()
        self.encoder_timer.stop()
        self.gps_timer.stop()
        self.log_timer.stop()
        self.close()
    def FitMap(self): self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio) # fit the map
    def ZoomOut(self): #zoom out
        self.zoom=self.zoom/2
        self.gv.scale(self.zoom,self.zoom)
    def ZoomIn(self): #zoom in
        self.zoom=self.zoom*2
        self.gv.scale(self.zoom,self.zoom)
#In Setup Operation
     # Setting the map file name
    def LoadSeedFile(self): self.seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File",".","*.txt")[0]
    def LoadFertFile(self): self.fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File",".","*.txt")[0]
    def LoadSeedMap(self): #Load Seed Map of file and show in Graphics View
        if self.cb_seed_map.isChecked():
            if ".txt" in self.seedfile_name:
                content=None # clear variable for security
                with open(self.seedfile_name, "r",encoding='latin-1') as f: content = f.read().splitlines()
                f.close()
                self.lat_map_seed,self.long_map_seed,self.pop_map_seed=operation.ReadMapFile(content)
                for i in range(len(self.lat_map_seed)):self.scene.addRect(self.lat_map_seed[i],self.long_map_seed[i],1,1,self.Rpen,self.Rbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                del content #clear memory
            else: self.lb_status.setText("No Seed Map")
    def LoadFertMap(self): #Load Fert Map of file and show in Graphics View
        if self.cb_fert_map.isChecked():
            if ".txt" in self.fertfile_name:
                content=None # clear variable for security
                with open(self.fertfile_name, "r",encoding='latin-1') as f:  content = f.read().splitlines()
                f.close()
                self.lat_map_fert,self.long_map_fert,self.map_fertrt=operation.ReadMapFile(content)
                for i in range(len(self.lat_map_fert)):self.scene.addRect(self.lat_map_fert[i],self.long_map_fert[i],1,1,self.Bpen,self.Bbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
                del content #clear memory
        else: self.lb_status.setText("No Fert Map")
    #Increase and Decrease Values for the operation variables and show the actual value in QLine Edits
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
#In Operation Monitoration
    #Define the number of Field and Machine ID
    def DecMacID(self):
        self.n_machine_id=self.n_machine_id-1
        self.ql_machine_id.setPlainText(str(self.n_machine_id))
    def IncMacID(self):
        self.n_machine_id=self.n_machine_id+1
        self.ql_machine_id.setPlainText(str(self.n_machine_id))
    def DecFilID(self):
        self.n_field_id=self.n_field_id-1
        self.ql_field_id.setPlainText(str(self.n_field_id))
    def IncFilID(self):
        self.n_field_id=self.n_field_id+1
        self.ql_field_id.setPlainText(str(self.n_field_id)) 
    def DefineID(self): # Define the Machine ID, Field ID and the logfilename
        self.machineID='M-'+str(self.n_machine_id)
        self.fieldID='F-'+str(self.n_field_id)
        self.logfile_name=self.machineID+'_'+self.fieldID+'.txt'
        f=open(self.logfile_name,'w') #Creat the logfile with header
        f.write("Data,Hora,MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),GPS Status,PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),\
Mean Op Cap(ha/h),Instantanea OpCap(ha/h),Time Operation,Area(ha),Row Spacing(m),Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode\n")
        f.close()
        #Reset area and time operation and clear configuration
        self.area=0.0
        self.time_operation=0.0
        self.fertfile_name=""
        self.seedfile_name=""
    #For calibrate the Load Cell of Fert Tank
    def SaveCal(self): #Save the point for calibration.
        sum_wgt=0 #Read 20 values for voltage, calculate the mean, show in Line Edit and save in array
        for i in range(0,20):
            value=round(1.8*ADC.read(pinLoadCell),2)
            sum_wgt=sum_wgt+value
            time.sleep(0.5)
        self.wgt_voltage_cal[self.list_cal_wgt.currentIndex()]=round(sum_wgt/20,5)
        self.qd_voltage_cal.setPlainText(str(round(sum_wgt/20,5)))
    #Calculate the calibration coeficiente m=a+b*voltage
    def Calibration(self):
        if self.wgt_voltage_cal[0]!=0 and self.wgt_voltage_cal[1]!=0 and self.wgt_voltage_cal[2]!=0 and self.wgt_voltage_cal[3]!=0: 
            mass = np.array([8.0,13.0,18.0,23.0]) #fix mass
            self.cal_a,self.cal_b,r_value,p_value,std_error=stats.linregress(self.wgt_voltage_cal,mass)
            self.cal_a=round(self.cal_a,4)
            self.cal_b=round(self.cal_b,4)
        else: self.qd_voltage_cal.setPlainText("Error")
##For Test Only Tab
# Incread and Decrease the Duty Cicle of PWM for Seed Motor and Fert Motor
    def DecSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal-10
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>100.0:self.dt_seed_cal=100.0
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
    def IncSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal+10
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>100.0:self.dt_seed_cal=100.0
            self.ql_dt_speed.setPlainText(str(self.dt_seed_cal))
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
    def DecFertCal(self):
        if self.cb_speed_fert.isChecked():
            self.dt_fert_cal=self.dt_fert_cal-10
            if self.dt_fert_cal<0:self.dt_fert_cal=0
            elif self.dt_fert_cal>100.0:self.dt_fert_cal=100.0
            self.ql_dt_fert.setPlainText(str(self.dt_fert_cal))
            PWM.set_duty_cycle(pinPWM_Fert,self.dt_fert_cal)
            GPIO.output(pinEnable_Fert,GPIO.HIGH)
    def IncFertCal(self):
        if self.cb_speed_fert.isChecked():
            self.dt_fert_cal=self.dt_fert_cal+10
            if self.dt_fert_cal<0:self.dt_fert_cal=0
            elif self.dt_fert_cal>100.0:self.dt_fert_cal=100.0
            self.ql_dt_fert.setPlainText(str(self.dt_fert_cal))
            PWM.set_duty_cycle(pinPWM_Fert,self.dt_fert_cal)
            GPIO.output(pinEnable_Fert,GPIO.HIGH)
# Incread and Decrease Population and Fert Ratio for a Dynamic Test
    def self.IncPopFert(self):
        if self.cb_dyn_seed.isChecked():
            self.popseed=self.popseed+10000
            if self.popseed==80000:self.popseed=0
            self.ql_seed_dyn.setPlainText(str(self.popseed))
        if self.cb_dyn_fert.isChecked():
            self.fert_rt=self.fert_rt+100
            if self.fert_rt==500:self.fert_rt=0
#
# Functions of the timers (timeouts)            
#
    def GPSFunction(self): #GPS Function - Try until read GPRM NMEA Sentece
        if GPIO.input(pinOnOffButton):
            while ('$GPRMC' in self.nmea) is False : 
                self.nmea=gps.readline()
                try:self.nmea=self.nmea.decode('utf-8')
                except:
                    self.nmea=gps.readline()
                    self.nmea=self.nmea.decode('utf-8')
            self.data,self.hora,self.lat_utm,self.long_utm,self.lat,self.long,self.status=operation.ReadGPS(self.nmea)
            self.nmea=''
    def EncoderFunction(self): #Read the Speed of Seed Motor, Machine and the Weight of Fert Tank
        if GPIO.input(pinOnOffButton):
            self.real_rot_seed=operation.SeedSpeed(self.change_popseed) # read the real speed of seed motor. The change_pop seed is a status if the population seed ou duty cicle change
            if self.cb_motion_simulate.isChecked(): #Use a simulated Speed (For tests) or read from the encoder
                self.speed=float(self.ql_sim_speed.toPlainText()) 
            else: self.speed=operation.WheelSpeed() 
            self.fert_wgt=operation.ReadWeight(self.cal_a,self.cal_b) #Read Fert Weight

    def LogFunction(self): # Function for generate a log file and remote monitoring
        if GPIO.input(pinOnOffButton):
            string="Saving in: "+str(self.logfile_name)
            self.ql_logfile.setPlainText(string)
            data_string=self.data+','+self.hora+","+self.machineID+","+self.fieldID+","+str(self.lat_utm)+","+\
str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+str(self.speed)+","+str(self.status)+","+ \
str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+","+str(self.inst_opcap)+","+str(self.time_operation)+\
","+str(self.area)+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+","+str(self.fertbym)\
+"," +str(self.seed_mode)+","+str(self.fert_mode)
            f=open(self.logfile_name,'a')
            f.write(data_string)
            f.write("\n")
            f.close()
            if self.cb_remote.isChecked():
                link='http://andrecoelho.tech/envia_mysql_hostinger.php?MachineID='+self.machineID\
+'&FieldID='+self.fieldID+'&Lati='+str(self.lat)+'&Longi='+str(self.long)+'&XUtm='+str(self.lat_utm)+'&YUtm='+str(self.long_utm)+'&Speed='+\
str(self.speed)+'&OpCap='+str(self.opcap)+'&TimeOperation='+str(self.time_operation)+'&Population='+str(self.popseed)+'&FertRatio='+\
str(self.fert_rt)+'&FertLevel='+str(self.fert_wgt)+'&Area='+str(self.area)
                sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+'\r'))
                time.sleep(1)
                sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
                time.sleep(1)
                sim800l.write(str.encode('AT+HTTPREAD'+'\r'))
                time.sleep(1)
                self.ql_remote_status.setPlainText("Saving")
            else:
                self.ql_remote_status.setPlainText("Disable")
        else:
            self.ql_logfile.setPlainText("Not Saving")
#
# Function that control the seed and fert application motor            
    def ControlFunction(self):  
        if GPIO.input(pinOnOffButton):
            self.disk_hole=int(self.list_holes.currentText()) #Read Hole Disk
            ###Seeder Distributor###
            if self.cb_seed_map.isChecked() and len(self.lat_map_seed)>1 and self.status=='A': #Map mode is active, have map and gps signal
                self.seed_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.popseed,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
            elif self.cb_seed_fix.isChecked(): #Fix seed distribuition rate
                self.seed_mode="FIX"
                self.scene.clear() # clear the Graphics View
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            elif self.cb_seed_fix.isChecked() and self.cb_seed_map.isChecked(): #if two mode is checked
                self.cb_seed_fix.setCheckState (False)
                self.cb_seed_map.setCheckState (False)
            else:  
                self.seed_mode="OFF"
                self.popseed=0
            #check if population change or duty cicle change ==> for use in Encoder Function
            if self.popseed!=self.last_popseed or self.dt_seed_cal!=self.last_dt_seed_cal:
                self.change_popseed=True
            else :self.change_popseed=False
            self.last_popseed=self.popseed
            self.last_dt_seed_cal=self.dt_seed_cal
            #Calcute and Control Speed Motor
            if self.cb_speed_cal.isChecked() is False: #if test function is not active
                self.rot_seed,self.seedbym=operation.Seeder(self.speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                self.dt_seed=operation.ControlSpeedSeed(self.change_popseed,self.rot_seed,self.real_rot_seed,self.cal_a_seed,self.cal_b_seed)
            ####Fertilizer Distribution###
            #
            if self.cb_fert_map.isChecked() and len(self.lat_map_fert)>1 and self.status=='A':#Map mode is active, have map and gps signal
                self.fert_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                self.fert_rt,lat_used,long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_fert,self.long_map_fert,self.map_fertrt)
                # add atual and used point in view
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.scene.addRect(lat_used,long_used,0.5,0.5,self.Kpen,self.Kbrush)
            elif self.cb_fert_fix.isChecked(): #Fix seed distribuition rate
                self.fert_mode="FIX"
                self.scene.clear()
                if self.status=='A': #plot in graph if have gps signal
                    self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                    self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            elif self.cb_fert_fix.isChecked() and self.cb_fert_map.isChecked(): #if two mode is checked
                self.cb_fert_fix.setCheckState (False)
                self.cb_fert_map.setCheckState (False)
            else:
                self.fert_mode="OFF"
                self.fert_rt=0
            #check if fertilizer ratio change ==> for use in dynamic calibration
            if self.fert_rt!=self.last_fert_rt:
                self.change_fertrt=True
            else :self.change_fertrt=False
            self.last_fert_rt=self.fert_rt
            self.last_wgt=self.fert_wgt #update the fertilizer wgt
            #Calcute and Control Speed Motor
            if self.cb_speed_fert.isChecked() is False:
                self.fertbym,self.fertbys=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
                operation.ControlSpeedFert(self.fertbys,self.fert_wgt,self.last_wgt,self.speed,self.time_control,self.last_fert_rt)
            #calculate operational capability and area 
            self.inst_area=round(self.row_spacing*self.speed*self.time_control/10000.0,4)
            self.area=round(self.area+self.inst_area,4) #in ha
            self.inst_time=round(self.time_control/3600.0,5)
            self.time_operation=round(self.time_operation+self.inst_time,5)   # in h
            self.opcap=round(self.area/(self.time_operation),3) # in ha/h
            self.inst_opcap=round(self.inst_area/self.inst_time,3)
            # Update LineEdit in main tab
            self.ql_speed.setPlainText(str(self.speed))
            if self.dt_seed==100.0: self.ql_seed.setPlainText('HIGH')
            elif self.dt_seed==0.0:self.ql_seed.setPlainText('LOW')
            else: self.ql_seed.setPlainText(str(self.popseed))
            if self.status=='A':self.ql_pdop.setPlainText('Active')
            if self.status!='A':self.ql_pdop.setPlainText('No Signal')
            self.ql_fert_rt.setPlainText(str(self.fert_rt))
            self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
            self.ql_area.setPlainText(str(self.area))
            self.ql_opcap.setPlainText(str(self.time_operation))
            self.lb_status.setText('CALC'+str(self.rot_seed)+' REAL:'+str(self.real_rot_seed))
            self.lb_datetime.setText(self.hora)
            #in Test Only Tab
            self.ql_speed_seed.setPlainText(str(self.real_rot_seed))
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

