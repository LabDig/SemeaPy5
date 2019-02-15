#02 de fev
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog,QMessageBox
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
#Wheel Encoder
pinEncWhell="P8_11"
GPIO.setup(pinEncWhell, GPIO.IN)
#PWM Seed
pinPWM_Seed="P8_13"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_10"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
pinEncSeed="P9_29"
GPIO.setup(pinEncSeed, GPIO.IN)
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
gps = serial.Serial ("/dev/ttyS4", 9600,timeout=0.5) # P9_11 P9_13
#3G
sim800l = serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26
#
#Arduino
ino = serial.Serial("/dev/ttyACM0", 9600) # P9_21 P9_22
#
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        #global variables declaration
        self.speed,self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap,self.time_operation=0.0,'',0.0,0.0,0.0,0.0,0.0 #operation variables
        self.inst_opcap,self.inst_area,self.inst_time=0,0,0 #operation variables
        self.time,self.date,self.nmea,self.lat_utm,self.long_utm,self.lat,self.long,self.pdop,self.status='','','',0,0,0,0.0,0,0 #gps variables
        self.row_spacing,self.disk_hole,self.seed_germ,self.logfile_name=0,0,0,"" #operation variables
        self.lat_map_fert,self.long_map_fert,self.map_fertrt=[],[],[] #map  fert
        self.lat_map_seed,self.long_map_seed,self.pop_map_seed,=[],[],[] #map seed
        self.seed_mode,self.fert_mode,self.fertbym,self.seedbym="OFF","OFF",0,0, #operation modes
        self.rot_seed,self.real_rot_seed=0.0,0.0 #seed speed
        self.wgt_voltage_cal=np.zeros(4) #calibrate load cell
        self.dt_seed_cal,self.dt_fert_cal=0,0 #variables for test 
        self.change_popseed=False #variables for dynamic test and check if population changet
        self.last_popseed,self.last_fert_rt=0,0 #for ckeck if population and fert ratio change
        self.dt_seed,self.last_dt_seed_cal=-1,-1  #for ckeck if population and fert ratio change
        self.n_machine_id,self.n_field_id=1,1 #number auxiliar for setting machine and field id
        self.lat_used,self.long_used=0,0 #lat e long used in Map aplication
        self.fert_rt_cal=0 #fert ratio to calibration distribuitor
        self.rm_st,self.rec='','' #remote status and string send by smartphone
        self.calc_m_fert,self.last_wgt=0,0 #calc the mass exit in log function period
        self.aux,self.error=0,0#to 3g function, log and gps
        self.array_s,self.array_w=[],[] #mean speed filter
        #timers configuration
        self.control_timer = QtCore.QTimer()
        self.log_timer = QtCore.QTimer()
        self.gps_timer = QtCore.QTimer()
        self.ino_timer = QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.log_timer.timeout.connect(self.LogFunction)
        self.gps_timer.timeout.connect(self.GPSFunction)
        self.ino_timer.timeout.connect(self.SpeedFunction)
        self.time_control=0.25 #in s
        self.control_timer.start(self.time_control*1000) #Start Control Function
        self.log_timer.start(5000)
        self.gps_timer.start(1000)
        self.ino_timer.start(200)
        s='1'
        ino.write(s.encode())
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
        #Calibration
        self.m_dt_seed.clicked.connect(self.DecSeedCal)  #button plus and minus
        self.p_dt_seed.clicked.connect(self.IncSeedCal)  # values for variavel operations
        self.m_dt_fert.clicked.connect(self.DecFertCal) 
        self.p_dt_fert.clicked.connect(self.IncFertCal)
        self.m_fert_cal.clicked.connect(self.DecFertRt) 
        self.p_fert_cal.clicked.connect(self.IncFertRt)
        self.save_cal_seed.clicked.connect(self.CalSeed)
        self.save_cal_fert.clicked.connect(self.CalFert)
        self.cal_seed_fert.clicked.connect(self.CalibrateSeedFert)
#
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
            self.cal_a_cell=float(f.readline()) #calibration for load cell
            self.cal_b_cell=float(f.readline()) #calibration for load cell
            self.cal_a_seed=float(f.readline()) #calibration for seed spped
            self.cal_b_seed=float(f.readline()) #calibration for seed spped
            self.cal_a_fert=float(f.readline()) #calibration for fert ratio
            self.cal_b_fert=float(f.readline()) #calibration for fert ratio
            self.area=float(f.readline()) #
            self.time_operation=float(f.readline())
            self.log_id=int(f.readline())
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
###
#Functions
# Im main Tab
#   
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
            f.write(str(self.cal_a_cell)+"\n")
            f.write(str(self.cal_b_cell)+"\n")
            f.write(str(self.cal_a_seed)+"\n")
            f.write(str(self.cal_b_seed)+"\n")
            f.write(str(self.cal_a_fert)+"\n")
            f.write(str(self.cal_b_fert)+"\n")
            f.write(str(self.area)+"\n")
            f.write(str(self.time_operation)+"\n")
            f.write(str(self.log_id)+"\n")
        f.close()
        s='0'
        ino.write(s.encode())
        GPIO.cleanup()
        PWM.cleanup()
        sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
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
            else: QMessageBox.information(self,'Load Seed Map','No map')
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
        else: QMessageBox.information(self,'Load Seed Map','No map')
    #Increase and Decrease Values for the operation variables and show the actual value in QLine Edits
    def DecPop(self):
        self.popseed=self.popseed-5000
        self.ql_set_pop.setPlainText(str(self.popseed))
    def IncPop(self):
        self.popseed=self.popseed+5000
        self.ql_set_pop.setPlainText(str(self.popseed))
    def DecGer(self):
        self.seed_germ=self.seed_germ-1
        self.ql_germ.setPlainText(str(self.seed_germ))
    def IncGer(self):
        self.seed_germ=self.seed_germ+1
        self.ql_germ.setPlainText(str(self.seed_germ))
    def DecFert(self):
        self.fert_rt=self.fert_rt-25
        self.ql_set_fert.setPlainText(str(self.fert_rt))
    def IncFert(self):
        self.fert_rt=self.fert_rt+25
        self.ql_set_fert.setPlainText(str(self.fert_rt))
    def DecRow(self):
        self.row_spacing=self.row_spacing-0.1
        self.ql_row_spacing.setPlainText(str(self.row_spacing))
    def IncRow(self):
        self.row_spacing=self.row_spacing+0.1
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
        self.logfile_name=os.path.join(self.dir,self.machineID+'_'+self.fieldID+'.txt')
        f=open(self.logfile_name,'w') #Creat the logfile with header
        f.write("LogID,Date,Time (UTC),MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),GPS Status,PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),\
Mean Op Cap(ha/h),Instantanea OpCap(ha/h),Time Operation,Area(ha),Row Spacing(m),Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode,\
Remote Status,Calc Mass Fert Exit, Real Mass Fert Exit, Difference Mass\n")
        f.close()
        #Reset area and time operation and clear configuration
        self.area=0.0
        self.time_operation=0.0
        self.fertfile_name=""
        self.seedfile_name=""
        self.log_id=0
#calibrate the Load Cell of Fert Tank
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
            self.cal_a_cell,self.cal_b_cell,r_value,p_value,std_error=stats.linregress(self.wgt_voltage_cal,mass)
            self.cal_a_cell=round(self.cal_a,4)
            self.cal_b_cell=round(self.cal_b,4)
            QMessageBox.information(self,'Calibrate Load Cell','Sucess')
        else: QMessageBox.information(self,'Calibrate Load Cell','Error')
        print ("Load Cell",self.cal_a_cell,self.cal_b_cell,r_value,p_value,std_error)
##Calibration
# Incread and Decrease the Duty Cicle of PWM for Seed Motor and Fert Motor
    def DecSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal-10
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>100.0:self.dt_seed_cal=100.0
            self.ql_dt_seed.setPlainText(str(self.dt_seed_cal))
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
    def IncSeedCal(self):
        if self.cb_speed_cal.isChecked():
            self.dt_seed_cal=self.dt_seed_cal+10
            if self.dt_seed_cal<0.0:self.dt_seed_cal=0.0
            elif self.dt_seed_cal>100.0:self.dt_seed_cal=100.0
            self.ql_dt_seed.setPlainText(str(self.dt_seed_cal))
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
    def DecFertRt(self): #set the fert ratio mensured
        if self.cb_speed_fert.isChecked():
            self.fert_rt_cal=self.fert_rt_cal-1.0
            self.ql_fert_cal.setPlainText(str(self.fert_rt_cal))
    def IncFertRt(self):
        if self.cb_speed_fert.isChecked():
            self.fert_rt_cal=self.fert_rt_cal+1.0
            self.ql_fert_cal.setPlainText(str(self.fert_rt_cal))
#   
# Incread and Decrease Population and Fert Ratio for a Dynamic Test
    def IncPopFert(self):
        self.popseed=self.popseed+10000
        self.fert_rt=self.fert_rt+100
        print ("din")
#Calibrate Seed and Fert
    def CalSeed(self):
        print (self.real_rot_seed,self.dt_seed_cal)
        f=open("cal_seed.txt","a")
        f.write(str(self.real_rot_seed)+','+str(self.dt_seed_cal)+'\n')
        f.close()
#
    def CalFert (self):
        print (self.fert_rt_cal,self.dt_fert_cal)
        f=open("cal_fert.txt","a")
        f.write(str(self.fert_rt_cal)+','+str(self.dt_fert_cal)+'\n')
        f.close()
#
    def CalibrateSeedFert(self):
        #seed
        if self.cb_speed_cal.isChecked():
            content=None # clear variable for security
            with open("cal_seed.txt", "r",encoding='latin-1') as f: content = f.read().splitlines()
            f.close()
            self.duty_cal_seed,self.speed_cal_seed=np.zeros(len(content)-1),np.zeros(len(content)-1)
            for i in range(1,len(content)-1):
                Row=content[i].split(',')
                self.speed_cal_seed[i-1]=float(Row[0])
                self.duty_cal_seed[i-1]=float(Row[1])
            if len (self.duty_cal_seed)>1:
                self.cal_a_seed,self.cal_b_seed,r_value,p_value,std_error=stats.linregress(self.speed_cal_seed,self.duty_cal_seed) # x (rot),y (duty) #duty =a*rot+b
                QMessageBox.information(self,'Calibrate Seed','Sucess')
                print ("Seed",self.cal_a_seed,self.cal_b_seed,r_value,p_value,std_error)
            else : QMessageBox.information(self,'Calibrate Seed','Error')
            f=open("cal_seed.txt","w")
            f.write('Seed Calibrate\n')
            f.close()
            #fert
        if self.cb_speed_fert.isChecked():
            content=None # clear variable for security
            with open("cal_fert.txt", "r",encoding='latin-1') as f: content = f.read().splitlines()
            f.close()
            self.duty_cal_fert,self.rt_cal_fert=np.zeros(len(content)-1),np.zeros(len(content)-1)
            for i in range(1,len(content)-1):
                Row=content[i].split(',')
                self.rt_cal_fert[i-1]=float(Row[0])
                self.duty_cal_fert[i-1]=float(Row[1])
            if len (self.rt_cal_fert)>1 :
                self.cal_a_fert,self.cal_b_fert,r_value,p_value,std_error=stats.linregress(self.rt_cal_fert,self.duty_cal_fert) # x (rot),y (duty) #duty =a*rot+b
                QMessageBox.information(self,'Calibrate Fert','Sucess')
                print("Fert",self.cal_a_fert,self.cal_b_fert,r_value,p_value,std_error)
            else :QMessageBox.information(self,'Calibrate Fert','Error')
            #clear files
            f=open("cal_fert.txt","w")
            f.write('Fert Calibrate\n')
            f.close()
#
# Functions of the timers (timeouts)            
#
    def GPSFunction(self): #GPS Function - Try until read GPRM NMEA Sentece
        while ('$GPRMC' in self.nmea) is False : 
            self.nmea=gps.readline()
            try:self.nmea=self.nmea.decode('utf-8')
            except:
                self.nmea=gps.readline()
                self.nmea=self.nmea.decode('utf-8')
        self.date,self.time,self.lat_utm,self.long_utm,self.lat,self.long,self.status=operation.ReadGPS(self.nmea)
        self.nmea=''
        #Search Neigbhor for Map based aplication
        if "MAP" in self.seed_mode:
            self.popseed,self.lat_used,self.long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_seed,self.long_map_seed,self.pop_map_seed)
        if "MAP" in self.fert_mode:
            self.fert_rt,self.lat_used,self.long_used=operation.FindNeig(self.lat_utm,self.long_utm,self.lat_map_fert,self.long_map_fert,self.map_fertrt)
#    
    def LogFunction(self): # Function for generate a log file
        if GPIO.input(pinOnOffButton):
            self.log_id=self.log_id+1
            #Local Logger
            string="Saving in: "+str(self.logfile_name)
            self.ql_logfile.setPlainText(string)
            data_string=str(self.log_id)+','+self.date+','+self.time+","+self.machineID+","+self.fieldID+","+str(self.lat_utm)+","+\
str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+str(self.speed)+","+str(self.status)+","+ \
str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+","+str(self.inst_opcap)+","+str(self.time_operation)+\
","+str(self.area)+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+","+str(self.fertbym)\
+"," +str(self.seed_mode)+","+str(self.fert_mode)+","+self.rm_st+","+str(self.dt_seed)+","+str(self.rot_seed)+","+str(self.real_rot_seed)+"," +\
    str(self.calc_m_fert)+"," + str(self.last_wgt-self.fert_wgt)+ "\n"
            f=open(self.logfile_name,'a')
            f.write(data_string)
            f.close()
            #Fert Mass Exit Control
            self.calc_m_fert=0 #reset
            self.last_wgt=self.fert_wgt
        else:self.ql_logfile.setPlainText("Not saving")
#
  
    def SpeedFunction(self):
            #speed
            vel=ino.readline()
            vel=vel.decode('utf-8')
            try:self.real_mach_speed,self.real_rot_seed=vel.split(',')
            except:pass
            self.real_mach_speed=2.0*float(self.real_mach_speed)
            self.real_rot_seed=float(self.real_rot_seed)
            if self.real_mach_speed>0.4 :
                self.real_mach_speed,self.array_w=self.MeanFilter(self.real_mach_speed,self.array_w)
                self.real_mach_speed=round(self.real_mach_speed,1)
            else:self.real_mach_speed=0
            if  self.change_popseed is True:  self.array_s=[]
            if  self.real_rot_seed>0.1 and self.change_popseed is False:
                self.real_rot_seed,self.array_s=self.MeanFilter(self.real_rot_seed,self.array_s)
                self.real_rot_seed=round(self.real_rot_seed,2)

    def MeanFilter(self,value,array):
        array=np.append(array,value)
        if len(array)>5:np.append(array,0)
        return np.mean(array),array
        
# Function that control the seed and fert application motor            
    def ControlFunction(self):
        if GPIO.input(pinOnOffButton):
            if self.status=='A': #plot in graph if have gps signal
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            #tank mass
            self.fert_wgt=operation.ReadWeight(self.cal_a_cell,self.cal_b_cell) #Read Fert Weight
            
            if self.cb_motion_simulate.isChecked(): #Use a simulated Speed (For tests) or read from the encoder
                self.speed=float(self.ql_sim_speed.toPlainText()) 
            else: self.speed=self.real_mach_speed
            #
            self.disk_hole=int(self.list_holes.currentText()) #Read Hole Disk
            ###Seeder Distributor###
            if self.cb_seed_map.isChecked() and len(self.lat_map_seed)>1 and self.status=='A': #Map mode is active, have map and gps signal
                self.seed_mode="MAP"
                # find in the map the point nearst to atual point. The return it's the population and the map point used
                #In GPS Function
                if self.lat_used>0:self.scene.addRect(self.lat_used,self.long_used,0.5,0.5,self.Kpen,self.Kbrush)
            elif self.cb_seed_fix.isChecked(): #Fix seed distribuition rate
                self.seed_mode="FIX"
            elif self.cb_seed_fix.isChecked() and self.cb_seed_map.isChecked(): #if two mode is checked
                self.cb_seed_fix.setCheckState (False)
                self.cb_seed_map.setCheckState (False)
            else:  
                self.seed_mode="OFF"
                self.popseed=0
            #check if population change or duty cicle change ==> reset encoder seed speed variable
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
                #In GPS Function
                if self.lat_used>0:self.scene.addRect(self.lat_used,self.long_used,0.5,0.5,self.Kpen,self.Kbrush)
            elif self.cb_fert_fix.isChecked(): #Fix seed distribuition rate
                self.fert_mode="FIX"
            elif self.cb_fert_fix.isChecked() and self.cb_fert_map.isChecked(): #if two mode is checked
                self.cb_fert_fix.setCheckState (False)
                self.cb_fert_map.setCheckState (False)
            else:
                self.fert_mode="OFF"
                self.fert_rt=0
            #check if fertilizer ratio change ==> for use in dynamic calibration (future development)
            if self.fert_rt!=self.last_fert_rt:
                self.change_fertrt=True
            else :self.change_fertrt=False
            self.last_fert_rt=self.fert_rt
            # 
            #Calcute and Control Speed Motor
            if self.cb_speed_fert.isChecked() is False:
                self.fertbym,self.fertbys=operation.Fert(self.speed,self.fert_rt,self.row_spacing)
                operation.ControlSpeedFert(self.cal_a_fert,self.cal_b_fert,self.fertbys)
            #cal mass distribution (in log periodo =10 s)
            self.calc_m_fert=self.calc_m_fert+self.fertbys*self.time_operation #in kg => reset in LogFunction
            #calculate operational capability and area 
            self.inst_area=self.row_spacing*self.speed*self.time_control #in ha
            self.area=round(self.area+self.inst_area/10000,2) #in h2
            self.inst_time=round(self.time_control/3600.0,5) #in h
            self.time_operation=round(self.time_operation+self.inst_time,5)   # in h
            self.opcap=round(self.area/(self.time_operation),3) # in ha/h
            self.inst_opcap=round(self.inst_area/self.inst_time,3)
            # Update LineEdit in main tab
            self.ql_speed.setPlainText(str(self.speed))
            self.ql_seed.setPlainText(str(self.popseed))
            if self.status=='A':self.ql_pdop.setPlainText('Active')
            else:self.ql_pdop.setPlainText('No Signal')
            self.ql_fert_rt.setPlainText(str(self.fert_rt))
            self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
            self.ql_area.setPlainText(str(self.area))
            self.ql_opcap.setPlainText(str(self.time_operation))
            self.lb_status.setText('CALC'+str(self.rot_seed)+' REAL:'+str(self.real_rot_seed)+'Dt:'+str(self.dt_seed))
            if self.dt_seed<40.0: self.lb_status.setText("Low Seed Speed")
            self.lb_datetime.setText(self.time)
            #in Calibrate
            self.ql_speed_cal.setPlainText(str(self.real_rot_seed))
            # 3G and LogFunctions
            dt=int(4*self.time_control)
            if self.aux==dt and  self.cb_remote.isChecked():sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
            if self.aux==2*dt and  self.cb_remote.isChecked(): sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
            if self.aux==3*dt and  self.cb_remote.isChecked(): sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
            if self.aux==4*dt and  self.cb_remote.isChecked(): sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
            if self.aux==5*dt and  self.cb_remote.isChecked(): sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
            if self.aux==6*dt and  self.cb_remote.isChecked(): sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
            link='http://andrecoelho.tech/SemeaView/send_mysql.php?'
            str_data='LogID='+str(self.log_id)+'&Date='+str(self.date)+'&Time='+str(self.time)+'&MachineID='+self.machineID\
+'&FieldID='+self.fieldID+'&Lati='+str(self.lat)+'&Longi='+str(self.long)+'&XUtm='+str(self.lat_utm)+'&YUtm='+str(self.long_utm)+'&Speed='+\
str(self.speed)+'&OpCap='+str(self.opcap)+'&TimeOperation='+str(self.time_operation)+'&Population='+str(self.popseed)+'&FertRatio='+\
str(self.fert_rt)+'&FertWgt='+str(self.fert_wgt)+'&Area='+str(self.area)
            if self.aux==9*dt and self.cb_remote.isChecked(): sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+str_data+'\r'))
            if self.aux==12*dt and self.cb_remote.isChecked():
                sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
                self.aux=7*dt
            rec=sim800l.readline()
            rec=rec.decode('utf-8')
            if '+SAPBR:' in rec: QMessageBox.information(self,'3G',rec)
            if '+HTTPACTION:' in rec:
                if "200" in rec :
                    self.error=0
                    self.rm_st='ON'
                    self.ql_remote_status.setPlainText("Send")
                if not "200" in rec :
                    self.error=self.error+1
                    self.rm_st='ERROR'
                    self.ql_remote_status.setPlainText("Error")
            if self.cb_remote.isChecked() is False :
                    self.rm_st='OFF'
                    self.aux=0
                    self.ql_remote_status.setPlainText("Disable")
                    sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
            if self.error==3:
                    self.error=0
                    self.rm_st="RESET"
                    sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
                    self.aux=0 #reset 3G
                    QMessageBox.information(self,'3G','Error')
            self.aux=self.aux+1
        else: # If button is off
            GPIO.output(pinEnable_Seed,GPIO.LOW)
            GPIO.output(pinEnable_Fert,GPIO.LOW)
            self.lb_status.setText("Desabilitado")
            sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
            self.aux=0

#Run the app:
if __name__ == '__main__':
    if not QtWidgets.QApplication.instance():
        app = QtWidgets.QApplication(sys.argv)
    else:
        app = QtWidgets.QApplication.instance()
    ex = Semea()
    ex.show()
    sys.exit(app.exec_())

