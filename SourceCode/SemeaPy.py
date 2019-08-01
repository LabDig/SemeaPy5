#Versao usada na Semeadura Maio/2019
#
# -*- coding: utf-8 -*-
#!/usr/bin/python3
import math
import utm
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog,QMessageBox
from PyQt5 import QtGui,QtCore
import time
from scipy import stats
import numpy as np
import shapefile
# import python files
from seeder_ui import Ui_SEMEA #gui
from keyboard import Ui_KeyBoard #class of keyboard
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
#PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_10"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
#PWM Fertilizer
pinPWM_Fert="P8_19"
#PWM.start(pinPWM_Fert,0, 1000.0) #pin, duty,frequencia
pinEnable_Fert="P8_9"
GPIO.setup(pinEnable_Fert, GPIO.OUT)
GPIO.output(pinEnable_Fert,GPIO.LOW)
#LoadCell
ADC.setup()
pinLoadCell="P9_33"
#GPS configure and disable nmea
gps = serial.Serial ("/dev/ttyS4", 9600,timeout=0.05) # P9_11 P9_13
msg=["$PUBX,40,GLL,0,0,0,0*5C\r\n","$PUBX,40,VTG,0,0,0,0*5E\r\n","$PUBX,40,GSV,0,0,0,0*59\r\n","$PUBX,40,GGA,0,0,0,0*5A\r\n"]
for i in msg : gps.write(i.encode())
#3G
sim800l = serial.Serial("/dev/ttyS1", 9600,timeout=0.05) # P9_24 P9_26
#
#Arduino
ino = serial.Serial("/dev/ttyS2", 9600,timeout=0.05) # P9_21 P9_22
#
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        #grapghs view configuration
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        #colors
        self.Mpen,self.Mbrush=QtGui.QPen(QtCore.Qt.magenta),QtGui.QBrush(QtCore.Qt.magenta)
        self.Ypen,self.Ybrush=QtGui.QPen(QtCore.Qt.yellow),QtGui.QBrush(QtCore.Qt.yellow)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)
        self.pen_color=[self.Mpen,self.Ypen,self.Rpen,self.Bpen]
        self.brush_color=[self.Mbrush,self.Ybrush,self.Rbrush,self.Bbrush]
        #global variables declaration
        self.popseed,self.fert_rt,self.fert_wgt,self.area,self.opcap,self.time_operation='',0.0,0.0,0.0,0.0,0.0 #operation variables
        self.inst_opcap,self.inst_area,self.inst_time=0,0,0 #operation variables
        self.time,self.date,self.lat_utm,self.long_utm,self.lat,self.long,self.status,self.pdop='','',0,0,0.0,0,0,0 #gps variables
        self.row_spacing,self.disk_hole,self.seed_germ,self.logfile_name=0,0,0,"" #operation variables
        self.map_fert,self.fert_values=0,0 #map  fert
        self.map_seed,self.pop_values=0,0 #map seed
        self.mach_speed=0
        self.seed_mode,self.fert_mode,self.fertbym,self.seedbym="OFF","OFF",0,0, #operation modes
        self.rot_seed,self.real_rot_seed=0.0,0.0 #seed speed calculated and measured
        self.wgt_voltage_cal=np.zeros(4) #calibrate load cell
        self.dt_seed_cal,self.dt_fert_cal=0,0 #variables for test and calibrate speed
        self.change_popseed=False #variables for dynamic test and check if population changet
        self.last_popseed,self.last_fert_rt=0,0 #for ckeck if population and fert ratio change
        self.dt_seed,self.last_dt_seed_cal=-1,-1  #for ckeck if population and fert ratio change
        self.dt_fert=0;
        self.n_machine_id,self.n_field_id=1,1 #number auxiliar for setting machine and field id
        self.lat_used,self.long_used=0,0 #lat e long used in Map aplication
        self.fert_rt_cal=0 #fert ratio to calibration distribuitor
        self.rm_st='' #remote status
        self.calc_m_fert,self.last_wgt=0,0 #calc the mass exit in log function period
        self.aux,self.error=0,0#to 3g function, log and gps
        self.start_remote='0'
        self.var='' #aux to keyboard function
        self.array,self.filter_rot_seed=[],0 #filter rot seed speed
        self.aux_block=0 #block seed motor check
        self.dist,self.last_dist,self.lat_fix,self.long_fix,self.lat_utm_fix,self.long_utm_fix=0,1000,0,0,0,0 #test of position precision
        self.st_increase=False
        self.lat_fix=0
        self.long_fix=0
        self.array2,self.filter_mach=[],0
        
        
        #Open the software with configuration of last use
        self.dir=os.path.dirname(os.path.abspath(__file__))  
        self.conffile_name=os.path.join(self.dir,"conf.txt")
        with open(self.conffile_name, "r",encoding='latin-1') as f:
            self.seed_mode=(f.readline()) #seed mode
            self.seedfile_name=f.readline()
            self.popseed=float(f.readline())
            self.seed_germ=int(f.readline())
            self.fert_mode=(f.readline()) #fert mode
            self.fertfile_name=f.readline()
            self.fert_rt=float(f.readline())
            self.row_spacing=float(f.readline())
            self.st_remote=f.readline()
            self.logfile_name=f.readline() #log file name
            self.machineID=f.readline() 
            self.fieldID=f.readline()
            self.cal_a_cell=float(f.readline()) #calibration for load cell
            self.cal_b_cell=float(f.readline()) #calibration for load cell
            self.load_cell_zero=float(f.readline()) #calibration for load cell
            self.cal_a_seed=float(f.readline()) #calibration for seed spped
            self.cal_b_seed=float(f.readline()) #calibration for seed spped
            self.cal_a_fert=float(f.readline()) #calibration for fert ratio
            self.cal_b_fert=float(f.readline()) #calibration for fert ratio
            self.area=float(f.readline()) #
            self.time_operation=float(f.readline())
            self.log_id=int(f.readline())
            self.time_control=int(f.readline())
            self.time_gps=int(f.readline())
            self.aux_bt=int(f.readline())
            self.aux_gps=int(f.readline())
            
        f.close()
        #Setup the operation, with configuration of last use
        self.seed_mode=self.seed_mode.rstrip()
        self.fert_mode=self.fert_mode.rstrip()
        self.seedfile_name=self.seedfile_name.rstrip()
        self.fertfile_name=self.fertfile_name.rstrip()
        self.logfile_name=self.logfile_name.rstrip()
        self.machineID=self.machineID.rstrip()
        self.fieldID=self.fieldID.rstrip()
        self.st_remote=self.st_remote.rstrip()
        if "OFF" in self.st_remote : self.ql_remote_status.setPlainText("OFF")
        else:  self.ql_remote_status.setPlainText("ON")
        self.le_set_mach_id.setText(str(self.machineID))
        self.le_set_fil_id.setText(str(self.fieldID))
        self.le_row_spacing.setText(str(self.row_spacing))
        self.le_set_germ.setText(str(self.seed_germ))
        self.le_set_fertrt.setText(str(self.fert_rt))
        self.le_set_pop.setText(str(self.popseed))
        self.le_set_t_ctrl.setText(str(self.time_control))
        self.le_set_t_gps.setText(str(self.time_gps))
        self.lb_tb.setText("")
        # Reload the Fert and Seed Map, when map mode is active
        if "MAP" in self.seed_mode: self.map_seed,self.pop_values=self.LoadMap(self.seedfile_name)
        if "MAP" in self.fert_mode: self.map_fert,self.fert_values=self.LoadMap(self.fertfile_name)
        #timers configuration
        self.control_timer = QtCore.QTimer()
        self.log_timer = QtCore.QTimer()
        self.gps_timer = QtCore.QTimer()
        self.ino_timer = QtCore.QTimer()
        self.control_timer.timeout.connect(self.ControlFunction)
        self.log_timer.timeout.connect(self.LogFunction)
        self.gps_timer.timeout.connect(self.GPSFunction)
        self.ino_timer.timeout.connect(self.SpeedFunction)
        self.control_timer.start(self.time_control) #Start Control Function
        self.log_timer.start(1000)
        self.gps_timer.start(self.time_gps)
        self.ino_timer.start(self.time_control)
        #GUI Buttons Configuration
        s='500' #equal self.time_control
        ino.write(s.encode())
        #Main
        self.exit.clicked.connect(self.Close)  #exit button
        #Operation
        self.bt_load_seed.clicked.connect(self.LoadSeedFile) # load seed map
        self.bt_load_fert.clicked.connect(self.LoadFertFile) # load fert map
        self.pop_off.clicked.connect(self.OffPop)
        self.fert_off.clicked.connect(self.OffFert)
        #LineEdit Clickable
        self.le_set_pop.clicked.connect(lambda : self.KeyFunc ('popseed')) #variable to update
        self.le_set_germ.clicked.connect(lambda : self.KeyFunc ('seed_germ')) #variable to update
        self.le_row_spacing.clicked.connect(lambda : self.KeyFunc ('row_spacing')) #variable to update
        self.le_set_fertrt.clicked.connect(lambda : self.KeyFunc ('fert_rt')) #variable to update
        #Setupt Monitoring and Others
        self.bt_st_id.clicked.connect(self.DefineID) # Define Id's for machine and field
        self.bt_save_cal.clicked.connect(self.SaveCal) # Savel Caliration Point
        self.bt_calibrate.clicked.connect(self.Calibration) #buton calibrate
        #
        self.le_set_mach_id.clicked.connect(lambda : self.KeyFunc ('n_machine_id')) #variable to update
        self.le_set_fil_id.clicked.connect(lambda : self.KeyFunc ('n_field_id')) #variable to update
        #
        self.le_set_t_ctrl.clicked.connect(lambda : self.KeyFunc ('time_control')) #variable to update
        self.le_set_t_gps.clicked.connect(lambda : self.KeyFunc ('time_gps')) #variable to update
        #Calibration
        self.le_seed_dt.clicked.connect(lambda : self.KeyFunc ('dt_seed_cal')) #variable to update
        self.le_fert_dt.clicked.connect(lambda : self.KeyFunc ('dt_fert_cal')) #variable to update
        self.le_fert_mass.clicked.connect(lambda : self.KeyFunc ('cal_fert_mass')) #variable to update
        self.le_set_latitude.clicked.connect(lambda : self.KeyFunc ('fix_latitude')) #variable to update
        self.le_set_longitude.clicked.connect(lambda : self.KeyFunc ('fix_longitude')) #variable to update

        #        
        self.save_cal_seed.clicked.connect(self.CalSeed)
        self.save_cal_fert.clicked.connect(self.CalFert)
        self.cal_seed.clicked.connect(self.CalibrateSeed)
        self.cal_fert.clicked.connect(self.CalibrateFert)
        #
        self.bt_remote.clicked.connect(self.ConfigRemote)


#
        
###
#Functions
#Configure remote monitoring

    def ConfigRemote(self):
        if "OFF" in self.st_remote :
            self.st_remote="ON"
            self.ql_remote_status.setPlainText("TURN ON")
            pass
        else:
            self.st_remote="OFF"
            self.ql_remote_status.setPlainText("TURN OFF")
            pass
# Im main Tab
    def Close(self): #save configuration inf file, clear GPIO, stop timer and close the software
        with open(self.conffile_name, "w",encoding='latin-1') as f:
            f.write(self.seed_mode+"\n")
            f.write(self.seedfile_name+"\n")
            f.write(str(self.popseed)+"\n")
            f.write(str(self.seed_germ)+"\n")
            f.write(self.fert_mode+"\n")
            f.write(self.fertfile_name+"\n")
            f.write(str(self.fert_rt)+"\n")
            f.write(str(self.row_spacing)+"\n")
            f.write(str(self.st_remote)+"\n")
            f.write(self.logfile_name+"\n")
            f.write(self.machineID+"\n")
            f.write(self.fieldID+"\n")
            f.write(str(self.cal_a_cell)+"\n")
            f.write(str(self.cal_b_cell)+"\n")
            f.write(str(self.load_cell_zero)+"\n")
            f.write(str(self.cal_a_seed)+"\n")
            f.write(str(self.cal_b_seed)+"\n")
            f.write(str(self.cal_a_fert)+"\n")
            f.write(str(self.cal_b_fert)+"\n")
            f.write(str(self.area)+"\n")
            f.write(str(self.time_operation)+"\n")
            f.write(str(self.log_id)+"\n")
            f.write(str(self.time_control)+"\n")
            f.write(str(self.time_gps)+"\n")
            f.write(str(self.aux_bt)+"\n")
            f.write(str(self.aux_gps)+"\n")
        f.close()
        GPIO.cleanup()
        PWM.cleanup()
        s='0'
        ino.write(s.encode())
        sim800l.write(str.encode('AT+SAPBR=0,1'+'\r')) #disable 3G GSM
        self.close()
#
#In Setup Operation
     # Setting the map file name
    def LoadSeedFile(self):
        self.seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File","/root/SemeaPy5/Maps","*.shp")[0]
        self.map_seed,self.pop_values=self.LoadMap(self.seedfile_name) #polygon map vertices and polygon atributes
        self.seed_mode="MAP" #automaticly set seed mode to MAP
        self.le_set_pop.setText("MAP")
    def LoadFertFile(self):
        self.fertfile_name=QFileDialog.getOpenFileName(self,"Open Fert File","/root/SemeaPy5/Maps","*.shp")[0]
        self.map_fert,self.fert_values=self.LoadMap(self.fertfile_name)
        self.fert_mode="MAP" #automaticly set fert mode to MAP
        self.le_set_fertrt.setText("MAP")
    #
    #Load maps show in Graphics View
    def LoadMap(self,filename):
        self.scene.clear()
        polygon = shapefile.Reader(filename)
        atribute=[]
        poly_map=[]
        if (polygon.shapeType == shapefile.POLYGON): #if shape is polygon type
            n=len (polygon) #polygon numbers
            subpolygon = polygon.shapes()
            print (n)
            subpolygon_atribute = polygon.records() #contain do atributo em cada polygon
            for i in range(n):
                poly_map.append(subpolygon[i].points)
                atribute.append(subpolygon_atribute[i][1]) #valor do atributo
                print (subpolygon_atribute[i][1])
                print (subpolygon[i].points)
                #plot graphics view
                sp_pt=np.array(subpolygon[i].points)
                nn=len(sp_pt)
                poly =  QtGui.QPolygonF()
                for jj in range (nn):
                    poly.append(QtCore.QPointF(sp_pt[jj,0],sp_pt[jj,1]))
                self.scene.addPolygon(poly,self.pen_color[i],self.brush_color[i])
            self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            del polygon, subpolygon,subpolygon_atribute,sp_pt,poly #clear memory
            return poly_map,atribute
        # if error in shapefile
        else :
            QMessageBox.information(self,'Load Map','No Polygon shapefile')
            return 0,0

#Configure variable by keyboard
    def KeyFunc(self,variable):
        self.var=variable
        self.key = Keyboard() #buid keyboard windows and show
        self.key.show()
        self.key.pass_value.connect(self.receiveValue) #receive value pass by keybpard
#
    def receiveValue(self, value): #update the variable in Clicked line edit
        if '.' in value: value=float(value)
        elif value=='':value=0
        else : value=int(value)
        #        
        if self.var=='popseed' :
            self.seed_mode="FIX" #automaticly set seed mode to FIX
            self.popseed=value
            self.le_set_pop.setText(str(self.popseed))
        elif self.var=='seed_germ' : 
            self.seed_germ=value
            self.le_set_germ.setText(str(self.seed_germ))
        elif self.var=='row_spacing' : 
            self.row_spacing=value
            self.le_row_spacing.setText(str(self.row_spacing))
        elif self.var=='fert_rt' :
            self.fert_mode="FIX" #automaticly set fert mode to FIX
            self.fert_rt=value
            self.le_set_fertrt.setText(str(self.fert_rt))
        elif self.var=='n_machine_id' : 
            self.n_machine_id=value
            self.le_set_mach_id.setText(str(self.n_machine_id))
        elif self.var=='n_field_id' : 
            self.n_field_id=value
            self.le_set_fil_id.setText(str(self.n_field_id))
        elif self.var=='time_control' : 
            self.time_control=value
            self.le_set_t_ctrl.setText(str(self.time_control))
        elif self.var=='time_gps' : 
            self.time_gps=value
            self.le_set_t_gps.setText(str(self.time_gps))
        elif self.var=='dt_seed_cal' : 
            self.dt_seed_cal=value
            self.le_seed_dt.setText(str(self.dt_seed_cal))
            if self.dt_seed_cal<10.0:
                self.dt_seed_cal=0.0
                GPIO.output(pinEnable_Seed,GPIO.LOW)
            elif self.dt_seed_cal>100.0:self.dt_seed_cal=100.0
            GPIO.output(pinEnable_Seed,GPIO.HIGH)
            PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed_cal)
        elif self.var=='dt_fert_cal' : 
            self.dt_fert_cal=value
            self.le_fert_dt.setText(str(self.dt_fert_cal))
            if self.dt_fert_cal<10.0:
                self.dt_fert_cal=0.0
                GPIO.output(pinEnable_Fert,GPIO.LOW)
            elif self.dt_fert_cal>100.0:self.dt_fert_cal=100.0
            GPIO.output(pinEnable_Fert,GPIO.HIGH)
            PWM.set_duty_cycle(pinPWM_Fert,self.dt_fert_cal)
        elif self.var=='cal_fert_mass' : 
            self.fert_rt_cal=value
            self.le_fert_mass.setText(str(self.fert_rt_cal))
        elif self.var=='sim_speed' : 
            self.sim_speed=value
            self.le_sim_speed.setText(str(self.sim_speed))
        elif self.var=='fix_latitude' : 
            self.lat_fix=value
            self.le_set_latitude.setText(str(self.lat_fix))
        elif self.var=='fix_longitude' : 
            self.long_fix=value
            self.le_set_longitude.setText(str(self.long_fix))
#  
    def OffPop(self):
        self.popseed=0
        self.seed_mode="OFF"
        self.le_set_pop.setText("OFF")
        self.scene.clear()
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
    def OffFert(self):
        self.fert_rt=0
        self.fert_mode="OFF"
        self.le_set_fertrt.setText("OFF")
        self.scene.clear()
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
        #   
    # Incread and Decrease Population and Fert Ratio for a Dynamic Test
    def IncPopFert(self):
        if "FIX" in self.seed_mode:
            self.popseed=self.popseed+10000
            self.le_set_pop.setText(str(self.popseed))
        if "FIX" in self.fert_mode:
            self.fert_rt=self.fert_rt+100
            self.le_set_fertrt.setText(str(self.fert_rt))
#
#In Operation Monitoration
    def DefineID(self): # Define the Machine ID, Field ID and the logfilename
        self.machineID='M'+str(self.n_machine_id)
        self.fieldID='F'+str(self.n_field_id)
        self.logfile_name=os.path.join(self.dir,self.machineID+'_'+self.fieldID+'.txt')
        f=open(self.logfile_name,'a') #Creat the logfile with header//Niver delete file
        f.write("LogID,Log_Bt,Date,Time (UTC),MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),GPS PDOP ,PopSeed(Plant/ha),FertRt(kg/ha),FertWgt(kg),\
Mean Op Cap(m2/h),Time Operation (h),Area(m2),Row Spacing(m),Holes, seed_germ (%), SeedByM, FertByM, Seed Mode, Fert Mode,\
Remote Status,Duty Seed, Calc Rot Seed, Real Rot Seed\n")
        f.close()
        #Reset area and time operation and clear configuration
        self.area=0.0
        self.aux_bt=0
        self.lb_tb.setText(str(self.aux_bt))
        self.time_operation=0.0
        self.fertfile_name=""
        self.seedfile_name=""
        self.log_id=0
        self.seed_mode="OFF"
        self.fert_mode="OFF"
        
        self.scene.clear()
    #calibrate the Load Cell of Fert Tank
    def SaveCal(self): #Save the point for calibration.
        sum_wgt=0 #Read 20 values for voltage, calculate the mean, show in Line Edit and save in array
        for i in range(0,20):
            value=round(1.8*ADC.read(pinLoadCell),2)
            sum_wgt=sum_wgt+value
            time.sleep(0.1)
        self.wgt_voltage_cal[self.list_cal_wgt.currentIndex()]=round(sum_wgt/20,5)
        self.qd_voltage_cal.setPlainText(str(round(sum_wgt/20,5)))
        if (self.list_cal_wgt.currentIndex()==0):
            self.load_cell_zero=round(sum_wgt/20,5)
    #Calculate the calibration coeficiente m=a+b*voltage
    def Calibration(self):
        if self.wgt_voltage_cal[0]!=0 and self.wgt_voltage_cal[1]!=0 and self.wgt_voltage_cal[2]!=0 and self.wgt_voltage_cal[3]!=0: 
            mass = np.array([0.0,5.0,10.0,15.0]) #fix mass
            self.cal_a_cell,self.cal_b_cell,r_value,p_value,std_error=stats.linregress(self.wgt_voltage_cal,mass)
            self.cal_a_cell=round(self.cal_a,4)
            self.cal_b_cell=round(self.cal_b,4)
            QMessageBox.information(self,'Calibrate Load Cell','Sucess')
        else: QMessageBox.information(self,'Calibrate Load Cell','Error')
        print ("Load Cell",self.cal_a_cell,self.cal_b_cell,r_value,p_value,std_error)
#Calibrate Seed and Fert
    def CalSeed(self):
        print (self.filter_rot_seed,self.dt_seed_cal)
        f=open("cal_seed.txt","a")
        f.write(str(self.filter_rot_seed)+','+str(self.dt_seed_cal)+'\n')
        f.close()
#
    def CalFert (self):
        print (self.fert_rt_cal,self.dt_fert_cal)
        f=open("cal_fert.txt","a")
        f.write(str(self.fert_rt_cal)+','+str(self.dt_fert_cal)+'\n')
        f.close()
#
    def CalibrateSeed(self):
        #seed
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
    def CalibrateFert(self):
        #fert
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
    def GPSFunction(self):
        nmea1=gps.readline()
        nmea2=gps.readline()
        if GPIO.input(pinOnOffButton):
            try:
                nmea1=nmea1.decode('utf-8')
                nmea1=nmea1.split(',')
                nmea2=nmea2.decode('utf-8')
                nmea2=nmea2.split(',')
                self.date,self.time,self.lat_utm,self.long_utm,self.lat,self.long,self.status,self.pdop=operation.ReadGPS(nmea1,nmea2)
            except:
                self.ql_pdop.setPlainText("Error")
          
            if "MAP" in self.seed_mode:
                    self.popseed=operation.FindNeig(self.lat_utm,self.long_utm,self.map_seed,self.pop_values)
            if "MAP" in self.fert_mode:
                    self.fert_rt=operation.FindNeig(self.lat_utm,self.long_utm,self.map_fert,self.fert_values)
#    
    def LogFunction(self): # Function for generate a log file
        if GPIO.input(pinOnOffButton) and self.lat_utm >0:
            self.log_id=self.log_id+1
            #Local Logger
            string="Saving in: "+str(self.logfile_name)
            self.ql_logfile.setPlainText(string)
            data_string=str(self.log_id)+','+str(self.aux_bt)+','+self.date+','+self.time+","+self.machineID+","+self.fieldID+","+str(self.lat_utm)+","+\
str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+str(self.mach_speed)+","+str(self.pdop)+","+ \
str(self.popseed)+","+str(self.fert_rt)+","+str(self.fert_wgt)+","+str(self.opcap)+","+str(self.time_operation)+\
","+str(self.area)+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+","+str(self.fertbym)\
+","+str(self.seed_mode)+","+str(self.fert_mode)+","+self.rm_st+","+str(self.dt_seed)+","+str(self.rot_seed)+","+str(self.filter_rot_seed)+ "\n"
            f=open(self.logfile_name,'a')
            f.write(data_string)
            f.close()
        else:self.ql_logfile.setPlainText("Not saving")
#
# Read Speed from Arduino and process
    def SpeedFunction(self):

            #Detect if button is pressed
            self.st_bt=GPIO.input(pinUpDyn)
            if self.st_bt==1 and  self.last_st_bt==0:
                self.aux_bt=self.aux_bt+1
                self.lb_tb.setText("")
            self.last_st_bt=self.st_bt
            

            #speed
            vel=ino.readline()
            try:
                vel=vel.decode('utf-8')
                st_char,self.mach_speed,self.real_rot_seed,end_char=vel.split(',')
                self.mach_speed=float(self.mach_speed)
                self.real_rot_seed=float(self.real_rot_seed)
            except:
                self.ql_speed.setPlainText("Error")
                pass
            #seed
            if self.real_rot_seed>0.1:self.array=np.append(self.array,self.real_rot_seed)
            if len(self.array)==20:self.array=np.delete(self.array,0)
            if len(self.array)>0: self.filter_rot_seed=round(np.median(self.array),2)
            if self.real_rot_seed<0.1 or self.change_popseed is True:
                self.array=[]
                self.filter_rot_seed=0
            #speed
            if self.mach_speed>0.3:self.array2=np.append(self.array2,self.mach_speed)
            if len(self.array2)==4:self.array=np.delete(self.array2,0)
            if len(self.array2)>0: self.filter_mach=round(np.mean(self.array2),2)
            if self.mach_speed<=0.3:
                self.array2=[]
                self.filter_mach=0
            self.mach_speed= round(self.filter_mach,2)
           



                
###
###
# Function that control the seed and fert application motor            
    def ControlFunction(self):
        #
        if GPIO.input(pinOnOffButton):
            
            #
            if self.status=='A': #plot in graph the actual position
                self.scene.addRect(self.lat_utm,self.long_utm,0.5,0.5,self.Gpen,self.Gbrush)
                self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
            #tank mass
            self.fert_wgt=operation.ReadWeight(self.cal_a_cell,self.cal_b_cell) #Read Fert Weight
            self.disk_hole=int(self.list_holes.currentText()) #Read Hole Disk
            ###
            ###
            ###Seeder Distributor###
            #check if population change or duty cicle change ==> reset encoder seed speed variable
            if self.popseed!=self.last_popseed or self.dt_seed_cal!=self.last_dt_seed_cal:
                self.change_popseed=True
            else :self.change_popseed=False
            self.last_popseed=self.popseed
            self.last_dt_seed_cal=self.dt_seed_cal
            #Calcute and Control Speed Motor
            if self.dt_seed_cal < 10: #if test function is not active
                self.rot_seed,self.seedbym=operation.Seeder(self.mach_speed,self.popseed,self.row_spacing,self.disk_hole,self.seed_germ)
                self.dt_seed=operation.ControlSpeedSeed(self.change_popseed,self.rot_seed,self.filter_rot_seed,self.cal_a_seed,self.cal_b_seed)
            '''
            #check if motor is blocked
            if (self.dt_seed > 35 or self.dt_seed_cal > 35) and self.filter_rot_seed < 0.1 :
                self.aux_block=self.aux_block+1
                print (self.aux_block)
            if self.aux_block >5: #motor is blocked
                QMessageBox.information(self,'Motor','Blocked!')
                PWM.set_duty_cycle(pinPWM_Seed,100)
                time.sleep(1)
                PWM.set_duty_cycle(pinPWM_Seed,self.dt_seed)
                self.aux_block=0
            if self.change_popseed is True:self.aux_block=0
            '''
            ##
            ####Fertilizer Distribution###
            #check if fertilizer ratio change ==> for use in dynamic calibration (future development)
            if self.fert_rt!=self.last_fert_rt:
                self.change_fertrt=True
            else :self.change_fertrt=False
            self.last_fert_rt=self.fert_rt
            # 
            #Calcute and Control Speed Motor
            if self.dt_fert_cal < 10:
                self.fertbym,self.fertbys=operation.Fert(self.mach_speed,self.fert_rt,self.row_spacing)
                self.dt_fert=operation.ControlSpeedFert(self.cal_a_fert,self.cal_b_fert,self.fertbys)
            #calculate operational capability and area 
            #
            self.inst_area=self.row_spacing*self.mach_speed*(self.time_control/1000) #in m2
            self.area=round((self.area+self.inst_area)/1000,3) #in ha
            self.inst_time=round(self.time_control/3600.0/1000,5) #in h
            self.time_operation=round(self.time_operation+self.inst_time,5)   # in h
            self.opcap=round(self.area/(self.time_operation),3) # in ha/h
            # Update LineEdit in main tab
            self.ql_speed.setPlainText(str(self.mach_speed))
            self.ql_seed.setPlainText(str(self.popseed))
            if self.status=='A':self.ql_pdop.setPlainText(str(self.pdop))
            else:self.ql_pdop.setPlainText('No Signal')
            self.ql_fert_rt.setPlainText(str(self.fert_rt))
            self.ql_fert_wgt.setPlainText(str(self.fert_wgt))
            self.ql_area.setPlainText(str(self.area))
            self.ql_opcap.setPlainText(str(self.time_operation)) #it's time operation
            if not "OFF" in self.seed_mode:self.lb_status.setText('C'+str(self.rot_seed)+' R:'+str(self.filter_rot_seed)+'DT:'+str(self.dt_seed))
            elif not "OFF" in self.seed_mode and self.dt_seed<40.0: self.lb_status.setText("Low speed in Seed Motor")
            else : self.lb_status.setText("")
            if not "OFF" in self.fert_mode and self.dt_fert<40.0: self.lb_status.setText("Low speed in Fert Motor")
            self.lb_datetime.setText(self.time)
            #in Calibrate
            self.ql_speed_cal.setPlainText(str(self.filter_rot_seed))

            # 3G Send Data
            dt=int(2000/self.time_control) #  5*dt/time_control must  10 s
            if self.aux==dt and  "ON" in self.st_remote:
                sim800l.write(str.encode('AT+SAPBR=3,1,\"Contype\",\"GPRS\"'+'\r'))
            if self.aux==4*dt and  "ON" in self.st_remote:
                sim800l.write(str.encode('AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"'+'\r'))
                self.ql_remote_status.setPlainText("Starting..")
            if self.aux==6*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+SAPBR=1,1'+'\r'))
            if self.aux==8*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+SAPBR=2,1'+'\r'))
            if self.aux==10*dt and  "ON" in self.st_remote:
                sim800l.write(str.encode('AT+HTTPINIT'+'\r'))
                self.ql_remote_status.setPlainText("HTTP INIT")
            if self.aux==12*dt and  "ON" in self.st_remote: sim800l.write(str.encode('AT+HTTPPARA=\"CID\",1'+'\r'))
            link='http://andrecoelho.tech/SemeaView/send_mysql.php?'
            str_data='LogID='+str(self.log_id)+'&Date='+str(self.date)+'&Time='+str(self.time)+'&MachineID='+self.machineID\
+'&FieldID='+self.fieldID+'&Lati='+str(self.lat)+'&Longi='+str(self.long)+'&XUtm='+str(self.lat_utm)+'&YUtm='+str(self.long_utm)+'&Speed='+\
str(self.mach_speed)+'&OpCap='+str(self.opcap)+'&TimeOperation='+str(self.time_operation)+'&Population='+str(self.popseed)+'&FertRatio='+\
str(self.fert_rt)+'&FertWgt='+str(self.fert_wgt)+'&Area='+str(self.area)
            if self.aux==14*dt and  "ON" in self.st_remote and self.start_remote==QMessageBox.Yes:
                sim800l.write(str.encode('AT+HTTPPARA=\"URL\",'+link+str_data+'\r'))
            if self.aux==17*dt and  "ON" in self.st_remote and self.start_remote==QMessageBox.Yes:
                sim800l.write(str.encode('AT+HTTPACTION=0'+'\r'))
                self.aux=12*dt
            rec=sim800l.readline()
            rec=rec.decode('utf-8')
            print (rec)
            if '+SAPBR:' in rec: self.start_remote=QMessageBox.question(self,'3G',rec)
            if '+HTTPACTION:' in rec:
                if "200" in rec :
                    self.error=0
                    self.rm_st='ON'
                    self.ql_remote_status.setPlainText("Sucess")
                if not "200" in rec :
                    self.error=self.error+1
                    self.rm_st='ERROR'
                    self.ql_remote_status.setPlainText("Error")
            if "OFF" in self.st_remote:
                    self.rm_st='OFF'
                    self.aux=0
                    self.ql_remote_status.setPlainText("Disable")
                    sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
            if self.error==5:
                    self.error=0
                    self.rm_st="RESET"
                    sim800l.write(str.encode('AT+SAPBR=0,1'+'\r'))
                    self.aux=0 #reset 3G
                    QMessageBox.information(self,'3G','Error')
            self.aux=self.aux+1

            
            #For test of change popseed by location specification
            if self.lat_fix>0 and self.long_fix>0 : #lat and long it is defined by click line edit
                utm_conv=utm.from_latlon(-self.lat_fix,-self.long_fix) #- because it in Brazil
                self.lat_utm_fix=float(utm_conv[0])
                self.long_utm_fix=float(utm_conv[1])
                self.scene.addRect(self.lat_utm_fix,self.long_utm_fix,0.5,0.5,self.Kpen,self.Kbrush)
                #calc dist between fix point and actual point
                self.dist=math.sqrt(math.pow(self.lat_utm_fix-self.lat_utm,2)+math.pow(self.long_utm_fix-self.long_utm,2))
                self.lb_status.setText(str(self.dist))
                if self.dist>self.last_dist and self.st_increase is False and self.dist<2.5: #if it's distance increase (no nearst)
                    self.popseed=self.popseed+10000 #increase the actual population
                    self.le_set_pop.setText(str(self.popseed))
                    self.ql_seed.setPlainText(str(self.popseed))
                    self.st_increase=True
                self.last_dist=self.dist #update dist            

        else: # If button is off
            GPIO.output(pinEnable_Seed,GPIO.LOW)
            GPIO.output(pinEnable_Fert,GPIO.LOW)
            self.lb_status.setText("Disable")
            self.ql_speed.setPlainText("")
            self.ql_seed.setPlainText("")
            self.ql_pdop.setPlainText("")
            self.ql_fert_rt.setPlainText("")
            self.ql_fert_wgt.setPlainText("")
            self.ql_area.setPlainText("")
            self.ql_opcap.setPlainText("")
            self.lb_datetime.setText("")



class Keyboard(QtWidgets.QMainWindow,Ui_KeyBoard): #See in keyoard
    pass_value = QtCore.pyqtSignal(str)
    def __init__(self,parent=None):
        super(Keyboard,self).__init__(parent)
        self.setupUi(self)
        #set number as '' and 
        self.number=''
        self.plainTextEdit.setPlainText(self.number) #display null
        #functions
        self.btn_ok.clicked.connect(self.Enter) #button ok
        self.btn_cls.clicked.connect(self.Clear) #clear plainText
        self.btn_0.clicked.connect(lambda : self.addNumber ('0'))
        self.btn_1.clicked.connect(lambda : self.addNumber ('1'))
        self.btn_2.clicked.connect(lambda : self.addNumber ('2'))
        self.btn_3.clicked.connect(lambda : self.addNumber ('3'))
        self.btn_4.clicked.connect(lambda : self.addNumber ('4'))
        self.btn_5.clicked.connect(lambda : self.addNumber ('5'))
        self.btn_6.clicked.connect(lambda : self.addNumber ('6'))
        self.btn_7.clicked.connect(lambda : self.addNumber ('7'))
        self.btn_8.clicked.connect(lambda : self.addNumber ('8'))
        self.btn_9.clicked.connect(lambda : self.addNumber ('9'))
        self.btn_dot.clicked.connect(lambda : self.addNumber ('.'))
 
    def Clear(self):
        self.number=''
        self.plainTextEdit.setPlainText(self.number)
    
    def Enter(self):
        gui.key.close()
        self.pass_value.emit(self.number)
        
    def addNumber(self,n):
        self.number=self.number+n #build the number as a string
        self.plainTextEdit.setPlainText(self.number)
        
 #Run the app:
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    gui = Semea()
    gui.show()
    sys.exit(app.exec_())

