#Version of software used in Field Test (May, 2019)
#Software developed to seed variable rate application controller
#It is using Python 3.7
#The GUI it is build by PyQt5 tools
#This software was developed for BeagleBone Black single board computer, using oficial Linux Debian 8.6
#The controller of GPIO and PWM pins was executed by Adafruit_BBIO Python module
#The GUI it is prepared by Fertilizer Spreader at variable rate aplication. So this version of software don't control the electric motor of fertilizer spreader
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
# import other python files of software
from seeder_ui import Ui_SEMEA #GUI
from keyboard import Ui_KeyBoard #Virtual Keyboard
import operation #Auxiliar Script Files
#
import serial
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
#Pins Configuration
#Button OnOff  (Enable)
pinOnOffButton="P8_16"
GPIO.setup(pinOnOffButton, GPIO.IN)
#Auxiliar Button
pinUpDyn="P9_12"
GPIO.setup(pinUpDyn, GPIO.IN)
#PWM Seed
pinPWM_Seed="P8_13"
PWM.start(pinPWM_Seed,0, 1000.0) #pin, duty,frequencia
pinEnable_Seed="P8_10"
GPIO.setup(pinEnable_Seed, GPIO.OUT)
GPIO.output(pinEnable_Seed,GPIO.LOW)
#Ublox NEO 6M UART comunication
gps = serial.Serial ("/dev/ttyS4", 9600,timeout=0.05) # P9_11 P9_13
#The software use only GPRMC and GPGSA NMEA sentences
#The module it is configure for not send the other NMEA senteces
msg=["$PUBX,40,GLL,0,0,0,0*5C\r\n","$PUBX,40,VTG,0,0,0,0*5E\r\n","$PUBX,40,GSV,0,0,0,0*59\r\n","$PUBX,40,GGA,0,0,0,0*5A\r\n"]
for i in msg : gps.write(i.encode())
#
#Arduino UART comunication configuration
ino = serial.Serial("/dev/ttyS2", 9600,timeout=0.05) # P9_21 P9_22
#
#Class of GUI
class Semea(QtWidgets.QTabWidget,Ui_SEMEA):
    def __init__(self,parent=None):
        super(Semea,self).__init__(parent)
        self.setupUi(self)
        #Graphics View Configuration
        self.scene=QtWidgets.QGraphicsScene()
        self.gv.scale(1,-1)
        self.gv.setScene(self.scene)
        #Color for Graphics View
        self.Mpen,self.Mbrush=QtGui.QPen(QtCore.Qt.magenta),QtGui.QBrush(QtCore.Qt.magenta)
        self.Ypen,self.Ybrush=QtGui.QPen(QtCore.Qt.yellow),QtGui.QBrush(QtCore.Qt.yellow)
        self.Rpen,self.Rbrush=QtGui.QPen(QtCore.Qt.red),QtGui.QBrush(QtCore.Qt.red)
        self.Bpen,self.Bbrush=QtGui.QPen(QtCore.Qt.blue),QtGui.QBrush(QtCore.Qt.blue)
        self.Gpen,self.Gbrush=QtGui.QPen(QtCore.Qt.green),QtGui.QBrush(QtCore.Qt.green)
        self.Kpen,self.Kbrush=QtGui.QPen(QtCore.Qt.black),QtGui.QBrush(QtCore.Qt.black)
        self.pen_color=[self.Mpen,self.Ypen,self.Rpen,self.Bpen]
        self.brush_color=[self.Mbrush,self.Ybrush,self.Rbrush,self.Bbrush]
        #Global variables declaration
        self.popseed,self.area,self.opcap,self.time_operation='',0.0,0.0,0.0 #operation variables
        self.inst_opcap,self.inst_area,self.inst_time=0,0,0 #operation variables
        self.time,self.date,self.lat_utm,self.long_utm,self.lat,self.long,self.status,self.pdop='','',0,0,0.0,0,0,0 #gps variables
        self.row_spacing,self.disk_hole,self.seed_germ,self.logfile_name=0,0,0,"" #operation variables
        self.map_seed,self.pop_values=0,0 #map seed
        self.mach_speed=0
        self.seed_mode,self.seedbym="OFF",0, #operation modes
        self.rot_seed,self.real_rot_seed=0.0,0.0 #seed speed calculated and measured
        self.dt_seed_cal=0 #variables for test and calibrate speed
        self.change_popseed=False #variables for dynamic test and check if population changet
        self.last_popseed,s=0 #for ckeck if population (plant density) change
        self.dt_seed,self.last_dt_seed_cal=-1,-1  #for ckeck if population  change
        self.n_machine_id,self.n_field_id=1,1 #number auxiliar for setting machine and field id
        self.lat_used,self.long_used=0,0 #lat e long used in Map aplication
        self.aux,self.error=0,0#, log and gps
        self.var='' #aux to keyboard function
        self.array,self.filter_rot_seed=[],0 #filter rot seed speed
        self.st_increase=False
        self.array2,self.filter_mach=[],0
     
        #Read the last configuration used in software
		#This configuration it is used in this software start
        self.dir=os.path.dirname(os.path.abspath(__file__))  
        self.conffile_name=os.path.join(self.dir,"conf.txt")
        with open(self.conffile_name, "r",encoding='latin-1') as f:
            self.seed_mode=(f.readline()) #seed mode
            self.seedfile_name=f.readline() # sowing map filename
            self.popseed=float(f.readline()) #last plant density (for fix mode)
            self.seed_germ=int(f.readline()) #seed germination percentage
            self.row_spacing=float(f.readline()) #row spacing
            self.logfile_name=f.readline() #log file name
            self.machineID=f.readline() 
            self.fieldID=f.readline()
            self.cal_a_seed=float(f.readline()) #calibration for seed spped
            self.cal_b_seed=float(f.readline()) #calibration for seed spped
            self.area=float(f.readline()) #
            self.time_operation=float(f.readline())
            self.log_id=int(f.readline())
            self.time_control=int(f.readline())
            self.time_gps=int(f.readline())
        f.close()
        #Setup the operation, with configuration of last use
        self.seed_mode=self.seed_mode.rstrip()
        self.seedfile_name=self.seedfile_name.rstrip()
        self.logfile_name=self.logfile_name.rstrip()
        self.machineID=self.machineID.rstrip()
        self.fieldID=self.fieldID.rstrip()
        self.le_set_mach_id.setText(str(self.machineID))
        self.le_set_fil_id.setText(str(self.fieldID))
        self.le_row_spacing.setText(str(self.row_spacing))
        self.le_set_germ.setText(str(self.seed_germ))
        self.le_set_pop.setText(str(self.popseed))
        self.le_set_t_ctrl.setText(str(self.time_control))
        self.le_set_t_gps.setText(str(self.time_gps))
        self.lb_tb.setText("")
        # Reload the Sowing Map, when map mode is active
        if "MAP" in self.seed_mode: self.map_seed,self.pop_values=self.LoadMap(self.seedfile_name)
        #Timers configuration
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
		# send messagem to Arduino with time interval to send planter speed and angular velocity
        s='500' #equal self.time_control
        ino.write(s.encode())
        #Main
        self.exit.clicked.connect(self.Close)  #exit button
        #Operation
        self.bt_load_seed.clicked.connect(self.LoadSeedFile) # load seed map
        self.pop_off.clicked.connect(self.OffPop)
        #LineEdit Clickable
        self.le_set_pop.clicked.connect(lambda : self.KeyFunc ('popseed')) #variable to update
        self.le_set_germ.clicked.connect(lambda : self.KeyFunc ('seed_germ')) #variable to update
        self.le_row_spacing.clicked.connect(lambda : self.KeyFunc ('row_spacing')) #variable to update
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
        #        
        self.save_cal_seed.clicked.connect(self.CalSeed)
        self.cal_seed.clicked.connect(self.CalibrateSeed)
#
#        
###
#Functions
# Im main Tab
    def Close(self): #save configuration inf file, clear GPIO, stop timer and close the software
        with open(self.conffile_name, "w",encoding='latin-1') as f:
            f.write(self.seed_mode+"\n")
            f.write(self.seedfile_name+"\n")
            f.write(str(self.popseed)+"\n")
            f.write(str(self.seed_germ)+"\n")
            f.write(str(self.row_spacing)+"\n")
            f.write(str(self.st_remote)+"\n")
            f.write(self.logfile_name+"\n")
            f.write(self.machineID+"\n")
            f.write(self.fieldID+"\n")
            f.write(str(self.cal_a_seed)+"\n")
            f.write(str(self.cal_b_seed)+"\n")
            f.write(str(self.area)+"\n")
            f.write(str(self.time_operation)+"\n")
            f.write(str(self.log_id)+"\n")
            f.write(str(self.time_control)+"\n")
            f.write(str(self.time_gps)+"\n")
        f.close()
        GPIO.cleanup()
        PWM.cleanup()
        s='0'
        ino.write(s.encode()) #send message to Arduino for stop send data
        self.close()
#
#In Setup Operation
     # Setting the map file name
    def LoadSeedFile(self):
        self.seedfile_name=QFileDialog.getOpenFileName(self,"Open Seed File","/root/SemeaPy5/Maps","*.shp")[0]
        self.map_seed,self.pop_values=self.LoadMap(self.seedfile_name) #polygon map vertices and polygon atributes
        self.seed_mode="MAP" #automaticly set seed mode to MAP
        self.le_set_pop.setText("MAP")
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
#Configure variable using the keyboard
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
	#
	#Turn seed metering device off  
    def OffPop(self):
        self.popseed=0
        self.seed_mode="OFF"
        self.le_set_pop.setText("OFF")
        self.scene.clear()
        self.gv.fitInView(self.scene.sceneRect(),QtCore.Qt.KeepAspectRatio)
#    
#
#In Operation Monitoration
	# Function Called when Define ID button it is clicjked
    def DefineID(self): # Define the Machine ID, Field ID and the logfilename
        self.machineID='M'+str(self.n_machine_id)
        self.fieldID='F'+str(self.n_field_id)
        self.logfile_name=os.path.join(self.dir,self.machineID+'_'+self.fieldID+'.txt')
        f=open(self.logfile_name,'a') #Creat the logfile with header//Niver delete file
        f.write("LogID,Log_Bt,Date,Time (UTC),MachineID,FieldID,LatUTM(m),LongUTM(m),Lat(º),Long(º),Speed (m/s),GPS PDOP ,PopSeed(Plant/ha),\
Mean Op Cap(m2/h),Time Operation (h),Area(m2),Row Spacing(m),Holes, seed_germ (%), SeedByM, Seed Mode, Duty Seed, Calc Rot Seed, Real Rot Seed\n")
        f.close()
        #Reset area and time operation and clear configuration
        self.area=0.0
        self.lb_tb.setText(str(self.aux_bt))
        self.time_operation=0.0
        self.seedfile_name=""
        self.log_id=0
        self.seed_mode="OFF"
        self.scene.clear()
#  
#Calibrate the duty cicle as a function of angular velocity of electric motor
    def CalSeed(self):
        print (self.filter_rot_seed,self.dt_seed_cal)
        f=open("cal_seed.txt","a")
        f.write(str(self.filter_rot_seed)+','+str(self.dt_seed_cal)+'\n')
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
#
# Functions of the timers (timeouts)            
#   Read the NMEA sentencs each time
    def GPSFunction(self):
		# read the NMEA two times to receive GPGSA and GPRMC
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
			# if sowing map it is enable, identify the plant density for the actual
			#managent zone
            if "MAP" in self.seed_mode:
                    self.popseed=operation.FindZone(self.lat_utm,self.long_utm,self.map_seed,self.pop_values)
            
#    # Function for generate a log file
    def LogFunction(self): 
        if GPIO.input(pinOnOffButton) and self.lat_utm >0:
            self.log_id=self.log_id+1
            #Local Logger
            string="Saving in: "+str(self.logfile_name)
            self.ql_logfile.setPlainText(string)
            data_string=str(self.log_id)+','+str(self.aux_bt)+','+self.date+','+self.time+","+self.machineID+","+self.fieldID+","+str(self.lat_utm)+","+\
str(self.long_utm)+","+str(self.lat)+","+str(self.long)+","+str(self.mach_speed)+","+str(self.pdop)+","+ \
str(self.popseed)+","+str(self.opcap)+","+str(self.time_operation)+\
","+str(self.area)+","+str(self.row_spacing)+","+str(self.disk_hole)+","+str(self.seed_germ)+","+str(self.seedbym)+)\
+","+str(self.seed_mode)+","+self.rm_st+","+str(self.dt_seed)+","+str(self.rot_seed)+","+str(self.filter_rot_seed)+ "\n"
            f=open(self.logfile_name,'a')
            f.write(data_string)
            f.close()
        else:self.ql_logfile.setPlainText("Not saving")
#
# Read planter speed and angular velocity send by Arduino
    def SpeedFunction(self):
			# a filter was  
            vel=ino.readline()
            try:
                vel=vel.decode('utf-8')
                st_char,self.mach_speed,self.real_rot_seed,end_char=vel.split(',')
                self.mach_speed=float(self.mach_speed)
                self.real_rot_seed=float(self.real_rot_seed)
            except:
                self.ql_speed.setPlainText("Error")
                pass
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
          ###
            ###
            ###Seed Metering Device###
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
           
            #calculate operational capability and area 
            #
            self.inst_area=self.row_spacing*self.mach_speed*(self.time_control/1000) #in m2
            self.area=round((self.area+self.inst_area)/1000,3) #in ha
            self.inst_time=round(self.time_control/3600.0/1000,5) #in h
            self.time_operation=round(self.time_operation+self.inst_time,5)   # in h
            self.opcap=round(self.area/(self.time_operation),3) # in ha/h
			#
            # Update LineEdit in main tab
            self.ql_speed.setPlainText(str(self.mach_speed))
            self.ql_seed.setPlainText(str(self.popseed))
            if self.status=='A':self.ql_pdop.setPlainText(str(self.pdop))
            else:self.ql_pdop.setPlainText('No Signal')
            self.ql_area.setPlainText(str(self.area))
            self.ql_opcap.setPlainText(str(self.time_operation)) #it's time operation
            if not "OFF" in self.seed_mode:self.lb_status.setText('C'+str(self.rot_seed)+' R:'+str(self.filter_rot_seed)+'DT:'+str(self.dt_seed))
            elif not "OFF" in self.seed_mode and self.dt_seed<40.0: self.lb_status.setText("Low speed in Seed Motor")
            else : self.lb_status.setText("")
            self.lb_datetime.setText(self.time)
            #in Calibrate
            self.ql_speed_cal.setPlainText(str(self.filter_rot_seed))

        else: # If button is off
            GPIO.output(pinEnable_Seed,GPIO.LOW)
            GPIO.output(pinEnable_Fert,GPIO.LOW)
            self.lb_status.setText("Disable")
            self.ql_speed.setPlainText("")
            self.ql_seed.setPlainText("")
            self.ql_pdop.setPlainText("")
            self.ql_area.setPlainText("")
            self.ql_opcap.setPlainText("")
            self.lb_datetime.setText("")


#Class of Virtual Numeric Keyboard 
class Keyboard(QtWidgets.QMainWindow,Ui_KeyBoard): #
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

