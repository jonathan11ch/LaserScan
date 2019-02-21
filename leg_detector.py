#Author: Juan Lara
import hokuyo.hokuyo as hokuyo
import hokuyo.lib.serial_port as serial_port
import sensors
import serial
import time
import logging
import threading
import numpy as np
#import matplotlib.pyplot as plt
import scipy.signal as spy
import scipy.io
import warnings
import os
warnings.filterwarnings("ignore")
#%%
logging.basicConfig(level = logging.DEBUG, format = '[%(levelname)s] (%(threadName)-9s) %(message)s',)
# the [HokuyoLRF] class allows to initialize, acquire data and close the LRF sensor.
class HokuyoLRF(object):
    def __init__(self,uart_speed = 115200,uart_port = "/dev/hokuyo",star_step = 300,stop_step = 468):

        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_port.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.star_step = star_step
        self.stop_step = stop_step
        logging.debug("baudrate: "+ str(uart_speed))

    def laser_on(self):
        self.laser.laser_on()

    def laser_off(self):
        self.laser.laser_off()
    def read_one_scan(self):
        return self.laser.get_single_scan(self.star_step, self.stop_step, 1)
#%% 
class LegDetector(object):
    def __init__(self, port = "COM25",name_csv = "Lrf_Data.csv",sample = 0.05):
        self.go_on=True
        self.__pause = False
	#Sample time (default=0.25s)
        self.__Ts=sample
        #File name
        self.__file=name_csv
        #Hokuyo object
        self.laser=HokuyoLRF(uart_port=port)
        #Initialize LRF
        self.laser.laser_on()
        #%% function to acquire a single scan (takes around 0.1s)
        #self.computer_serial=serial.Serial(port=port_pc,baudrate=4800,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
        self.cont=0
        self.t1=threading.Thread(target=self.laser_timer)
        self.t1.start()
        time.sleep(5)
        self.t2=threading.Thread(target=self.process)
        self.t2.start()
        
        self.LDD_tot=np.array([])
        self.LDD=np.array([])
        self.max_time=np.array([])
        self.cadence=np.array([])
        self.speed=np.array([])
        self.max_point=np.array([])
        self.min_point=np.array([])
        self.f=open('data_leg_detector.csv','w',os.O_NONBLOCK)
        
        
        
    def laser_timer(self):
        self.cont=0
        while self.go_on:
            #print "*" * 20
            #print "cont increment: " + str(self.cont)
            #print "*" * 20
            self.cont=self.cont+0.05
            time.sleep(0.05)
    def pause(self):
        self.__pause=True
    def play(self):
        self.__pause=False
    def process(self):
        
        while self.go_on:
            if not self.__pause:
                print "habemus data"
                #Reading one scan from LRF
                self.data_temp=self.laser.read_one_scan()
                #If there is no data then continue
                if not self.data_temp:
                    print "no data from Hokuyo, Reading again..."
                    continue
                #Storing data as a numpy array
                self.data=np.array(self.data_temp)
                #Calling the legs segmentation method.
                self.legs_segmentation()
                if self.LDD.size<1 or self.cadence.size<1 or self.speed.size<1:
                    continue
                print "writing file" + str(self.LDD[-1])+','+str(self.cadence[-1])+','+str(self.speed[-1])
                self.f.write(str(self.LDD[-1])+','+str(self.cadence[-1])+','+str(self.speed[-1])+'\n')
                self.f.flush()
                time.sleep(0.01)
            else:
                time.sleep(1)
    def legs_segmentation(self):
        #Storing organized theta data
        self.theta=self.data[:,0]        
        self.theta=self.theta[::-1]
        #Storing organized radio data
        self.radio=self.data[:,1]
        self.radio=self.radio[::-1]
        self.radio[self.radio>1600]=0
        self.radio[self.radio<30]=1600
        self.x_scan,self.y_scan=self.polar_xyz(self.theta,self.radio)
        self.leg_indx=np.argmin(self.y_scan)
        if self.LDD_tot.size<3:
            self.LDD_tot=np.append(self.LDD_tot,self.y_scan[self.leg_indx])
            return
        else:
            self.LDD_tot=np.append(self.LDD_tot,self.y_scan[self.leg_indx])
            self.LDD_tot=np.delete(self.LDD_tot,0)
        if self.LDD_tot[0]<self.LDD_tot[1] and self.LDD_tot[2]<self.LDD_tot[1]:
            print "max"
            
            if self.max_point.size<2:
                self.max_point=np.append(self.max_point,self.LDD_tot[1])
                self.max_time=np.append(self.max_time,self.cont)
            
            else:
                self.max_point=np.append(self.max_point,self.LDD_tot[1])
                self.max_point=np.delete(self.max_point,0)
                self.max_time=np.append(self.max_time,self.cont)
                self.max_time=np.delete(self.max_time,0)
            #print "########" + str(self.max_time[-2:-1])
            print self.cont
                       
            
        elif self.LDD_tot[0]>self.LDD_tot[1] and self.LDD_tot[2]>self.LDD_tot[1]:
            print "min"
            if self.min_point.size<2:
                self.min_point=np.append(self.min_point,self.LDD_tot[1])
            else:
                self.min_point=np.append(self.min_point,self.LDD_tot[1])
                self.min_point=np.delete(self.min_point,0)
        if self.max_point.size<2 or self.min_point.size<2:
            return
        if self.LDD.size<5:
            self.LDD=np.append(self.LDD,(self.max_point[-1]-self.min_point[-1])*2)
            self.cadence=np.append(self.cadence,1/(2*(self.max_time[-1]-self.max_time[-2])))
            self.speed=np.append(self.speed,self.LDD[-1]*self.cadence[-1]*36/10000)
        else:
            temp=self.LDD[-10:-1]
            temp=np.append(temp,(self.max_point[-1]-self.min_point[-1])*2)
            temp2=self.cadence[-10:-1]
            temp2=np.append(temp2,1/(2*(self.max_time[-1]-self.max_time[-2])))
            self.LDD=np.append(self.LDD,np.mean(temp))
            self.LDD=np.delete(self.LDD,0)
            self.cadence=np.append(self.cadence,np.mean(temp2))
            self.cadence=np.delete(self.cadence,0)
            self.speed=np.append(self.speed,self.LDD[-1]*self.cadence[-1]*36/10000)
            self.speed=np.delete(self.speed,0)
    def get_data(self):
        return self.LDD[-1],self.cadence[-1],self.speed[-1]
    def close(self):
        self.go_on=False
        self.laser.laser_off()
        self.laser.laser.terminate()
        time.sleep(0.5)
        self.f.close()
        #%% The following method changes the corrdinate system from polar to cartesian.
        #The method requires a theta and a radio value
    def polar_xyz(self,theta,radio):
        return radio*np.cos(theta*np.pi/180),radio*np.sin(theta*np.pi/180)
if __name__=='__main__':
    a=LegDetector(port="/dev/ttyACM0")
    #a.process()
    a.play()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        a.close()
    
##    
##    for i in range(30):
##        time.sleep(1)
##        print('Time is: '+str(i))
##        print "data is: "+str(a.get_data())
##        print "*"*20
##    a.close()
##
