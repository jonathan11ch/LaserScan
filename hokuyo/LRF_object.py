## -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries   

import serial
import hokuyo
import serial_port
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading

## -- Laser range finder scan object -- Laser range finder scan object -- Laser range finder scan object

class LRF(object):
    def __init__(self,uart_speed = 9800,uart_port = "COM1",star_step = 44,stop_step = 725):
      
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_port.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.star_step = star_step
        self.stop_step = stop_step

    def laser_on(self):
        self.laser.laser_on()

    def laser_off(self):
        self.laser.laser_off()
        
    def read_one_scan(self):
        return self.laser.get_single_scan(self.star_step, self.stop_step, 1)

## -- Port LRF detection -- Port LRF detection -- Port LRF detection -- Port LRF detection 

def Find_port_URG_laser():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    Found = False 
    for x in ports:
        if x[1].find("URG Series") == 0:
            Laser_port = x[0]
            Found = True
            break

    if Found == True:
        return Laser_port
    else:
        print ("didn't find laser port")
        return "Error"

## -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction
    
def animate(i):

    global Laser_lec
    
    theta = np.array([x[0]*np.pi/180 for x in Laser_lec])
    radius = np.array([x[1] for x in Laser_lec])
    
    ax.clear()
    ax.plot(theta, radius, color='b', linewidth=1)
    
    ax.set_rmax(2000)
    ax.grid(True)
    plt.title("LRF scan", va='bottom')

## -- Thread for LRF Lecture

def LRF_Lecture():
    
    global Laser_lecture,Laser_obj,Laser_lec,Time_LRF

    while Laser_lecture:
        Laser_lec = Laser_obj.read_one_scan()
        Time_LRF.append(time.time())
        Laser_scan_regitry.append(Laser_lec)

    print("Laser off")
    

# -- Global variables -- Global variables -- Global variables -- Global variables

Laser_obj = None # Object to manipulate the Hokuyo URG scan laser
Laser_lec = [] # List that contains the laser's samples
Laser_lecture = False # Variable for begin or finish the lecture of the
Time_LRF = [] # List that contains the time of each sample of the LRF
Laser_scan_regitry = [] # List that each list scan 

## -- Program's Main -- Program's Main -- Program's Main -- Program's Main -- Program's Main
   
if __name__ == "__main__":
    
    LRF_port = Find_port_URG_laser()
    start = 280
    stop = 488
    serial_comunication_speed = 256000

    if LRF_port != "Error":

        #global Laser

        # LRF object declaration
        
        Laser_obj = LRF(serial_comunication_speed,LRF_port,start,stop)
        Laser_obj.laser_on()

        # Thead LRF lecture begins
        
        Laser_lecture = True
        try:
            thread_LRF_lecture = threading.Thread(target=LRF_Lecture)
            thread_LRF_lecture.daemon = True
            thread_LRF_lecture.start()
        except:
            Laser_obj.laser_off()
            print("thread LRF lecture does not work")
        
        # Initialize the figure to plot
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')
        ani= animation.FuncAnimation(fig, animate, interval=1)

        # Show the animate plot
        
        try:
            fig.show()
        except:
            Laser_lecture = False
            Laser_obj.laser_off()

    else:
        print("The program doesn't work")
    
            
