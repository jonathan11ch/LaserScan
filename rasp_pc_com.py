import serial
import time
import threading
import numpy as np
import os
class test(object):
    def __init__(self,port):
        self.serial_pc=serial.Serial(port=port,baudrate=115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
        self.f=open("data_leg_detector.csv","r",os.O_NONBLOCK)
    def transfer_data(self):
        while True:
            data=self.f.readlines()
            if not data:
                print "no data"
                time.sleep(0.1)
                continue
            data=data[-1]
            data=data.split(',')
            data=[float(i) for i in data]
            print data
            vel=str(round(data[2],2))
            cad=str(round(data[1],2))
            sl=str(round(data[0],2))
            #Adquirir un byte
            s = self.serial_pc.read(1)
            print s
            if not s:
                print "paila papu"
                continue
            
            val=ord(s)
            #Si es 0x55 significa que el comptutador solicito una medicion
            if val!=0x55:
                time.sleep(0.05)
                print "wrong init value"
                continue
            print "Begin: {0}".format(hex(val))
            #Enviar 0x0A en caso de recibir la inicializacion
            self.serial_pc.write(chr(0x0A))
            #Enviar la longitud de la velocidad
            self.serial_pc.write(chr(len(vel)))
            #Enviar los datos de velocidad
            for a in vel:
                self.serial_pc.write(a)
            #Enviar la longitud de la cadencia
            self.serial_pc.write(chr(len(cad)))
            #enviar el valor de la cadencia
            for a in cad:
                self.serial_pc.write(a)
            #Enviar la longitud de la longitud de paso
            self.serial_pc.write(chr(len(sl)))
            #Enviar el valor de la longitud de paso
            for a in sl:
                self.serial_pc.write(a)
            #solicitar condicion de parada
            s = self.serial_pc.read(1)
            if not s:
                print "paila papu dos"
                continue
            val=ord(s)
            print "Data sended: {0}".format(hex(val))
            #si la condicion es 0xAA, la adquisicion se detiene
            if val==0xAA:
                self.serial_pc.close()
                self.f.close()
                break
    def shutdown(self):
        try:
            self.serial_pc.close()
        except:
            pass
        try:
            self.f.close()
        except:
            pass
if __name__=='__main__':
    print "Waiting"
    time.sleep(10)
    #Cambiar el puerto
    a=test(port="/dev/serial0")
    #Iniciar la comunicacion
    t = threading.Thread(target = a.transfer_data)
    t.start()
    #a.transfer_data()
    print "transfer finished..."
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        a.shutdown()
