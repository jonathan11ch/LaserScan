import serial
import time
import numpy as np
import sensors
class LRF_Node(sensors.sensor):
    def __init__(self,port,Ts=1):
        self.serial_pi=serial.Serial(port=port,baudrate=4800,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
        self.go_on=True
        self.Ts=Ts
    def process(self):
        while self.go_on:
            #Escribir byte de inicio
            self.serial_pi.write(0x55)
            #adquirir respuesta
            val=ord(self.serial_pi.read(1))
            #si el resultado es 0x0A, significa que la raspberry recibio el mensaje
            if val!=0x0A:
                time.sleep(0.05)
                continue
            #pedir la longitud de la velocidad
            val=ord(self.serial_pi.read(1))
            #adquirir los datos de la velocidad
            vel=self.serial_pi.read(val)
            #pedir la longitud de la cadencia
            val=ord(self.serial_pi.read(1))
            #adquirir los datos de la cadencia
            cad=self.serial_pi.read(val)
            #pedir la longitud de la longitud de paso
            val=ord(self.serial_pi.read(1))
            #adquirir los datos de longitud de paso
            sl=self.serial_pi.read(val)
            #mostrar los datos en consola...
            print "vel: "+str(vel)
            print "cadence: "+str(cad)
            print "step length: "+str(sl)
            #submuestreo...
            time.sleep(self.Ts)
    def shutdown(self):
        self.go_on=False
        time.sleep(2*self.Ts)
        #Enviar finalizacion de la comunicacion...
        self.serial_pi.write(0xAA)
        #cerrar el puerto serial...
        self.serial_pi.close()
if __name__=='__main__':
    a=LRF_Node(port='COMX')
    a.start()
    try:
        while True:
            time.sleep(0.1)
    except:
        a.shutdown()
