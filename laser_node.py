import leg_detector as ld
import rasp_pc_com as ptcl

import time, sys, os
import threading 
import multiprocessing
class laser_manager(object):
    def __init__(self,port_lrf="/dev/ttyACM0"):
        self.detector=ld.LegDetector(port=port_lrf)
        self.transmiter=ptcl.test(port="/dev/serial0")
        #self.p1=multiprocessing.Process(target=self.transmiter.transfer_data)
        self.p2=multiprocessing.Process(target=self.detector.process)
    def launch(self):
        self.p2.start()
        time.sleep(10)
        #self.p1.start()
    def shutdown(self):
        self.detector.close()
        self.transmiter.shutdown()
if __name__=="__main__":
    a=laser_manager()
    a.launch()
    for i in range(30):
        time.sleep(1)
        print "time is:"+str(i)
    a.shutdown()
