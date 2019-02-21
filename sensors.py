import threading
import time
import sys
import logging

logging.basicConfig(level = logging.DEBUG, format = '[%(levelname)s] (%(threadName)-9s) %(message)s',)
"""
******************************************************************************************************************
******************************************************************************************************************
*** The [sensor] object is a generic class used to acquire data from any sensor.                               ***
*** This class handles the storage and the sampling of each sensor.                                            ***
*** [sensor] class inherits from the [threading.Thread] object and runs the desired acquisition process.       ***
*** This process (denoted as a function [p]) can be set in three different ways:                               ***
***                                                                                                            ***
*** 1. Passed as an argument.                                                                                  ***
*** 2. Set by the function [set_acquire_process]                                                               ***
*** 3. Override the function [process]                                                                         ***
***                                                                                                            ***
*** The [sensor] object receives the following arguments:                                                      ***
*** - [process] (function): High-order function that will be used in the Thread's run method.                  ***
*** - [name] (String): Name used to label the thread in debug.                                                 ***
*** - [header_file] (String): Header that will be added to the backup file.                                    ***
*** - [file_name] (String): Name of the file which will be used as a backup for the data of any sensor.        ***
*** - [sample_time] (Float): Sampling period used for the downsampling.                                        ***
******************************************************************************************************************
******************************************************************************************************************
"""
class sensor(threading.Thread):
    def __init__(self, group = None,process = None, target = None, name = "sensor-thread", args = (), kwargs = None, verbose = None,header_file ="sensor data: " ,file_name ="sensor_data.csv", sample_time = 1):
        super(sensor, self).__init__(group = group, target = target, name = name, verbose = verbose)
        self.name_csv = file_name
        self.header = header_file
        self.st = sample_time
        self.p = process
        self.go_on = True
        logging.debug("BackUp Name:" + self.name_csv)
        logging.debug("Sample Time" + str(self.st))

        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [run] method is an override to the Thread's main function.                                             ***
        *** [run] also includes the management of the correspondent backup file.                                       ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def run(self):
        self.file = open(self.name_csv,'w+')
        self.file.write(self.header+"\n")
        self.process()
        self.file.close()
        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [process] method executes the incoming high-order function denoted as [p].                             ***
        *** This method is intended to be linked to the sensor's acquisition loops.                                    ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def process(self):
        self.p()
        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [load_data] method manages the data that is being saved into the backup file.                          ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def load_data(self, d):
        #logging.debug("writing data")
        self.file.write(d)
        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [set_acquire_process] recieves the acquisition function that will be executed                          ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def set_acquire_process(self, p):
        self.process = p
        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [set_header_file] recieves the new name for the backup file.                                           ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def set_header_file(self,s):
        self.header = s
        """
        ******************************************************************************************************************
        ******************************************************************************************************************
        *** The [shutdown] method modifies the main process' flag. Which means that the thread will end                ***
        ******************************************************************************************************************
        ******************************************************************************************************************
        """
    def shutdown(self):
        self.go_on =False
"""
******************************************************************************************************************
******************************************************************************************************************
*** The [main] method is an example of a common usage of the [sensor] object                                   ***
******************************************************************************************************************
******************************************************************************************************************
"""
def main():
    #Defining the high-order function that will be used.
    def f():
        logging.debug("sensor function works")
    #Creation the [sensor] object, specifying that the process is the function [f].
    s = sensor(process=f)
    #Starting the thread.
    s.start()
    #Creation of another [sensor] object, in this case, only [header_file] and [file_name] are specified.
    d = sensor(header_file ="ang,velocity,dev", file_name = "my_sensor.csv")
    #using the [set_acquire_process] method to set the [f] method.
    d.set_acquire_process(f)
    #Starting the thread.
    d.start()
    #Creating a class that inherits from the sensor object, in this case the [process] method is overrided.
    class h_sensor(sensor):
        def __init__(self):
            super(h_sensor,self).__init__(name = "h_sensor-thread",header_file ="header h_sensor",file_name = "h_sensor.csv")

        def process(self):
            logging.debug("from object function")
    #Creating a [h_sensor] object.
    h = h_sensor()
    #Starting the thread.
    h.start()
    try:
        while True:
            time.sleep(1)
            pass

    except KeyboardInterrupt:
        s.shutdown()
        d.shutdown()
        h.shutdown()
        raise


if __name__=='__main__':
    main()
