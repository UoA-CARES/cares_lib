import serial
import threading
import time
import queue

class Sensor(object):
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.max_readings = [0,0]

    def initialise(self):
        self.serialInst = serial.Serial()
        self.serialInst.baudrate = self.baudrate
        self.serialInst.port = self.port
        self.serialInst.open()

        #thread for reading from port
        self.read_thread = threading.Thread(target=self.read_from_port,args=(self.serialInst,))
        self.read_thread.start()
        

    def read_from_port(self, serial_connection):
        while True:
            packet = serial_connection.readline().decode('utf').strip()
            if packet:
                values = packet.split(" ")
                if len(values) < 2:
                    continue
                else:
                    try:
                        values = [float(value) for value in values if value]
                        combined = []
                        for num1, num2 in zip(values, self.max_readings):
                            combined.append(max(num1,num2))
                        self.max_readings = combined
                        # self.pressure_readings = [float(value) for value in values if value]
                    except: 
                        print("Recieved non-numeric data:", packet)


    def get_pressure_readings(self):
        return self.max_readings
    
    def reset_pressure_readings(self):
        self.max_readings = [0,0]
    
    def reboot(self):
        print("Rebooting Arduino...")
        self.stop()
        time.sleep(2)
        self.initialise()
        time.sleep(2)
        print("Finished Reboot")
        print(self.get_pressure_readings())
        print("Values Recieved")
        
    
    def stop(self):
        self.read_thread.join(2)
        print("Exited Cleanly")
        self.serialInst.close()
