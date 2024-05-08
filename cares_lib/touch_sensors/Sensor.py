import serial
import threading
import time
class Sensor(object):
    def __init__(self, type, port, baudrate):
        self.type = type
        self.port = port
        self.baudrate = baudrate
        self.pressure_readings = []

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
                try:
                    self.pressure_readings = [float(value) for value in values if value]
                    # print("Updated pressure reading:", self.pressure_readings)
                    # print(f"this is what: {serial_connection}")
                except:
                    if (packet == "quit"):
                        self.stop()
                    print("Recieved non-numeric data:", packet)

    def get_pressure_readings(self):
        return self.pressure_readings
    
    def stop(self):
        self.read_thread.join()
        self.write_thread.join()
        print("Exited Cleanly")
        self.serialInst.close()