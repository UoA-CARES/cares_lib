import serial
import threading

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
        self.thread = threading.Thread(target=self.read_from_port, args=(self.serialInst,))
        self.thread.start()

        #main thread for writing to port
        self.write_to_port(self.serialInst)

    def read_from_port(self, serial_connection):
        while True:
            if serial_connection.in_waiting:
                packet = serial_connection.readline().decode('utf').strip()
                if packet:
                    values = packet.split(" ")
                    try:
                        self.pressure_readings = [float(value) for value in values if value]
                        print("Updated pressure reading:", self.pressure_readings)
                    except:
                        if (packet == "quit"):
                            self.stop()
                        print("Recieved non-numeric data:", packet)

    def write_to_port(self, serial_connection):
        while True:
            command = input()
            if command == 'reboot':
                serial_connection.write(command.encode())
            if command == 'exit':
                serial_connection.write(command.encode())
                break
            if not command:
                command = 'ping'
                serial_connection.write(command.encode())

    def stop(self):
        self.thread.join()
        print("Exited Cleanly")
        self.serialInst.close()
