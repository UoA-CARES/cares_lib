import socket
import serial
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import re

class Sensor(object):
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.values = []
        self.running = False
        self.server_ready = False
        self.timeout = 6
        self.baseline_values = [0,0,0,0]
        self.max_values = [0,0,0,0]
        self.error = False

    def initialise(self):
        self.serialInst = serial.Serial()
        self.serialInst.baudrate = self.baudrate
        self.serialInst.port = self.port
        self.serialInst.open()
        # Start a thread for reading from the serial port
        self.running = True
        self.read_thread = threading.Thread(target=self.read_from_port, args=(self.serialInst,))
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_from_port(self, serial_connection):
        start_time = time.time()
        while True:
            try:
                if not self.running:
                    break
                elapsed_time = time.time() - start_time
                packet = serial_connection.readline().decode('utf-8').strip()
                if packet:
                    if "Pressure Baselines" in packet or elapsed_time > self.timeout and not self.server_ready:
                        print("Sensor preamble complete or timeout reached.")
                        self.server_ready = True
                        numbers = [float(num) for num in re.findall(r"[-+]?\d*\.\d+|\d+", packet)]
                        self.baseline_values = [num for num in numbers if num != 0.0]
                    values = packet.split(" ")
                    if len(values) < 4:
                        # print("Received incomplete data:", packet)
                        continue
                    else:
                        try:
                            self.values = [float(value) for value in values if value] 
                            if len(self.values) != len(self.max_values):
                                continue
                            else:
                                self.max_values = [max(self.values[i], self.max_values[i]) for i in range(len(self.values))]
                        except ValueError:
                            print("Received non-numeric data:", packet)
            except serial.SerialException as e:
                print("Error reading from serial port:", e)
                self.error = True
                break
            serial_connection.reset_input_buffer()
        
    def reset_max_values(self):
        self.max_values = [0.0] * len(self.max_values)

    def stop(self):
        self.running = False
        self.read_thread.join(2)
        self.serialInst.close()

class Server(Sensor):
    def __init__(self, port, baudrate, socket_port, host='localhost'):
        super().__init__(port, baudrate)
        self.host = host
        self.socket_port = socket_port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.socket_port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(2)
        self.executor = ThreadPoolExecutor(max_workers=5)
        self.running = True

    def start(self):
        self.server_ready = False
        print("Starting Sensors...")
        self.initialise()  # Initialize the sensor
        print("Sensor preamble in progress...")
        while not self.server_ready:
            time.sleep(0.5)
        print(f"Server started and ready on {self.host}:{self.socket_port}")
    
        try:
            while self.running:
                try:
                    client_socket, addr = self.server_socket.accept()
                    client_socket.settimeout(2)
                    print(f"Connected by {addr}")
                    self.executor.submit(self.handle_client, client_socket)
                except socket.timeout:
                    continue
                except OSError as e:
                    if not self.running:
                        break
                    print("Error accepting connection:", e)
                
        finally:
            print("Exiting Server...")
            self.server_socket.close()
            time.sleep(1)
            self.executor.shutdown(wait=True, cancel_futures=True)
            print("Server Exited")
            print("Exiting Sensors...")
            super().stop()
            print("Sensors Exited")

    def handle_client(self, client_socket):
        while self.running:
            try:
                response = str(self.values)  # Send the first two values
                client_socket.sendall(response.encode('utf-8'))
                client_socket.shutdown(socket.SHUT_RDWR)
                client_socket.close()
                break
            except (socket.timeout, ConnectionResetError):
                client_socket.close()
                break

    def stop(self):
        self.running = False
        self.server_socket.close()
        super().stop()