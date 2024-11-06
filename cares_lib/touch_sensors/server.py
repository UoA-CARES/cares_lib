import socket
import serial
import threading
import time
from concurrent.futures import ThreadPoolExecutor

class Sensor(object):
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.values = []
        self.running = False
        self.server_ready = False
        self.timeout = 5

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
            if not self.running:
                break
            elapsed_time = time.time() - start_time
            packet = serial_connection.readline().decode('utf-8').strip()
            if packet:
                if "Pressure Baselines" in packet or elapsed_time > self.timeout and not self.server_ready:
                    print("Sensor preamble complete or timeout reached.")
                    self.server_ready = True
                values = packet.split(" ")
                if len(values) < 2:
                    print("Received incomplete data:", packet)
                    continue
                else:
                    try:
                        self.values = [float(value) for value in values if value]
                    except ValueError:
                        print("Received non-numeric data:", packet)
            serial_connection.reset_input_buffer()
        

    def stop(self):
        self.running = False
        self.read_thread.join(2)
        self.serialInst.close()

class Server(Sensor):
    def __init__(self, port, baudrate, host='localhost', socket_port=65432):
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
        print(f"Server strated and ready on {self.host}:{self.socket_port}")
    
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


        # while self.running:
        #     try:
        #         data = client_socket.recv(1024).decode('utf-8')
        #         if data == "i":
        #             response = str(self.values)  # Send the first two values
        #             client_socket.sendall(response.encode('utf-8'))
        #             client_socket.shutdown(socket.SHUT_RDWR)
        #             client_socket.close()
        #             break
        #         elif data == "quit":
        #             self.stop()
        #             client_socket.shutdown(socket.SHUT_RDWR)
        #             client_socket.close()
        #             break
        #     except (socket.timeout, ConnectionResetError):
        #         client_socket.close()
        #         break

    def stop(self):
        self.running = False