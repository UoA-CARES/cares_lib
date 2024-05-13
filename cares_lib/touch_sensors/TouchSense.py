import serial
import threading

pressure_readings = []

def read_from_port(serial_connection):
    global pressure_readings
    while True:
        if serial_connection.in_waiting:
            packet = serial_connection.readline().decode('utf').strip()
            if packet:  
                values = packet.split(" ")
                try:
                    pressure_readings = [float(value) for value in values if value]
                    print("Updated pressure readings:", pressure_readings)
                except ValueError:
                    if(packet == "quit"):
                        break
                    print("Received non-numeric data:", packet)
        

def write_to_port(serial_connection):
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

def main():
    serialInst = serial.Serial()
    serialInst.baudrate = 921600
    serialInst.port = "/dev/ttyACM0"
    serialInst.open()
    # thread for reading from port
    thread = threading.Thread(target=read_from_port, args=(serialInst,))
    thread.start()
    # main thread for writing to the port
    write_to_port(serialInst)

    thread.join()
    print("Exited cleanly")
    serialInst.close()

if __name__ == '__main__':
    main()
