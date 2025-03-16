# Put inside utils.
import socket
import server
import time
import threading

def get_values(id, host='127.0.0.1', port=65432):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((host, port))
            client_socket.sendall(id.encode('utf-8'))
            data = client_socket.recv(1024).decode('utf-8')
            return data
    except ConnectionRefusedError:
        return "Failed to connect to the server."
    except ConnectionResetError:
        return "Connection to the server was reset."
    
if __name__ == "__main__":
    print("Starting server...")
    server1 = server.Server(port="/dev/Arduino_Flat", baudrate=921600, socket_port=65432)
    server_thread = threading.Thread(target=server1.start)
    server_thread.daemon = True
    server_thread.start()
    print("Server started in separate thread.")
    while not server1.server_ready:
        time.sleep(0.5)
    try:
        while True:
            user_input = input("Enter 'i' to get values, 'reboot' if stuck or 'quit': ")
            if user_input == "i":
                values = get_values(user_input)
                print(f"Values received: {values}")
            elif user_input == "quit":
                server1.stop()
                server_thread.join()
                time.sleep(2)
                break
            elif user_input == "reboot":
                server1.stop()
                time.sleep(5)
                server1 = server.Server(port="/dev/Arduino_Flat", baudrate=921600, socket_port=65432)
                server_thread = threading.Thread(target=server1.start)
                server_thread.daemon = True
                server_thread.start()
                while not server1.server_ready:
                    time.sleep(1)
    except KeyboardInterrupt:
        server1.stop()
        server_thread.join()
    
