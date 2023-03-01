import socket
import threading

bind_ip = "10.209.218.204" # Replace this with your own IP address
bind_port = 27700 # Feel free to change this port
# create and bind a new socket
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((bind_ip, bind_port))
server.listen(5)
print("Server is listening on %s:%d" % (bind_ip, bind_port))

def clientHandler(client_socket):
    # send a message to the client
    client_socket.send("ready".encode())
    # receive and display a message from the client
    request = client_socket.recv(1024)
    print("Received \"" + request.decode() + "\" from client")
    # close the connection again
    client_socket.close()
    print("Connection closed")

while True:
    # wait for client to connect
    client, addr = server.accept()
    print("Client connected " + str(addr))
    # create and start a thread to handle the client
    client_handler = threading.Thread(target = clientHandler, args=(client,))
    client_handler.start()