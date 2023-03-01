import socket

target_host = "10.209.218.204" # Change this to the IP address of your server
target_port = 27700 # Change this to the port of your server

# create a socket
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(1)

# connect to the server
client.connect((target_host, target_port))
# receive
response = client.recv(4096)
if response.decode() == "ready":
    print("Successful")
else:
    print("Not successful")
# send
client.send("hello world".encode())