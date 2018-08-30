import socket
 
TCP_IP = '127.0.0.1'
TCP_PORT = 9527
BUFFER_SIZE = 1024
MESSAGE = b"Hello, World!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

while(1):
    #s.send(MESSAGE)
    data = s.recv(BUFFER_SIZE)
    #print("received data:", data)

s.close()

