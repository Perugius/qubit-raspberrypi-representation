import socket
import time
#server script
port = 12345

sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.bind(('192.168.1.201', port))
sock.listen(1)
MESSAGE = "TESTING123"
while True:
    c, addr = sock.accept()
    print("sending")
    while True:
        c.send(MESSAGE.encode())
   # c.send('connected'.encode())
    #data = c.recv(1024)
        print("sent")
        time.sleep(0.5)
    print("closing")
    c.close()
    #print(data)
