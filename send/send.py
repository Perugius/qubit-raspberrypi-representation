import socket
import time
UDP_IP = "192.168.1.203"
UDP_PORT = 5005
MESSAGE = "idle"

while True:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))
	time.sleep(0.2)	


