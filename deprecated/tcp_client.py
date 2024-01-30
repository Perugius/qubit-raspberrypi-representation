import socket
#client script

TCP_IP = '192.168.1.200'
TCP_PORT = 12345
BUFFER_SIZE = 1024

MESSAGE = "HELLO"										

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.1.201', TCP_PORT))
while True:
	print("looping")
	try:
		data = sock.recv(1024)
		print(data)
	finally:
		print("data read")
		#sock.close()
	#print(sock.recv(1024))
	#sock.send(MESSAGE.encode())
	#time.sleep(1)

