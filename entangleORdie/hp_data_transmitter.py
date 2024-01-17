import socket

#transmit hp to computer by reading it from a file that the main program creates

# Define the IP address and port of the computer
computer_ip = "192.168.158.43"
computer_port = 5005	  # Choose any available port number

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
message = 'test'

while True:
	f = open("data_trans.txt", "r")
	message = f.read()
	f.close()
	print(message)
	# Send the message to the computer
	sock.sendto(message.encode(), (computer_ip, computer_port))
