#! /usr/bin/env python3
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
#import RPi.GPIO as GPIO

time.sleep(30)


UDP_IP = "192.168.1.203"
UDP_PORT = 5005
count = 0
brightness = 1

#comms
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#leds
pixels1 = neopixel.NeoPixel(board.D18, 30, brightness = 1)
#accel
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

while True:
	data, addr = sock.recvfrom(1024)
	acc = accelerometer.acceleration
	if (data == b"BLUE"):
		print("0")
		pixels1.fill((0,0,40))
	elif (data == b"RED"):
		print("1")
		pixels1.fill((40,0,0))
	elif (data == b"OFF"):
		pixels1.fill((0,0,0))
	else:
		if (acc[2] < -8):
			pixels1.fill((40*brightness, 0, 0))
		elif( acc[2] > 8):
			pixels1.fill((0, 0, 40*brightness))
		if (-2 < acc[0] < 2 and -2 < acc[1] < 2 and -2 < acc[2] < 2):
			count += 1
		if (count >= 5):
			brightness = (brightness + 1)%2
			count = 0
