#! /usr/bin/env python

# client script to run on raspberry WITHOUT nfc reader
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
import RPi.GPIO as GPIO
#---------------------------------------------------Initializations---------------------------------------------

#TCP setup
TCP_SERVER = "192.168.1.200"
TCP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_SERVER, TCP_PORT))

#LED setup
# init gpio, number of lights and brightness
pixels1 = neopixel.NeoPixel(board.D18, 30, brightness =1)

#button setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)


#sockSend = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sockSend.bind((UDP_IP_SEND, UDP_PORT_SEND))

# accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

#var
accelerometerXYZ = []
buttonPress = 0
buttonCount = 0
ledBrightness_new = 0
ledBrightness_old = 0
stateEntangled = 0
MESSAGE = "NONO"
deviceState = "OFF1"
#----------------------------------------------------Main Function---------------------------------------

def colorfn(color):
    if color == "RED1":
        pixels1.fill((10, 0, 0))
    if color == "BLUE":
        pixels1.fill((0, 0, 10))
    if color == "OFF1":
        pixels1.fill((0, 0, 0))

def squeeze_control():
    global buttonPress
    global buttonCount
    global ledBrightness_new
    global ledBrightness_old
    
    if buttonCount >= 15:
        ledBrightness_old = ledBrightness_new
        ledBrightness_new = (ledBrightness_new+1)%2
        buttonCount = 0
    if buttonPress == 0:
        buttonCount = buttonCount + 1
    else:
        buttonCount = 0

while True:

    accelerometerXYZ = accelerometer.acceleration
    buttonPress = GPIO.input(17)
    squeeze_control()

    try:
        data = sock.recv(4)
        data = data.decode()
        sock.send(deviceState.encode())
        if data == '':
            break
        print("data received: ", data)
    finally:
        print("comm received!")

    if ledBrightness_new == 1:
        if accelerometerXYZ[2] < -8:
            deviceState = "RED1"
        if accelerometerXYZ[2] > 8:
            deviceState = "BLUE"
    else:
        deviceState = "OFF1"

    # color control that checks if entangled or not based on message received
    if data == 'NONO':
        colorfn(deviceState)
    else:
        colorfn(data)

    time.sleep(0.1)