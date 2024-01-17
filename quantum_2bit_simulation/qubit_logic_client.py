#! /usr/bin/env python3

# server script to run on raspberry that has nfc reader 
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
import RPi.GPIO as GPIO
import random
import numpy as np

#------------------------------------------Initializations----------------------------------------

#TCP setup
TCP_SERVER = "192.168.168.222"
TCP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_SERVER, TCP_PORT))
MESSAGE = "hello"
#LED setup
# init gpio, number of lights and brightness
pixels1 = neopixel.NeoPixel(board.D18, 60, brightness =1)

#button setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)


#operations as np arrays
xGate = np.array([[0, 1], [1, 0]])
hadamard = np.array([[1, 1], [1, -1]])*(2**(-1/2))
zGate = np.array ([[1, 0], [0, -1]])
cnot12 = np.array([[1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0]])
identity = np.identity(2)
x1 = np.kron(identity, xGate)
x2 = np.kron(xGate, identity)
h1 = np.kron(identity, hadamard)
h2 = np.kron(hadamard, identity)
z1 = np.kron(identity, zGate)
z2 = np.kron(zGate, identity)
ntilde = np.array([[1, 0], [0, 0]])
n = np.array([[0, 0], [0, 1]])
n1tilde = np.kron(identity, ntilde)
n2tilde = np.kron(ntilde, identity)
n1 = np.kron(identity, n)
n2 = np.kron(n, identity)
#state
pillow1State = np.array([1, 0]) #this pillow
pillow2State = np.array([1, 0]) #other pillow
combinedState = np.kron(pillow2State, pillow1State)
hadamardTracker = False
color = "RED1"
#dropping
dropCount = 0
dropped = False
#flipping
accelerometerXYZ = []
flipped = False
pillowside_new = "UP" #keeps track of if pillow is facing up or down to check flipping
pillowside_old = "UP"
flipCount = 0 #keeps track of flips and only changes in default and entangled states
#button
buttonTracker_new = 0 #keeps track of button to check if squeezed or not
buttonTracker_old = 0
buttonCount = 0
buttonPress = 0
squeezeCount = 0
squeezed = False

# -------------------------------Function-----------------------------
def colorfn(color):
    if color == "RED":
        pixels1.fill((10, 0, 0))
    if color == "BLUE":
        pixels1.fill((0, 0, 10))
    if color == "OFF":
        pixels1.fill((0, 0, 0))

def colorSetter(color):

    if hadamardTracker == False:
        if color == "RED1":
            colorfn("RED")
        elif color == "BLUE":
            colorfn("BLUE")
    
    elif hadamardTracker == True:
        colorfn("OFF")

def whiteBlink():
    for i in range(3):
        pixels1.fill((10, 10, 10))
        time.sleep(0.1)
        pixels1.fill((0, 0, 0))
        time.sleep(0.1)

def flipCheck():
    global flipCount
    global pillowside_new
    global pillowside_old
    global accelerometerXYZ
    global flipped

    if pillowside_old != pillowside_new:
        flipCount += 1
        flipped = True
        pillowside_old = pillowside_new

    if accelerometerXYZ[2] < -8:
        pillowside_new = "DOWN"
    elif accelerometerXYZ[2] > 8:
        pillowside_new = "UP"

def squeezeCheck():
    global buttonPress
    global buttonCount
    global buttonTracker_new
    global buttonTracker_old
    global squeezeCount
    global squeezed

    if buttonCount >= 50:
        buttonTracker_old = buttonTracker_new
        buttonTracker_new = (buttonTracker_new+1)%2
        buttonCount = 0
        squeezeCount += 1
        squeezed = True

    if buttonPress == 0:
        buttonCount += 1
    else:
        buttonCount = 0

def fallingCheck():
    global accelerometerXYZ  
    global dropCount
    global dropped

    if dropCount >= 3:
        dropped = True
    if (-2<accelerometerXYZ[0]<2 and -2<accelerometerXYZ[1]<2 and -2<accelerometerXYZ[2]<2):
        dropCount += 1
    else:
        dropCount = 0
    
while True:
    accelerometerXYZ = accelerometer.acceleration
    buttonPress = GPIO.input(17)
    flipCheck()
    squeezeCheck()
    fallingCheck()
    colorSetter(color)

    if flipped:
        whiteBlink()
        MESSAGE = "FLIP"
        flipped = False
    elif squeezed:
        whiteBlink()
        hadamardTracker = True
        MESSAGE = "SQUZ"
        squeezed = False
    elif dropped: 
        whiteBlink()
        hadamardTracker = False
        MESSAGE = "DROP"
        dropped = False
    else:
        MESSAGE = "IDLE"
    
    try:
        #send operation done (i.e X gate , measurement etc. Receice color that should take)
        color = sock.recv(4)
        sock.send(MESSAGE.encode())
        color = color.decode()
    finally:
        print("comm received")

    print(color)
