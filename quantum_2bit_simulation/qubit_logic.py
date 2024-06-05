#! /usr/bin/env python3

# server script to run on raspberry that has nfc reader 
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
import RPi.GPIO as GPIO
from pn532 import *
import random
import numpy as np
import sys
import select

#------------------------------------------Initializations----------------------------------------

#TCP setup
TCP_SERVER = "192.168.178.22" # maybe "127.0.0.1" better?
TCP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_SERVER, TCP_PORT))
sock.listen(1)
c, addr = sock.accept()
MESSAGE = "TESTING"

#LED setup
# init gpio, number of lights and brightness
pixels1 = neopixel.NeoPixel(board.D18, 60, brightness =1)

#button setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

# nfc
pn532 = PN532_I2C(debug=False, reset=20, req=16)
ic, ver, rev, support = pn532.get_firmware_version()
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
pn532.SAM_configuration()
print('Waiting for RFID/NFC card...')

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
B_0 = -(xGate + zGate)/np.sqrt(2)
B_1 = (xGate - zGate)/np.sqrt(2)
e0, v0 = np.linalg.eigh(B_0)
e1, v1 = np.linalg.eigh(B_1)
gate0 = np.kron(v0, identity)
gate1 = np.kron(v1, identity)
#state
pillow1State = np.array([1, 0]) #this pillow
pillow2State = np.array([1, 0]) #other pillow
combinedState = np.kron(pillow2State, pillow1State)
combinedState_previous = combinedState
pillow2op = "IDLE" #other pillow sends operation done
hadamardTracker = False #false = no hadamard gate applied yet, true = hadamard has been applied once at least
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
#nfc
nfcDelay = 0

# -------------------------------Function-----------------------------
def colorfn(color):
    if color == "RED":
        pixels1.fill((10, 0, 0))
    if color == "BLUE":
        pixels1.fill((0, 0, 10))
    if color == "OFF":
        pixels1.fill((0, 0, 0))

def vector2color():
    global hadamardTracker
    global combinedState
    global MESSAGE
    global thisPillow

    if np.allclose(abs(combinedState), np.array([1, 0, 0, 0])):
        thisPillow = "RED"
        MESSAGE = "RED1"
    elif np.allclose(abs(combinedState), np.array([0, 1, 0, 0])):
        thisPillow = "BLUE"
        MESSAGE = "RED1"
    elif np.allclose(abs(combinedState), np.array([0, 0, 1, 0])):
        thisPillow = "RED"
        MESSAGE = "BLUE"
    elif np.allclose(abs(combinedState), np.array([0, 0, 0, 1])):
        thisPillow = "BLUE"
        MESSAGE = "BLUE"
    elif np.allclose(abs(combinedState), np.array([0.70710678, 0, 0, 0.70710678])) or np.allclose(abs(combinedState), np.array([0, 0.70710678, 0.70710678, 0])) or np.allclose(abs(combinedState), np.array([0.5, 0.5, 0.5, 0.5])):
        thisPillow = "OFF"
        MESSAGE = "OFF1"
    elif np.allclose(abs(combinedState), np.array([0.70710678, 0.70710678, 0, 0])):
        thisPillow = "OFF"
        MESSAGE = "RED1"
    elif np.allclose(abs(combinedState), np.array([0, 0, 0.70710678, 0.70710678])):
        thisPillow = "OFF"
        MESSAGE = "BLUE"
    elif np.allclose(abs(combinedState), np.array([0, 0.70710678, 0, 0.70710678])):
        thisPillow = "BLUE"
        MESSAGE = "OFF1"  
    elif np.allclose(abs(combinedState), np.array([0.70710678, 0,  0.70710678, 0])):
        thisPillow = "RED"
        MESSAGE = "OFF1"
    elif np.allclose(abs(combinedState), np.array([0, 0.92387953, 0, 0.38268343])) or np.allclose(abs(combinedState), np.array([0, 0.38268343, 0, 0.92387953])):
        thisPillow = "BLUE"
        MESSAGE = "OFF1"  
    elif np.allclose(abs(combinedState), np.array([0.92387953, 0,  0.38268343, 0])) or np.allclose(abs(combinedState), np.array([0.38268343, 0, 0.92387953, 0])):
        thisPillow = "RED"
        MESSAGE = "OFF1"
    

    if hadamardTracker == False:
        colorfn(thisPillow)
    elif hadamardTracker == True:
        colorfn("OFF")

def whiteBlink():
    for i in range(3):
        pixels1.fill((10, 10, 10))
        time.sleep(0.05)
        pixels1.fill((0, 0, 0))
        time.sleep(0.05)

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

    if buttonCount >= 20:
        buttonTracker_old = buttonTracker_new
        buttonTracker_new = (buttonTracker_new+1)%2
        buttonCount = 0
        squeezeCount += 1
        squeezed = True

    if buttonPress == 0:
        buttonCount += 1
    else:
        buttonCount = 0

def nfc_read():
    uid = pn532.read_passive_target(timeout=0.01)
    # Try again if no card is available.
    if uid is None:
        return
    return [hex(i) for i in uid]

def fallingCheck():
    global accelerometerXYZ
    global dropCount
    global dropped

    if dropCount >= 6:
        dropped = True
        dropCount = 0
    elif (-3<accelerometerXYZ[0]<3 and -3<accelerometerXYZ[1]<3 and -3<accelerometerXYZ[2]<3):
        dropCount += 1
    else:
        dropCount = 0
    
def measurement(pillow):
    global combinedState
    global hadamardTracker

    if pillow == 1:
        hadamardTracker = False
        P1red = combinedState.conjugate()@n1tilde@combinedState
        P1blue = combinedState.conjugate()@n1@combinedState
        color1Select = np.random.choice(np.arange(2), p=(P1red, P1blue))
        #red color selected pillow 1
        if color1Select == 0:
            combinedState = n1tilde@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)
        #blue color selected pillow 1
        if color1Select == 1:
            combinedState = n1@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)

    if pillow == 2:
        P2red = combinedState.conjugate()@n2tilde@combinedState
        P2blue = combinedState.conjugate()@n2@combinedState
        color2Select = np.random.choice(np.arange(2), p=(P2red, P2blue))
        #red color selected pillow 2
        if color2Select == 0:
            combinedState = n2tilde@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)
        #blue color selected pillow 2
        if color2Select == 1:
            combinedState = n2@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)




while True:
    #TCP communication
    c.send(MESSAGE.encode())
    pillow2op = c.recv(4)
    pillow2op = pillow2op.decode()

    accelerometerXYZ = accelerometer.acceleration
    buttonPress = GPIO.input(17)
    flipCheck()
    squeezeCheck()
    fallingCheck()
    vector2color()
    nfcDelay += 1


    if squeezed:
        whiteBlink()    
        hadamardTracker = True
        combinedState = np.dot(h1, combinedState)
        squeezed = False
    if flipped:
        whiteBlink()
        combinedState = np.dot(x1, combinedState)
        flipped = False
    if dropped:
        whiteBlink()
        measurement(1)
        dropped = False

    if nfcDelay >= 100:
        nfcDelay = 0
        if nfc_read():
            #check if this pillow is entangled and only send entg message if yes
            time.sleep(1)
            whiteBlink() 
            combinedState = cnot12@combinedState
            if np.allclose(abs(combinedState), np.array([0.70710678, 0, 0, 0.70710678])) or np.allclose(abs(combinedState), np.array([0, 0.70710678, 0.70710678, 0])) or np.allclose(abs(combinedState), np.array([0.5, 0.5, 0.5, 0.5])):
                MESSAGE = "ENTG"


    if pillow2op == "SQUZ":
        combinedState = np.dot(h2, combinedState)
    if pillow2op == "FLIP":
        combinedState = np.dot(x2, combinedState)
    if pillow2op == "DROP":
        measurement(2)
    if pillow2op == "GAT1":
        print("gate2")
        combinedState = np.dot(gate0, combinedState)
    if pillow2op == "GAT2":
        combinedState = np.dot(gate1, combinedState)

    print(combinedState)
