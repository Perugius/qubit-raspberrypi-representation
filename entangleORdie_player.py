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

#------------------------------------------Initializations----------------------------------------

#TCP setup
TCP_SERVER = "192.168.104.106" # maybe "127.0.0.1" better?
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
#nfc
nfcDelay = 0
#game specific
startTime = time.time()
HP = 20
redDebuff = False
greenBuff = False
temp_redDebuff = False
temp_greenBuff = False
pillow2measured = False
# -------------------------------Function-----------------------------
def colorfn(color):
    if color == "RED":
        pixels1.fill((10, 0, 0))
    if color == "BLUE":
        pixels1.fill((0, 10, 0))
    if color == "OFF":
        pixels1.fill((0, 0, 0))

def vector2color():
    global hadamardTracker
    global combinedState
    global MESSAGE
    global thisPillow

    if np.allclose(combinedState, np.array([1, 0, 0, 0])):
        thisPillow = "RED"
        MESSAGE = "RED1"
    elif np.allclose(combinedState, np.array([0, 1, 0, 0])):
        thisPillow = "BLUE"
        MESSAGE = "RED1"
    elif np.allclose(combinedState, np.array([0, 0, 1, 0])):
        thisPillow = "RED"
        MESSAGE = "BLUE"
    elif np.allclose(combinedState, np.array([0, 0, 0, 1])):
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

    if hadamardTracker == False:
        colorfn(thisPillow)
    elif hadamardTracker == True:
        colorfn("OFF")

def whiteBlink():
    for i in range(3):
        pixels1.fill((10, 10, 10))
        time.sleep(0.1)
        pixels1.fill((0, 0, 0))
        time.sleep(0.1)

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

    if dropCount >= 3:
        dropped = True
        dropCount = 0
    if (-3<accelerometerXYZ[0]<3 and -3<accelerometerXYZ[1]<3 and -3<accelerometerXYZ[2]<3):
        dropCount += 1
    else:
        dropCount = 0
    
def measurement(pillow):
    global combinedState
    global hadamardTracker
    global redDebuff
    global greenBuff
    global temp_greenBuff
    global temp_redDebuff

    if pillow == 1:
        hadamardTracker = False
        P1red = combinedState.conjugate()@n1tilde@combinedState
        P1blue = combinedState.conjugate()@n1@combinedState
        color1Select = np.random.choice(np.arange(2), p=(P1red, P1blue))
        #red color selected pillow 1
        if color1Select == 0:
            redDebuff = True
            combinedState = n1tilde@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)
        #blue color selected pillow 1
        if color1Select == 1:
            greenBuff = True
            combinedState = n1@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)

    if pillow == 2:
        P2red = combinedState.conjugate()@n2tilde@combinedState
        P2blue = combinedState.conjugate()@n2@combinedState
        color2Select = np.random.choice(np.arange(2), p=(P2red, P2blue))
        #red color selected pillow 2
        if color2Select == 0:
            temp_redDebuff = True
            combinedState = n2tilde@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)
        #blue color selected pillow 2
        if color2Select == 1:
            temp_greenBuff = True
            combinedState = n2@combinedState
            combinedState = combinedState/np.linalg.norm(combinedState)

def HP_calculation(hp):
    global redDebuff
    global greenBuff
    global temp_redDebuff
    global temp_greenBuff

    if redDebuff == True:
        hp -= 10
        redDebuff = False
        time.sleep(2)
    elif greenBuff == True:
        hp += 10
        greenBuff = False
        time.sleep(2)

    if temp_redDebuff == True:
        hp -= 10
        temp_redDebuff = False
    elif temp_greenBuff == True:
        hp += 10
        temp_greenBuff = False

    return hp

def round_reset():
    global combinedState
    global hadamardTracker
    global temp_greenBuff
    global temp_redDebuff
    global greenBuff
    global redDebuff
    
    combinedState = np.array([0.5, 0.5, 0.5, 0.5])
    hadamardTracker = True
    temp_greenBuff = False
    temp_redDebuff = False

round_reset()

while True:
    elapsedStartTime = time.time() - startTime
    print(combinedState)
    print(temp_greenBuff)
    print(temp_redDebuff)
    print(HP)
    if elapsedStartTime >= 60:
        break

    if HP == 0:
        print("YOU DIED")
        break
    accelerometerXYZ = accelerometer.acceleration
    fallingCheck()
    vector2color()
    nfcDelay += 1
    #TCP communication
    c.send(MESSAGE.encode())
    pillow2op = c.recv(4)
    pillow2op = pillow2op.decode()

    #if we measure second pillow then wait 3 seconds we reset state
    if pillow2measured and time.time()-roundElapsedTime>3:
        pillow2measured = False
        round_reset()

    #if drop player pillow then all the hp is calculated
    if dropped:
        whiteBlink()
        measurement(1)
        vector2color()
        HP = HP_calculation(HP)
        dropped = False
        round_reset()

    #if we drop the second pillow the potential hp changed are calculated but not actualized
    if pillow2op == "DROP":
        pillow2measured = True
        measurement(2)
        roundElapsedTime = time.time()
    
    #if we touch we go into entanglement
    if nfcDelay >= 100:
        nfcDelay = 0
        if nfc_read():
            whiteBlink() 
            combinedState = np.array([0.70710678, 0, 0, 0.70710678])



print("HP: ", HP)

