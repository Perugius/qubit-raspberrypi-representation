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
# pn532 = PN532_I2C(debug=False, reset=20, req=16)
# ic, ver, rev, support = pn532.get_firmware_version()
# print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
# pn532.SAM_configuration()
# print('Waiting for RFID/NFC card...')

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
#dropping
dropCount = 0
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

    if buttonCount >= 200:
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
    return 1 #[hex(i) for i in uid]

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
    

print(n1)
print(n2)
print(n1tilde)
print(n2tilde)
# while True:
#     accelerometerXYZ = accelerometer.acceleration
#     flipCheck()
#     squeezeCheck()
#     buttonPress = GPIO.input(17)

#     if squeezed:
#         combinedState = np.dot(h1, combinedState)
#         squeezed = False
#     if flipped:
#         combinedState = np.dot(x1, combinedState)
#         flipped = False
#     #print(flipCount)
#     print(flipped)
#     print(combinedState)
