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
#------------------------------------------Initializations----------------------------------------

# wait for systemctl
# time.sleep(30)

#TCP setup
# TCP_SERVER = "192.168.1.200" # maybe "127.0.0.1" better?
# TCP_PORT = 12345
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.bind((TCP_SERVER, TCP_PORT))
# sock.listen(1)
# c, addr = sock.accept()

#LED setup
# init gpio, number of lights and brightness
pixels1 = neopixel.NeoPixel(board.D18, 30, brightness =1)

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

#var
buttonPress = 0
buttonCount = 0
accelerometerXYZ = []
ledBrightness_new = 0
ledBrightness_old = 0
nfcDelay = 0 #read nfc every x clock cycles, this variable is the delay of x clock cycles
ledNormalizer = 1 #used to start as red no matter what side the pillow is held on
state = "default"
ledColor = "RED1"
oldColor = "RED1" #keeps track of old color so we can see when pillow flipped
flipCount = 0 #keeps track of flips and only changes in default state
internalColor = 0 #flip count before the going into superposition
dropCount = 0 #count for checking how long pillow has been dropping
dropped = False #keeps track of if pillow has been dropped
randomColor = [0, 1] #0 for even(red) 1 for odd(blue) when dropping to choose randomly 
otherPillow = 'DEFO' #message from other pillow which says which state pillow is in, DEFO = default, SUPE = superposition ,etc
thisPillow = 'DEFO' #message to send to other pillow to show what state current pillow is in

# -------------------------------Function-----------------------------
def colorfn(color):
    if color == "RED1":
        pixels1.fill((10, 0, 0))
    if color == "BLUE":
        pixels1.fill((0, 0, 10))
    if color == "OFF1":
        pixels1.fill((0, 0, 0))

#check if pillow has been flipped and increment flipCount
def flipCheck():
    global oldColor
    global ledColor
    global flipCount
    global accelerometerXYZ

    if oldColor != ledColor:
        flipCount += 1
        oldColor = ledColor

    if accelerometerXYZ[2] * ledNormalizer < -8:
        ledColor = "BLUE"
    elif accelerometerXYZ[2] * ledNormalizer > 8:
        ledColor = "RED1"

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
        buttonCount += 1
    else:
        buttonCount = 0

def nfc_read():
    uid = pn532.read_passive_target(timeout=0.01)
    # Try again if no card is available.
    if uid is None:
        return
    return 1 #[hex(i) for i in uid]
    # print('Found card with UID:', [hex(i) for i in uid])

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
    

#----------------------------Main Program-------------------------------
#start color is red
colorfn(ledColor)
if accelerometer.acceleration[2] < -8:
    ledNormalizer = -1

    
while True:
    #readings
    accelerometerXYZ = accelerometer.acceleration
    buttonPress = GPIO.input(17)
    squeeze_control()
    fallingCheck() 
    nfcDelay += 1

    print(state)
    print(dropCount)
    #print("led b new: ", ledBrightness_new)
    #print('led b old:',  ledBrightness_old)
    time.sleep(0.01)
    #-------------------------------default state--------------------------
    if state == 'default':
        thisPillow = 'DEFO'
        #check if pillow was flipped
        flipCheck()
        #set color
        if flipCount%2 == 0:
            colorfn("RED1")
        else:
            colorfn("BLUE")

        #check if pillow has been squeezed -> enter superposition
        if ledBrightness_new != ledBrightness_old:
            ledBrightness_old = ledBrightness_new
            internalColor = flipCount
            state = "superposition"

        #check if pillow has been dropped -> assign random color
        if dropped:
          flipCount = random.choice(randomColor)
          dropped = False

        #check if other pillow is in superposition and if so check for nfc contant. If there is then go into entanglement
        if otherPillow == "SUPE":
            if nfcDelay >= 10:
                nfcDelay = 0
                if nfc_read():
                    state = 'entangled'

    #----------------------------superposition----------------------
    if state == 'superposition':
        ledColor = "OFF1"
        colorfn(ledColor)

        #dropping in superposition -> take random color and go to default again
        if dropped:
            state = 'default'
            flipCount = random.choice(randomColor)
            dropped = False

        #other pillow has color and contant is made -> entangled
        if otherPillow == "DEFO":
            if nfcDelay >=10:
                nfcDelay = 0
                if nfc_read():
                    state = 'entangled'

    #------------------------entangled-----------------------------------
    if state == 'entangled':
        print(state)
        