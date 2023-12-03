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
TCP_SERVER = "192.168.1.200" # maybe "127.0.0.1" better?
TCP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_SERVER, TCP_PORT))
sock.listen(1)
c, addr = sock.accept()

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
squeezeCount = 0 #count amount of times pillow has been squeezed
nfcDelay = 0 #read nfc every x clock cycles, this variable is the delay of x clock cycles
ledNormalizer = 1 #used to start as red no matter what side the pillow is held on
state = "default"
ledColor = "RED1"
oldColor = "RED1" #keeps track of old color so we can see when pillow flipped
flipCount = 0 #keeps track of flips and only changes in default and entangled states
internalColor = 0 #flip count before the going into superposition
previousState = 'default' #keep previous state for disentangling
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

def squeezeCheck():
    global buttonPress
    global buttonCount
    global ledBrightness_new
    global ledBrightness_old
    global squeezeCount

    if buttonCount >= 15:
        ledBrightness_old = ledBrightness_new
        ledBrightness_new = (ledBrightness_new+1)%2
        buttonCount = 0
        squeezeCount += 1
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
    fallingCheck() 
    nfcDelay += 1

    #TCP communication after at the end of loop!
    # c.send(thisPillow.encode())
    # otherPillow = c.recv(4)
    # otherPillow = otherPillow.decode()


    print(otherPillow)
    print("squeeze count ", squeezeCount)
    print("prev", previousState)
    print("flip count ", flipCount)
    #print(dropCount)

    time.sleep(0.1)
    #-------------------------------default state--------------------------
    if state == 'default':
        thisPillow = 'DEFO'
        #check if pillow was flipped and squeezed and save color every cycle
        flipCheck()
        squeezeCheck()
        internalColor = flipCount
        #set color
        if flipCount%2 == 0:
            colorfn("RED1")
        else:
            colorfn("BLUE")

        #check if pillow has been squeezed -> enter superposition
        if squeezeCount % 2 != 0:
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
                    previousState = state
                    state = 'entangled'

    #----------------------------superposition----------------------
    if state == 'superposition':
        thisPillow = "SUPE"
        ledColor = "OFF1"
        oldColor = ledColor
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
                    previousState = state
                    state = 'entangled'

    #------------------------entangled-----------------------------------
    if state == 'entangled':
        oldColor = ledColor
        thisPillow = 'ENTA'

        #count times flipped and squeezed
        flipCheck()
        squeezeCheck()

        #check if flip and squeeze count is even, if they are check for nfc contact to disentangle and set to previous state
        if (squeezeCount%2 == 0) and (flipCount%2 == 0):
            if nfcDelay >= 10:
                nfcDelay = 0
                if nfc_read():
                    squeezeCount = 0
                    state = previousState
                    #set this pillow to NULL, otherwise this pillow will send ENTA so other pillow goes into default then into entanglement instantly again
                    thisPillow = 'NULL'
                    flipCount = internalColor
                    msg = "DIS0"
                    c.send(msg.encode())

        #if squeeze is odd, disentangle and set both pillows to default with opposite states
        #here use the last char of the message to send if flip is even/odd
        #first case is if the other pillow was in superposition second case is this pillow was in sup.
        if (squeezeCount%2 == 1):
            if nfcDelay >= 10:
                nfcDelay = 0
                if nfc_read():
                        squeezeCount = 0
                        state = 'default'
                        thisPillow = 'DEFO'
                    #THIS PILLOW PREVIOUS STATE = DEFAULT | OTHER PILLOW PREV. STATE = SUPERPOSITION
                        if previousState == 'default':
                            if internalColor%2 == 0: #red color previously -> other pillow is going to be red
                                msg = "DIS1"
                            else: #blue color prev. -> other pillow will be blue
                                msg = "DIS2"
                            flipCount = internalColor + 1
                            c.send(msg.encode())
                    #THIS PILLOW PREVIOUS STATE = SUPERPOSITION | OTHER PILLOW PREV. = DEFAULT
                        if previousState == 'superposition':
                            if otherPillow == 'ENT0': #other pillow prev color = red
                                flipCount = 0
                            elif otherPillow == 'ENT1':#other pillow prev color = blue
                                flipCount = 1
                            msg = "DIS3"
                            c.send(msg.encode())
                        
        
        #check if flip and squeeze count
    #-----TCP--------
    c.send(thisPillow.encode())
    otherPillow = c.recv(4)
    otherPillow = otherPillow.decode()
