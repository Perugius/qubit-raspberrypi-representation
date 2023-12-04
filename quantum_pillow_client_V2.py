#! /usr/bin/env python3

# client script to run on raspberry without nfc reader
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
import RPi.GPIO as GPIO
import random
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
buttonPress = 0
buttonCount = 0
accelerometerXYZ = []
ledBrightness_new = 0
ledBrightness_old = 0
squeezeCount = 0
ledNormalizer = 1 #used to start as red no matter what side the pillow is held on
state = "default"
ledColor = "RED1"
oldColor = "RED1" #keeps track of old color so we can see when pillow flipped
flipCount = 0 #keeps track of flips and only changes in default state
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

    #TCP communication
    try:
        otherPillow = sock.recv(4)
        sock.send(thisPillow.encode())
        otherPillow = otherPillow.decode()
    finally:
        ("comm received")
    
    print(otherPillow)
    print(state)
    time.sleep(0.1)
    #-------------------------------default state--------------------------
    if state == 'default':
        thisPillow = 'DEFO'
        #check if pillow was flipped and squeezed  and save color every cycle
        squeezeCheck()
        flipCheck()
        internalColor = flipCount
        #set color
        if flipCount%2 == 0:
            colorfn("RED1")
        else:
            colorfn("BLUE")

        #check if pillow has been squeezed -> enter superposition
        if ledBrightness_new != ledBrightness_old:
            ledBrightness_old = ledBrightness_new
            state = "superposition"

        #check if pillow has been dropped -> assign random color
        if dropped:
          flipCount = random.choice(randomColor)
          dropped = False

        #check if server pillow is sending entangled state message -> go into entangled state
        if otherPillow == 'ENTA':
            previousState = state
            state = 'entangled'

     #----------------------------superposition----------------------
    if state == 'superposition':
        thisPillow = 'SUPE'
        ledColor = "OFF1"
        oldColor = ledColor
        colorfn(ledColor)

        #dropping in superposition -> take random color and go to default again
        if dropped:
            state = 'default'
            flipCount = random.choice(randomColor)
            dropped = False

        #other pillow has color and contant is made -> entangled
        if otherPillow == 'ENTA':
            previousState = state
            state = 'entangled'

    #------------------------entangled-----------------------------------
    if state == 'entangled':
        oldColor = ledColor
        #check and send if previous color red or blue so server pillow can adjust accordingly
        if internalColor%2 == 0:
            thisPillow = 'ENT0'
        else:
            thisePillow = 'ENT1'
        #thisPillow = 'ENTA'
        #count times flipped and squeezed
        flipCheck()
        squeezeCheck()

        #if other pillow notices contact when squeezes and flips are even, disentangle and set to previous state
        if  otherPillow == "DIS0":
            state = previousState
            print(state)
            flipCount = internalColor
        
        #DIS1 TO DIS3 are condition of disentanglement when squeeze is odd, described in server pillow
        if otherPillow == "DIS1":
            state = 'default'
            flipCount = 0
            squeezeCount = 0
        if otherPillow == "DIS2":
            state = 'default'
            flipCount = 1
            squeezeCount = 0
        if otherPillow == "DIS3":
            state = 'default'
            flipCount = internalColor + 1
            squeezeCount = 0
        

    #----------TCP-----------
    # try:
    #     otherPillow = sock.recv(4)
    #     sock.send(thisPillow.encode())
    #     otherPillow = otherPillow.decode()
    # finally:
    #     ("comm received")
    
