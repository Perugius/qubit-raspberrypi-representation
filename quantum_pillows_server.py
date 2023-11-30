#! /usr/bin/env python

# server script to run on raspberry that has nfc reader 
import time
import board
import neopixel
import busio
import adafruit_adxl34x
import socket
import RPi.GPIO as GPIO
from pn532 import *
#------------------------------------------Initializations----------------------------------------

# wait for systemctl
# time.sleep(30)

#TCP setup
TCP_SERVER = "192.168.1.200" # maybe "127.0.0.1" better?
TCP_PORT = 12345
MESSAGE = "NONO"
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
accelerometerXYZ = []
buttonPress = 0
buttonCount = 0
ledBrightness_new = 0
ledBrightness_old = 0
nfcDelay = 0 #read nfc every x clock cycles, this variable is the delay of x clock cycles
stateEntangled = 0
deviceState = "OFF1"
deviceEntangledState = "OFF1"
# -------------------------------Main Function-----------------------------

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

def nfc_read():
    uid = pn532.read_passive_target(timeout=0.01)
    # Try again if no card is available.
    if uid is None:
        return
    return 1 #[hex(i) for i in uid]
    # print('Found card with UID:', [hex(i) for i in uid])

# controls the color of both devices when entangled by changing deviceEntangledState
# input the color of each device
def entangled_controller(currentDevice, otherDevice):
    global deviceEntangledState

    #check if currentDevice (color) changed then change entangled color, then check if otherDevice changed...
    #if currentDevice != deviceEntangledState:
    #    deviceEntangledState = currentDevice
    #elif otherDevice != deviceEntangledState:
    #    deviceEntangledState = otherDevice 
    
    deviceEntangledState = currentDevice

while True:
    #readings
    accelerometerXYZ = accelerometer.acceleration
    buttonPress = GPIO.input(17)
    squeeze_control()
    nfcDelay = nfcDelay + 1

    c.send(MESSAGE.encode())
    data = c.recv(4)
    data = data.decode()
    print("data: ", data)

    #non entangled color control
    if ledBrightness_new == 1:
        if accelerometerXYZ[2] < -8:
            deviceState = "RED1"
        if accelerometerXYZ[2] > 8:
            deviceState = "BLUE"
    else:
        deviceState = "OFF1"

    #entagle check (touch check)
    if nfcDelay >= 5:
        nfcDelay = 0
        if (ledBrightness_new == 1 and data != "OFF1") or stateEntangled == 1:
            if nfc_read() == 1:
                ledBrightness_new = 0
                deviceState = "OFF1"
                stateEntangled = (stateEntangled+1)%2

    #if entangled send the color to the other device otherwise send NO state to signify non entanglement
    if stateEntangled == 1:
        entangled_controller(deviceState, data)
        colorfn(deviceEntangledState)
        MESSAGE = deviceEntangledState
    else:
        colorfn(deviceState)
        MESSAGE = "NONO"


    print("device state / entangled state ", deviceState, stateEntangled)
    time.sleep(0.1)

c.close()