#! /usr/bin/env python

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

#LED setup
# init gpio, number of lights and brightness
pixels1 = neopixel.NeoPixel(board.D18, 30, brightness =1)

#button setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# UDP for communication with other pi
UDP_IP = "192.168.1.201"
UDP_PORT = 5005
MESSAGE = "idle"

# accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

# nfc
pn532 = PN532_I2C(debug=False, reset=20, req=16)
ic, ver, rev, support = pn532.get_firmware_version()
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
pn532.SAM_configuration()
print('Waiting for RFID/NFC card...')

# variables
buttonPressed = 0
buttonCount = 0
ledBrightness_new = 0
ledBrightness_old = 0
accelerometerXYZ = []
stateEntangled = 0 # state 0 means the pillos are not entangled, state 1 means they are entangled
# -------------------------------Main Function-----------------------------
def color_button():
    global ledBrightness_old
    global ledBrightness_new

    if ledBrightness_new != ledBrightness_old:
        pixels1.fill((100*ledBrightness_new))
        ledBrightness_old = ledBrightness_new
    
def color_flip():
    if accelerometerXYZ[2] < -8:
        pixels1.fill((ledBrightness_new*10, 0, 0))
        return "RED"
    elif accelerometerXYZ[2] > 8:
        pixels1.fill((0, 0, ledBrightness_new*10))
        return "BLUE"
    else:
        return "FLIPPING"
    

def squeeze_control():
    global buttonCount
    global ledBrightness_old
    global ledBrightness_new
    global buttonPressed

    if buttonCount >= 20:
        ledBrightness_old = ledBrightness_new
        ledBrightness_new = (ledBrightness_new+1)%2
        buttonCount = 0
    if buttonPressed == 0:
        buttonCount += 1
    else:
        buttonCount = 0

def nfc_read():
    uid = pn532.read_passive_target(timeout=0.01)
    # print('.', end="")
    # Try again if no card is available.
    if uid is None:
        return
    return [hex(i) for i in uid]
    # print('Found card with UID:', [hex(i) for i in uid])

def udp_send():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))

while True:

    accelerometerXYZ = accelerometer.acceleration
    buttonPressed = GPIO.input(17)
    squeeze_control()
    print("button count: ", buttonCount)
    color_button()
    color = color_flip()

    # check for quantum entaglement (touch) only if both have color OR they already are entangled
    if ledBrightness_new == 1 or stateEntangled == 1:
        if nfc_read() != None:
            stateEntangled = (stateEntangled+1)%2
    else:
        time.sleep(0.1)
    
    if stateEntangled == 1:
        MESSAGE = color
    else:
        MESSAGE = "idle"
    print(MESSAGE)
    udp_send()
    print(stateEntangled)
