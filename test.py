import time
import board
import neopixel
import busio
import adafruit_adxl34x

#accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

#lights
pixels1 = neopixel.NeoPixel(board.D18, 30, brightness = 1)

#declarations
acc = []
status = []
states = ["static", "flipped", "falling"]
brightness = 1
count = 0
#correction factor for so pillow acts same way no matter what side program 
#is started on
if (accelerometer.acceleration[2] < -8):
	c = -1
else:
	c = 1 	
while True:	
	acc = accelerometer.acceleration
	#flip check
	if (acc[2] < -8*c):
		status = states[1]
		pixels1.fill((brightness*20,0,0))
	elif (acc[2] > 8*c):
		status = states[0]
		pixels1.fill((0,0,brightness*20))
	#falling check
	if ( -2 < acc[0] < 2 and -2 < acc[1] < 2 and -2 < acc[2] < 2):
		status = state[2]
		count += 1
	#check if pillow has been falling for more than 5 clock cycles
	#(just in case)
	if (count >= 5):
		#toggle between 1 and 0, i.e when pillow falls light
		#turns on or off
		brightness = (brightness + 1) % 2
		count = 0
	print(acc)
	time.sleep(0.01)
