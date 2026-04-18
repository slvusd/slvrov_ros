import board
import busio
import time
from adafruit_pca9685 import PCA9685

ESC_CHANNEL = 5
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
pca = PCA9685(i2c)
pca.channels[ESC_CHANNEL].duty_cycle = 1500
time.sleep(3)
print("BEEP")
for i in range (20):
	pca.channels[ESC_CHANNEL].duty_cycle = 1000 + (i * 50)
	time.sleep(.2)
	print(f"PWM = {1000 + (i * 50)}")


