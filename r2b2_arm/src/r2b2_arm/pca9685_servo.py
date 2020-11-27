#!/usr/bin/env python3

# This example moves a servo its full range (180 degrees by default) and then back.
import time

from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# from adafruit_servokit import ServoKit




# This example also relies on the Adafruit motor library available here:
# https://github.com/adafruit/Adafruit_CircuitPython_Motor
from adafruit_motor import servo

i2c = busio.I2C(SCL, SDA)

# kit = ServoKit(channels=16)


# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.

# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2480)

# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2400)

# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)

# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2600)

# The pulse range is 1000 - 2000 by default.
servo_0 = servo.Servo(pca.channels[0])

# for i in range(180):
#     print(f"Angle: {i}")
#     servo_1.angle = i
# for i in range(180):
#     servo_1.angle = 180 - i

def move_to(srvo, angle):
    cur_angle = round(srvo.angle)
    direction = 1 if cur_angle < angle else -1
    print(f"cur: {cur_angle}, tar: {angle}, dir: {direction}")
    for i in range(cur_angle, angle, direction):
        print(f"Angle: {i}")
        srvo.angle = i


angles = [0, 180, 90]
delay_sec = 2.

# Adjust pulse width range for 0 to 180 deg
servo_0.set_pulse_width_range(550, 2550)

for i in angles:
    move_to(servo_0, i)
    time.sleep(delay_sec)

pca.deinit()
