#!/usr/bin/env python
import sys
from time import sleep
import pigpio

RIGHT_SERVO=12
MIN_WIDTH=1000
MAX_WIDTH=2000

pi = pigpio.pi()

print("sleeping for 5s")
sleep(5)

if not pi.connected:
   exit()

while True:
    for i in range(0, 10, 1):
        pulse = MIN_WIDTH + (100 * i)
        pi.set_servo_pulsewidth(RIGHT_SERVO, pulse)
        print(pulse)
        sleep(2.5)