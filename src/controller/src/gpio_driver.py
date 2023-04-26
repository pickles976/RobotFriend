#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep

pwmpin = 32				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pwmpin,GPIO.OUT)
pi_pwm = GPIO.PWM(pwmpin,50)		#create PWM instance with frequency
pi_pwm.start(5)				#start PWM of required Duty Cycle 

# duty cycle range
# 5 -> 7.5 -> 10
# 5 is reverse, 7.5 is stopped, 10 is forward

while True:
    for duty in range(0,10,1):
        d = 5 + (duty * 0.5)
        pi_pwm.ChangeDutyCycle(d) #provide duty cycle in the range 0-100
        sleep(5)