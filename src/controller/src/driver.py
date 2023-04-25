#!/usr/bin/env python
import RPi.GPIO as GPIO
# from time import sleep

pwmpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pwmpin,GPIO.OUT)
pi_pwm = GPIO.PWM(pwmpin,50)		#create PWM instance with frequency
pi_pwm.start(0)				#start PWM of required Duty Cycle 

# duty cycle range
# 5 -> 7.5 -> 10
# 5 is reverse, 7.5 is stopped, 10 is forward

pi_pwm.ChangeDutyCycle(10)

# while True:
#     for duty in range(0,101,1):
#         pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
#         sleep(0.01)
#     sleep(0.5)
    
#     for duty in range(100,-1,-1):
#         pi_pwm.ChangeDutyCycle(duty)
#         sleep(0.01)
#     sleep(0.5)