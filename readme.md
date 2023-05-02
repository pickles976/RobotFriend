# How to use

## First time setup
1. clone repo
2. source opt/ros/kinetic/setup.bash
3. follow instructions for subsequent setup

## Subsequent
1. bash initialize.bash
2. source devel/setup.sh
3. sudo pigpiod
4. on Raspberry pi run "rosrun controller driver.py"

# Debugging hardware

Servo datasheet
https://docs.rs-online.com/0e85/0900766b8123f8d7.pdf

Servos need to be grounded to the RaspberryPi

Servos need 50hz w/ duty cycle of 5% to 10%
Duty cycle of 7.5% is stopped.

# Chassis Dimensions
Length = ???
Width = 7.931cm

Battery dims
L = 13.97cm
W = 6.731cm
H = 1.6256cm

Pi hole dims
4.9cm
5.8cm

robot hole dims
6.985cm x 6.985cm

wall thickness
4mm

# Fiducial Info:

10 Aruco fiducials, 8cm wide

ID- X,Y,Z
0 - 0,0,1
1 - 1,0,1
2 - 0,0,2
3 - -11ft 9 in, 7ft, 1
4 - 

rosrun image_view image_view image:=/camera/image