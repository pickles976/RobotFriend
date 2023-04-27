# How to use

1. clone repo if first time, otherwise rm -rf build folder
2. source opt/ros/kinetic/setup.bash
3. in repo root run "catkin_make"
4. source devel/setup.sh
5. on Raspberry pi run "rosrun controller subscriber.py"

# Debugging hardware

Servo datasheet
https://docs.rs-online.com/0e85/0900766b8123f8d7.pdf

Servos need to be grounded to the RaspberryPi

Servos need 50hz w/ duty cycle of 5% to 10%
Duty cycle of 7.5% is stopped.

# Chassis
Length = ???
Width = 7.931

Battery dims
L = 13.97cm
W = 6.731cm
H = 1.6256cm

Pi hole dims
4.9cm
5.8cm

cam hole dims
1.34cm
2.0828cm

robot hole dims
6.985cm x 6.985cm

wall thickness
4mm