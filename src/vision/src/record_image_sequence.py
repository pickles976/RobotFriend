#!/usr/bin/python
from picamera import PiCamera
from time import sleep
import rospy
from geometry_msgs.msg import Twist
import json

messages = []

def callback(data):
    global messages
    messages.append(str(data))
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("velocity_controller/cmd_vel", Twist, callback)

if __name__ == '__main__':

    listener()

    WIDTH, HEIGHT = 1280, 960

    camera = PiCamera()
    camera.resolution = (WIDTH, HEIGHT)
    camera.rotation = 180
    camera.framerate = 10
    camera.start_preview()
    sleep(1)

    for filename in camera.capture_continuous('img{counter:03d}.jpg',use_video_port=True):
        print('Captured %s' % filename)
        with open('controls.json', 'w') as f:
            json.dump({ "inputs" : messages}, f)
        sleep(0.1) # wait 0.1s

camera.stop_preview()

