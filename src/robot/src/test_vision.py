#!/usr/bin/python
'''
  A Simple mjpg stream http server for the Raspberry Pi Camera
  inspired by https://gist.github.com/n3wtron/4624820
'''
import io
import time
import picamera
from picamera.array import PiRGBArray
from sensor_msgs.msg import Image
import rospy
import numpy as np
import ros_numpy

# (640, 480)
# (1280, 960)
WIDTH, HEIGHT = 320, 240
camera=None
topic = 'picamera/image'
node_name = 'camera'

def talker():

    print('Initializing node: {} with topic "{}"'.format(node_name, topic))
    pub = rospy.Publisher(topic, Image, queue_size=30)
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(10) # 10 hz

    print("Starting camera...")
    camera = picamera.PiCamera()
    camera.resolution = (WIDTH, HEIGHT)
    rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))
    camera.framerate = 30
    camera.rotation = 180
    camera.awb_mode = 'off'
    camera.awb_gains = (1.4, 1.5)
    
    time.sleep(2)


    # print("Starting capture...")
    # while not rospy.is_shutdown():

    #     message = Image()

    #     output = np.empty((HEIGHT*WIDTH*3,),dtype=np.uint8)
    #     camera.capture(output, 'rgb')
    #     output = output.reshape((HEIGHT,WIDTH,3))

    #     message = ros_numpy.msgify(Image, output, encoding='rgb8')

    #     # rospy.loginfo(message)
    #     print("Sending image...")
    #     pub.publish(message)
    #     rate.sleep()

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        output = frame.array

        message = ros_numpy.msgify(Image, output, encoding='rgb8')

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        print("Sending image...")
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass