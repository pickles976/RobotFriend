#!/usr/bin/python
'''
  A Simple mjpg stream http server for the Raspberry Pi Camera
  inspired by https://gist.github.com/n3wtron/4624820
'''
import io
import time
import picamera
from sensor_msgs.msg import Image
import rospy
import numpy as np
import ros_numpy
import cv2

# (640, 480)
WIDTH, HEIGHT = 640, 480
camera=None
topic = 'picamera/image'
node_name = 'camera'

def talker():

    print('Initializing node: {} with topic "{}"'.format(node_name, topic))
    pub = rospy.Publisher(topic, Image, queue_size=10)
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(5) # 5hz

    # print("Starting camera...")
    # camera = picamera.PiCamera()
    # camera.resolution = (WIDTH, HEIGHT)
    # camera.framerate = 5
    # camera.rotation = 180

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
    SCALE = 0.7
    FPS = 10.0

    print("Starting capture...")
    while not rospy.is_shutdown():

        message = Image()

        # output = np.empty((HEIGHT*WIDTH*3,),dtype=np.uint8)
        # camera.capture(output, 'rgb')
        # output = output.reshape((HEIGHT,WIDTH,3))

        # Capture the video frame
        ret, frame = imgCap.read()

        # Resize
        width = int(frame.shape[1] * SCALE)
        height = int(frame.shape[0] * SCALE)
        frame = cv2.resize(frame, (width, height))
        frame = cv2.flip(frame, 0)

        message = ros_numpy.msgify(Image, frame, encoding='rgb8')

        # rospy.loginfo(message)
        print("Sending image...")
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass