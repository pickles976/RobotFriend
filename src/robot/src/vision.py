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

camera=None
topic = 'picamera/image'
node_name = 'camera'

def talker():

    print('Initializing node: {} with topic "{}"'.format(node_name, topic))
    pub = rospy.Publisher(topic, Image, queue_size=10)
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(10) # 10hz

    print("Starting camera...")
    camera = picamera.PiCamera()
    camera.resolution = (1280, 960)
    camera.framerate = 10
    camera.rotation = 180

    while not rospy.is_shutdown():

        message = Image()

        output = np.empty((960*1280*3,),dtype=np.uint8)
        # output = np.empty((960, 1280, 3), dtype=np.uint8)
        camera.capture(output, 'rgb')
        output = output.reshape((960,1280,3))

        message = ros_numpy.msgify(Image, output, encoding='rgb8')

        # rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass