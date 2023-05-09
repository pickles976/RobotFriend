#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import os
import cv2
import numpy as np
np.float = float # monkey patch
import ros_numpy
from scipy.spatial.transform import Rotation as R

pose_pub = None

last_pose = None

def callback_delta(data):

    global pose_pub
    pose_pub.publish(data) # Just write the fiducial pose into reality

def callback_pose(data):

    global pose_pub
    global last_pose

    # Apply pose delta to last pose (it's just a transformation matrix)

if __name__ == '__main__':

    print("Starting node...")

    rospy.init_node('pose_estimator', anonymous=True)

    rospy.Subscriber("geometry_msgs/pose_fiducial", PoseStamped, callback_pose)
    rospy.Subscriber("geometry_msgs/deltas", TwistStamped, callback_delta)
    pose_pub = rospy.Publisher('geometry_msgs/pose_estimate', PoseStamped, queue_size = 10)

    rospy.spin()