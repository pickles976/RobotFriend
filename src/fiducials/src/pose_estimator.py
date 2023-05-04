#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import os
from aruco_tracker import ArucoTracker
import cv2
import ros_numpy

method = cv2.SOLVEPNP_ITERATIVE
marker_dict = "./src/fiducials/src/aruco_markers.json"
camera_matrix =  "./src/fiducials/src/camera_matrix.json"
tracker = None

def callback(data):
    print(data)
    image = ros_numpy.numpify(Image, encoding='rgb8')
    pose_estimates = tracker.getPoseEstimatesFromImage(image)
    print(pose_estimates)
    
def listener():

    print("Starting node...")

    rospy.Subscriber("camera/image", Image, callback)
    rospy.init_node('pose_estimator', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # load Aruco Tracker
    print("Initializing Tracker...")
    tracker = ArucoTracker(camera_matrix, marker_dict)

    listener()