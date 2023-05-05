#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import os
from aruco_tracker import ArucoTracker
import cv2
import numpy as np
np.float = float # monkey patch
import ros_numpy
from scipy.spatial.transform import Rotation as R

method = cv2.SOLVEPNP_ITERATIVE
marker_dict = "./src/fiducials/src/aruco_markers.json"
camera_matrix =  "./src/fiducials/src/camera_matrix.json"
tracker = None
pose_pub = None

def callback(data):
    image = ros_numpy.numpify(data)
    pose_estimates = tracker.getPoseEstimatesFromImage(image, method)

    if len(pose_estimates) < 1:
        return

    trans_corrected = np.array([0,0,0], dtype=np.float32)
    quat_corrected = []

    for pose in pose_estimates:
        trans = pose[:3,3]
        trans = list(map(lambda x: x / 304.80, trans))
        trans = np.array(trans, dtype=np.float32)
        print("Position:")
        print(trans)

        rot = pose[:3,:3]
        r = R.from_matrix(rot)
        quat = R.as_quat(r)
        print("Orientation:")
        print(quat)

        trans_corrected = trans
        quat_corrected = quat

    p = PoseStamped()

    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()

    p.pose.position.x = trans_corrected[0]
    p.pose.position.y = trans_corrected[1]
    p.pose.position.z = trans_corrected[2]
    p.pose.orientation.w = quat_corrected[0]
    p.pose.orientation.x = quat_corrected[1]
    p.pose.orientation.y = quat_corrected[2]
    p.pose.orientation.z = quat_corrected[3]

    global pose_pub
    pose_pub.publish(p) # Send it when ready!

    
def init_node():

    print("Starting node...")

    rospy.init_node('pose_estimator', anonymous=True)

    rospy.Subscriber("camera/image", Image, callback)
    global pose_pub 
    pose_pub = rospy.Publisher('pose', PoseStamped, queue_size = 10)

    rospy.spin()

if __name__ == '__main__':

    # load Aruco Tracker
    print("Initializing Tracker...")
    tracker = ArucoTracker(camera_matrix, marker_dict)

    init_node()