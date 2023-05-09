#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import numpy as np
np.float = float # monkey patch
from scipy.spatial.transform import Rotation as R

pose_pub = None

last_pose = None

def callback_delta(data):

    global pose_pub
    pose_pub.publish(data) # Just write the fiducial pose into reality

def callback_pose(data):

    global pose_pub
    global last_pose

    # Apply delta to last pose (it's just a transformation matrix)
    rotation = np.array(data.pose.orientation, dtype=np.float32)
    rotation = R.from_quat(rotation)
    rotation = R.as_matrix(rotation)

    translation = np.array(data.position).reshape(3)

    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    if last_pose == None:
        last_pose = transform
        return

    last_pose = np.dot(last_pose, transform)

    # Create pose message
    translation = last_pose[:3,3]
    rotation = last_pose[:3,:3]
    r = R.from_matrix(rotation)
    quat = R.as_quat(r)

    p = PoseStamped()

    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()

    p.pose.position.x = translation[0]
    p.pose.position.y = translation[1]
    p.pose.position.z = translation[2]
    p.pose.orientation.x = quat[0]
    p.pose.orientation.y = quat[1]
    p.pose.orientation.z = quat[2]
    p.pose.orientation.w = quat[3]

    pose_pub.publish(p) # Update pose estimate



if __name__ == '__main__':

    print("Starting node...")

    rospy.init_node('pose_estimator', anonymous=True)

    rospy.Subscriber("geometry_msgs/pose_fiducial", PoseStamped, callback_pose)
    rospy.Subscriber("geometry_msgs/deltas", TwistStamped, callback_delta)
    pose_pub = rospy.Publisher('geometry_msgs/pose_estimate', PoseStamped, queue_size = 10)

    rospy.spin()