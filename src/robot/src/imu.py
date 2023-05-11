#!/usr/bin/env python
from imu_reader import IMUReader
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
from math import cos, sin, pi

delta_publisher = None
imu = None

translating = False
rotating = False
prev_rot = 0

def integrate_position(angle, velocity, dt):
    dvel = velocity * dt * 0.3048 # convert to feet
    rad = angle * pi / 180.0
    return [cos(rad) * dvel, sin(rad) * dvel]

def callback(data):

    global imu
    global translating
    global rotating

    x = data.linear.x
    z = data.angular.z
    
    vel = 0
    rot = 0
    dt = imu.dt
    dTheta = 0

    # Only read rotation if controls are coming in
    if abs(z) > 0.1:
        if rotating:
            rot = imu.angle
            dTheta = rot - prev_rot
        else: 
            rotating = True
            imu.rot_bias = 0
    else:
        rotating = False

    # Only read velocity when controls are coming in
    if abs(x) > 0.01:
        if translating:
            vel = imu.velocity
        else: 
            translating = True
            imu.start_reading_accel()
    else:
        translating = False

    print("Velocity %s"%vel)
    print("Angle: %s"%rot)

    dx,dy = integrate_position(rot, vel, dt)

    message = TwistStamped()
    message.header.frame_id = "map" # TODO: wtf are the different coordinate frames?
    message.header.stamp = rospy.Time.now()
    message.twist.angular.z = dTheta * dt
    message.twist.linear.x = dx * dt
    message.twist.linear.y = dy * dt

    global delta_publisher
    print(delta_publisher)
    delta_publisher.pub(message)

def init_node():

    # Initialize connection to IMU module
    global imu
    imu = IMUReader('/dev/ttyACM0')
    imu.initialize_connection()

    print("Starting node...")
    rospy.init_node('imu_reader', anonymous=True)
    rospy.Subscriber("velocity_controller/cmd_vel", Twist, callback)
    global delta_publisher
    delta_publisher = rospy.Publisher('geometry_msgs/deltas', TwistStamped, queue_size = 10)

    while not rospy.is_shutdown():
        imu.update()

if __name__ == '__main__':

    init_node()

    