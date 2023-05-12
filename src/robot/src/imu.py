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
dt = 0.1

def integrate_position(velocity, dt):
    dvel = velocity * dt * 0.3048 # convert to feet
    return dvel

def callback(data):

    global imu
    global translating
    global rotating
    global dt

    x = data.linear.x
    z = data.angular.z
    
    vel = 0
    dTheta = 0

    # Only read rotation if controls are coming in
    if abs(z) > 0.1:
        if rotating:
            dTheta = imu.dTheta
        else: 
            rotating = True
            imu.rot_bias = 0
    else:
        rotating = False

    # Only read velocity when controls are coming in
    if abs(x) > 0.01:
        if translating:
            if x > 0:
                vel = imu.velocity
            else:
                vel = -imu.velocity
        else: 
            translating = True
            imu.start_reading_accel()
    else:
        translating = False

    print("Velocity %s"%vel)
    print("dTheta: %s"%dTheta)

    # calulate linear displacement
    dPos = integrate_position(vel, dt)

    message = TwistStamped()
    message.header.frame_id = "map" # TODO: wtf are the different coordinate frames?
    message.header.stamp = rospy.Time.now()
    message.twist.angular.z = dTheta
    message.twist.linear.x = dPos

    global delta_publisher
    print(delta_publisher)
    delta_publisher.publish(message)

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

    