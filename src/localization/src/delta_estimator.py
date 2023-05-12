#!/usr/bin/env python
### Estimates deltas from controls input
import rospy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

delta_publisher = None

def callback(data):

    vel = 0.5263 #ft/s
    angular = 216.86 #deg/s
    dt = 0.1

    x = data.linear.x
    z = data.angular.z

    dTheta = 0
    dPos = 0

    # Only read rotation if controls are coming in
    if abs(z) > 0.1:
        dTheta = angular * dt

    # Only read velocity when controls are coming in
    if abs(x) > 0.01:
        if x > 0:
            dPos = vel * dt
        else:
            dPos = -vel * dt

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

    