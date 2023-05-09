#!/usr/bin/env python
### Reads messages from controller and reads IMU, 
### Processes IMU data and publishes estimated position deltas
from imu_reader import IMUReader
import rospy
from geometry_msgs.msg import Twist
import rospy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

delta_publisher = None
imu = None

translating = False
rotating = False

def callback(data):

    global imu
    global delta_publisher
    global translating
    global rotating

    x = data.linear.x
    z = data.angular.z
    
    vel = 0
    rot = 0
    dt = imu.dt

    # Only read rotation if controls are coming in
    if abs(z) > 0.1:
        if rotating:
            rot = imu.angle
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


    # delta_publisher.pub()


if __name__ == '__main__':

    # Initialize connection to IMU module
    imu = IMUReader('/dev/ttyACM0')
    imu.initialize_connection()

    print("Starting node...")
    rospy.init_node('imu_reader', anonymous=True)
    rospy.Subscriber("velocity_controller/cmd_vel", Twist, callback)

    delta_publisher = rospy.Publisher('geometry_msgs/deltas', PoseStamped, queue_size = 10)

    while not rospy.is_shutdown():
        imu.update()

    