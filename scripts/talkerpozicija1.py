#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pymultiwii import MultiWii
from sys import stdout
from sensor_msgs.msg import Imu
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Vector3


def talkerpozicija1():
    pub = rospy.Publisher('pozicija', Imu, queue_size=10)
    rospy.init_node('letjelica', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    #board = MultiWii("/dev/ttyUSB0")
    board = MultiWii("/dev/virtualcom0")
    while not rospy.is_shutdown():
	board.getData(MultiWii.RAW_IMU)
	message = Imu()
        message.header.frame_id = "/map"
        message.header.stamp = rospy.Time.now()
	message.orientation_covariance[0] = -1
        message.angular_velocity_covariance[0] = -1
	message.linear_acceleration_covariance[0] = -1
	message.linear_acceleration.x = board.rawIMU['ax']
        message.linear_acceleration.y = board.rawIMU['ay']
	message.linear_acceleration.z = board.rawIMU['az']
	message.angular_velocity.x = board.rawIMU['gx']
        message.angular_velocity.y = board.rawIMU['gy']
	message.angular_velocity.z = board.rawIMU['gz']
	message.orientation.w = 1
        message.orientation.x = board.rawIMU['ax']*3.14159265/180
        message.orientation.y = board.rawIMU['ay']*3.14159265/180
        message.orientation.z = board.rawIMU['az']*3.14159265/180
	
	#elapsed = board.attitude['elapsed']
	#message = "ax = {:+.0f} \t ay = {:+.0f} \t az = {:+.0f} gx = {:+.0f} \t gy = {:+.0f} \t gz = {:+.0f} mx = {:+.0f} \t my = {:+.0f} \t mz = {:+.0f} \t elapsed = {:+.4f} \t" .format(float(message.linear_acceleration.x),float(message.linear_acceleration.y),float(message.linear_acceleration.z),float(message.angular_velocity.x),float(message.angular_velocity.y),float(message.angular_velocity.z),float(message.orientation.x),float(message.orientation.y),float(message.orientation.z))
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talkerpozicija1()
    except rospy.ROSInterruptException:
        pass

