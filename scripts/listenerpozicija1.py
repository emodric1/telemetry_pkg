#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Vector3

def callback(data):
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z
    gx = data.angular_velocity.x
    gy = data.angular_velocity.y
    gz = data.angular_velocity.z
    mx = data.orientation.x
    my = data.orientation.y
    mz = data.orientation.z
    print rospy.get_caller_id(), "I just heard that ax= %f , ay= %f , az= %f , gx= %f , gy = %f , gz= %f , mx= %f , my = %f , mz = %f"%(ax, ay, az, gx, gy, gz, mx, my, mz)
    

def listenerpozicija1():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenerpozicija1', anonymous=True)

    rospy.Subscriber('pozicija', Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listenerpozicija1()
