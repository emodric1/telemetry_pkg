#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the topic

import rospy
from std_msgs.msg import String
from telemetrija.msg import koordinate

def callback(data):
    ax = data.ax
    ay = data.ay
    az = data.az
    gx = data.gx
    gy = data.gy
    gz = data.gz
    mx = data.mx
    my = data.my
    mz = data.mz
    elapsed = data.elapsed
    print rospy.get_caller_id(), "I just heard that ax= %f , ay= %f , az= %f , gx= %f , gy = %f , gz= %f , mx= %f , my = %f , mz = %f , elapsed = %f "%(ax, ay, az, gx, gy, gz, mx, my, mz, elapsed)
    

def listenerpozicija():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenerpozicija', anonymous=True)

    rospy.Subscriber('pozicija', koordinate, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listenerpozicija()
