#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pymultiwii import MultiWii
from sys import stdout
from telemetrija.msg import pid


def talkerpid():
    pub = rospy.Publisher('pid', pid, queue_size=10)
    rospy.init_node('letjelica', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    #board = MultiWii("/dev/ttyUSB0")
    board = MultiWii("/dev/virtualcom0")
    while not rospy.is_shutdown():
	board.getData(MultiWii.PID)
	message = pid()
	message.rp = board.PIDcoef['rp']
        message.ri = board.PIDcoef['ri']
	message.rd = board.PIDcoef['rd']
	message.pp = board.PIDcoef['pp']
        message.pi = board.PIDcoef['pi']
	message.pd = board.PIDcoef['pd']
	message.yp = board.PIDcoef['yp']
        message.yi = board.PIDcoef['yi']
        message.yd = board.PIDcoef['yd']
	
	#message = "ax = {:+.0f} \t ay = {:+.0f} \t az = {:+.0f} gx = {:+.0f} \t gy = {:+.0f} \t gz = {:+.0f} mx = {:+.0f} \t my = {:+.0f} \t mz = {:+.0f} \t elapsed = {:+.4f} \t" .format(float(message.linear_acceleration.x),float(message.linear_acceleration.y),float(message.linear_acceleration.z),float(message.angular_velocity.x),float(message.angular_velocity.y),float(message.angular_velocity.z),float(message.orientation.x),float(message.orientation.y),float(message.orientation.z))
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talkerpid()
    except rospy.ROSInterruptException:
        pass

