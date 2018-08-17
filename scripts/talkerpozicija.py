#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from pymultiwii import MultiWii
from sys import stdout
from telemetrija.msg import koordinate


def talkerpozicija():
    pub = rospy.Publisher('pozicija', koordinate, queue_size=10)
    rospy.init_node('letjelica', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    #board = MultiWii("/dev/ttyUSB0")
    board = MultiWii("/dev/virtualcom0")
    while not rospy.is_shutdown():
	board.getData(MultiWii.RAW_IMU)
	ax = board.rawIMU['ax']
	ay = board.rawIMU['ay']
	az = board.rawIMU['az']
	gx = board.rawIMU['gx']
	gy = board.rawIMU['gy']
	gz = board.rawIMU['gz']
	mx = board.rawIMU['mx']
	my = board.rawIMU['my']
	mz = board.rawIMU['mz']
	elapsed = board.attitude['elapsed']
	#message = "ax = {:+.0f} \t ay = {:+.0f} \t az = {:+.0f} gx = {:+.0f} \t gy = {:+.0f} \t gz = {:+.0f} mx = {:+.0f} \t my = {:+.0f} \t mz = {:+.0f} \t elapsed = {:+.4f} \t" .format(float(board.rawIMU['ax']),float(board.rawIMU['ay']),float(board.rawIMU['az']),float(board.rawIMU['gx']),float(board.rawIMU['gy']),float(board.rawIMU['gz']),float(board.rawIMU['mx']),float(board.rawIMU['my']),float(board.rawIMU['mz']),float(board.attitude['elapsed']))
        #rospy.loginfo(message)
        pub.publish(koordinate(ax, ay, az, gx, gy, gz, mx, my, mz, elapsed))
        rate.sleep()

if __name__ == '__main__':
    try:
        talkerpozicija()
    except rospy.ROSInterruptException:
        pass

