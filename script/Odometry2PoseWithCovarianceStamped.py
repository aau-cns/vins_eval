#!/usr/bin/env python

import sys
import argparse
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class translator:
	def __init__(self):
		self.pub = rospy.Publisher('/pose_est', PoseWithCovarianceStamped, queue_size = 1)
		self.sub = rospy.Subscriber('/odom_est', Odometry, self.callback, queue_size = 1)

	def callback(self, msg_in):
		msg_out = PoseWithCovarianceStamped()
		msg_out.header.seq = msg_in.header.seq
		msg_out.header.stamp = msg_in.header.stamp
		msg_out.pose.pose.position.x = msg_in.pose.pose.position.x
		msg_out.pose.pose.position.y = msg_in.pose.pose.position.y
		msg_out.pose.pose.position.z = msg_in.pose.pose.position.z
		msg_out.pose.pose.orientation.x = msg_in.pose.pose.orientation.x
		msg_out.pose.pose.orientation.y = msg_in.pose.pose.orientation.y
		msg_out.pose.pose.orientation.z = msg_in.pose.pose.orientation.z
		msg_out.pose.pose.orientation.w = msg_in.pose.pose.orientation.w
		msg_out.pose.covariance = msg_in.pose.covariance
		self.pub.publish(msg_out)
        
def main(args):
	tr = translator()
	rospy.init_node('translator', anonymous=True)
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
    main(sys.argv)
