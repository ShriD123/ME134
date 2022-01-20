#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class Talker:

	def __init__(self):
		self.pub = rospy.Publisher('chatter', String)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			hello_str = 'hello world %s' % rospy.get_time()
			rospy.loginfo(hello_str)
			self.pub.publish(hello_str)
			rate.sleep()


if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('talker')

    talker = Talker()