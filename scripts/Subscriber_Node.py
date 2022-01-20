#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
 

class Listener:

    def __init__(self):
        self.sub = rospy.Subscriber('chatter', String, self.callback)
        #rospy.spin()

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('listener')
    listener = Listener()
