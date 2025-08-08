#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('first_publisher', anonymous=True)

    pub = rospy.Publisher("/pub", String, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hi, this is from Robot News Radio!"
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Node was stopped")
