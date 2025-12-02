#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

if __name__ == "__main__":
    rospy.init_node("number_publisher", anonymous=True)

    pub = rospy.Publisher("/number", Int64, queue_size=10)

    # rosparam set /number_publish_frequency 1
    publish_frequency = rospy.get_param("/number_publish_frequency")
    rate = rospy.Rate(publish_frequency)

    # rosparam set /number_to_publish 3
    number = rospy.get_param("/number_to_publish")

    rospy.set_param("/another_param", "Hello")

    while not rospy.is_shutdown():
        msg = Int64()
        msg.data = number
        pub.publish(msg)
        rospy.loginfo("Published: %d", msg.data)
        rate.sleep()