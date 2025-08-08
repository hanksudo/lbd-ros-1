#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback_receive_radio_data(msg: String):
    rospy.loginfo(f"message received : {msg.data}")

if __name__ == "__main__":
    rospy.init_node("first_subscriber")

    sub = rospy.Subscriber("/pub", String, callback_receive_radio_data)

    rospy.spin()
