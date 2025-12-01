#!/usr/bin/env python3

import rospy

if __name__ == "__main__":
    rospy.init_node("load_param")
    rospy.loginfo(f"This robot name is : {rospy.get_param('/robot_name')}")
