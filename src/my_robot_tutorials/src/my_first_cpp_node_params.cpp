#include <ros/ros.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_first_cpp_node_params");
    ros::NodeHandle nh;
    // Private NodeHandle for private parameters (~)
    ros::NodeHandle nh_private("~");

    ROS_INFO("Node has been started");

    ros::Rate rate(10);

    std::string robot_name;

    // Option 1
    if (nh.getParam("/robot_name", robot_name)) {
        ROS_INFO("Got robot_name: %s", robot_name.c_str());
    } else {
        ROS_WARN("Parameter /robot_name not found");
    }

    // Option 2
    std::string robot_name2 = nh.param<std::string>("/robot_name", "default_robot");
    ROS_INFO("robot_name: %s", robot_name2.c_str());

    // private param with default value
    std::string robot_name3 = nh_private.param<std::string>("robot_name", "default_robot");
    ROS_INFO("robot_name: %s", robot_name3.c_str());

    nh.setParam("/another_param", "hello");
}

