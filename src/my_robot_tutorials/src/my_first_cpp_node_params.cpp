#include <ros/ros.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_first_cpp_node_params");
    ros::NodeHandle nh;
    ROS_INFO("Node has been started");

    ros::Rate rate(10);

    std::string robot_name;

    if (nh.getParam("/robot_name", robot_name)) {
        ROS_INFO("Got robot_name: %s", robot_name.c_str());
    } else {
        ROS_WARN("Parameter /robot_name not found");
    }
}

